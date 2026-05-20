#!/usr/bin/env python3
"""
OBB Plastic Detection ROS2 Node
=================================
Publishes detected plastic objects with:
- Pose (position + quaternion orientation)
- Dimensions in mm (dx, dy, dz)
- Classification (class + confidence)

Stabilisation:
- Confidence threshold: 0.7
- Consecutive frames: 5

Visual:
- Live camera feed with OBB boxes and detection info
- Comment out SHOW_DISPLAY to disable camera feed window
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
import pyrealsense2 as rs
from collections import deque
from ultralytics import YOLO
from scipy.spatial.transform import Rotation
import os
import time
import cv2
import threading

# ──────────────────────────────────────────────────────────────────────────────
# CONFIG
# ──────────────────────────────────────────────────────────────────────────────

import os as _os
MODEL_PATH       = _os.path.join(_os.path.dirname(__file__), 'best.pt')
CALIBRATION_FILE = _os.path.join(_os.path.dirname(__file__), 'camera_to_robot_calibration.json')

DEPTH_KERNEL_SIZE    = 11
SMOOTHING_WINDOW     = 5
DEPTH_JUMP_THRESHOLD = 0.05
CONF_THRESHOLD       = 0.7
STABILITY_FRAMES     = 5
GRIP_OFFSET          = 50
APPROACH_OFFSET      = 150

SHOW_DISPLAY = True  # ← comment this out to disable the camera feed window

# ──────────────────────────────────────────────────────────────────────────────
# WORK PLANE
# ──────────────────────────────────────────────────────────────────────────────

def get_table_z(x_mm, y_mm):
    return 0.001841 * x_mm + -0.007761 * y_mm + -185.5378

# ──────────────────────────────────────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────────────────────────────────────

def get_median_depth(depth_frame, cx, cy, k=DEPTH_KERNEL_SIZE):
    h, w = depth_frame.get_height(), depth_frame.get_width()
    half = k // 2
    x0, x1 = max(0, cx - half), min(w, cx + half + 1)
    y0, y1 = max(0, cy - half), min(h, cy + half + 1)
    arr = np.asanyarray(depth_frame.get_data())
    region = arr[y0:y1, x0:x1]
    valid = region[region > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid)) / 1000.0

def deproject(intrinsics, u, v, depth_m):
    return rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth_m)

def camera_to_robot(T, cam_xyz):
    p = np.array([cam_xyz[0], cam_xyz[1], cam_xyz[2], 1.0])
    r = T @ p
    return {"x": float(r[0]), "y": float(r[1]), "z": float(r[2])}

def table_ray_intersect(T, cam_xyz, table_z_m):
    """
    Project the camera ray through cam_xyz onto the table plane (z = table_z_m)
    in robot frame.  This corrects the systematic XY error caused by the
    camera viewing at an angle — instead of using wherever the depth sensor
    happens to hit on the object surface, we follow the ray from the camera
    origin through that surface point until it hits the table, giving the
    true footprint of the object regardless of its height.

    Returns (x_m, y_m) in robot frame metres, or None if ray is parallel to
    the table (shouldn't happen with a downward-angled camera).
    """
    # Camera origin in robot frame  (last column of T, homogeneous dropped)
    O = T[:3, 3]
    # Deprojected surface point in robot frame
    P = (T @ np.array([cam_xyz[0], cam_xyz[1], cam_xyz[2], 1.0]))[:3]
    D = P - O  # ray direction (unnormalised — t is still valid)
    if abs(D[2]) < 1e-6:
        return None  # ray parallel to table
    t = (table_z_m - O[2]) / D[2]
    if t <= 0:
        return None  # intersection would be behind the camera
    return float(O[0] + t * D[0]), float(O[1] + t * D[1])

def angle_to_quaternion(angle_rad):
    """Convert 2D OBB rotation angle to 3D quaternion (rotation around Z axis)."""
    r = Rotation.from_euler('z', angle_rad)
    q = r.as_quat()  # [x, y, z, w]
    return {
        "qx": round(float(q[0]), 6),
        "qy": round(float(q[1]), 6),
        "qz": round(float(q[2]), 6),
        "qw": round(float(q[3]), 6),
    }

def pixels_to_mm(pixels, depth_m, focal_length):
    """Convert pixel dimension to mm using depth and focal length."""
    return (pixels / focal_length) * depth_m * 1000.0

def compute_dz(depth_frame, intrinsics, cx, cy, half_width_px, depth_m):
    """
    Compute bottle dz (diameter) from depth data.
    Returns (dz_mm, 'depth') if successful, or (None, 'failed') if unreliable.
    """
    try:
        edge_x = int(cx + half_width_px)
        edge_x = max(0, min(depth_frame.get_width() - 1, edge_x))

        depth_centre = get_median_depth(depth_frame, cx, cy, k=5)
        depth_edge   = get_median_depth(depth_frame, edge_x, cy, k=5)

        if depth_centre is None or depth_edge is None:
            return None, "failed"

        pt_centre = rs.rs2_deproject_pixel_to_point(
            intrinsics, [cx, cy], depth_centre)
        pt_edge   = rs.rs2_deproject_pixel_to_point(
            intrinsics, [edge_x, cy], depth_edge)

        dist = np.linalg.norm(np.array(pt_centre) - np.array(pt_edge))
        dz   = dist * 2 * 1000  # metres → mm

        if 20.0 <= dz <= 200.0:
            return round(dz, 1), "depth"
        else:
            return None, "failed"

    except Exception:
        return None, "failed"

# ──────────────────────────────────────────────────────────────────────────────
# NODE
# ──────────────────────────────────────────────────────────────────────────────

class OBBDetectorNode(Node):
    def __init__(self):
        super().__init__('obb_detector_node')

        self.publisher_ = self.create_publisher(String, 'plastic_detections', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Shared frame for display thread
        self.display_frame = None
        self.frame_lock    = threading.Lock()
        self.running       = True

        # Load model
        self.get_logger().info(f'Loading OBB model from: {MODEL_PATH}')
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info(f'Model loaded — classes: {self.model.names}')

        # Load calibration
        if not os.path.exists(CALIBRATION_FILE):
            self.get_logger().error(
                f'Calibration file not found: {CALIBRATION_FILE}')
            raise FileNotFoundError(CALIBRATION_FILE)
        with open(CALIBRATION_FILE) as f:
            data = json.load(f)
        self.T = np.array(data["transform_matrix"])

        # Derive table Z from calibration corner heights (average of robot z values)
        cal_pts = data.get("calibration_points", [])
        if cal_pts:
            self.table_z_m = float(np.mean([p["robot"]["z"] for p in cal_pts]))
        else:
            self.table_z_m = 0.035  # fallback: ~35 mm above robot base
        self.get_logger().info(
            f'Calibration loaded — table Z = {self.table_z_m*1000:.1f} mm in robot frame'
        )

        # RealSense setup
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = self.pipeline.start(cfg)

        self.align = rs.align(rs.stream.color)
        depth_profile = profile.get_stream(rs.stream.depth)
        self.intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()

        self.pose_histories    = {}
        self.prev_depths       = {}
        self.detection_counts  = {}
        self.stable_detections = {}

        # Start display thread
        if SHOW_DISPLAY:
            self.display_thread = threading.Thread(
                target=self.display_loop, daemon=True)
            self.display_thread.start()

        self.get_logger().info(
            f'OBB Detector Node ready — publishing to /plastic_detections '
            f'(conf>={CONF_THRESHOLD}, stable>={STABILITY_FRAMES} frames)'
        )

    def display_loop(self):
        """Runs in separate thread — handles cv2 window."""
        while self.running:
            with self.frame_lock:
                frame = self.display_frame.copy() \
                    if self.display_frame is not None else None
            if frame is not None:
                cv2.imshow("OBB Plastic Sorter", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False
                    break
            else:
                time.sleep(0.01)
        cv2.destroyAllWindows()

    def timer_callback(self):
        if not self.running:
            return

        frames  = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()

        if not depth_frame or not color_frame:
            return

        color_img = np.asanyarray(color_frame.get_data())
        results   = self.model(color_img, verbose=False, imgsz=640, device='cpu')[0]

        # Draw OBB boxes on display frame
        display = results.plot()

        seen_this_frame    = set()
        current_detections = {}

        if results.obb is not None:
            for obb_box in results.obb:
                conf = float(obb_box.conf[0])
                if conf < CONF_THRESHOLD:
                    continue

                cls_id   = int(obb_box.cls[0])
                cls_name = self.model.names[cls_id]

                xywhr     = obb_box.xywhr[0].tolist()
                cx, cy    = int(xywhr[0]), int(xywhr[1])
                w_px      = float(xywhr[2])
                h_px      = float(xywhr[3])
                angle_rad = float(xywhr[4])
                angle_deg = float(np.degrees(angle_rad))

                depth_m = get_median_depth(depth_frame, cx, cy)
                if depth_m is None or depth_m <= 0.01 or depth_m > 3.0:
                    continue

                # Spike filter
                prev = self.prev_depths.get(cls_name)
                if prev is not None and \
                        abs(depth_m - prev) > DEPTH_JUMP_THRESHOLD:
                    depth_m = prev
                self.prev_depths[cls_name] = depth_m

                # Camera → robot via table-plane ray intersection
                # This corrects the XY shift caused by the angled camera:
                # instead of using the surface point the depth sensor hit
                # (which is somewhere up the side of a 3D object), we follow
                # the camera ray from that surface point down to the table
                # plane, giving the true XY footprint of the object.
                cam_xyz = deproject(self.intrinsics, cx, cy, depth_m)
                xy = table_ray_intersect(self.T, cam_xyz, self.table_z_m)
                if xy is None:
                    # Fallback to direct transform if ray is parallel to table
                    raw_robot = camera_to_robot(self.T, cam_xyz)
                else:
                    raw_robot = {"x": xy[0], "y": xy[1], "z": self.table_z_m}

                # Temporal smoothing
                if cls_name not in self.pose_histories:
                    self.pose_histories[cls_name] = deque(maxlen=SMOOTHING_WINDOW)
                self.pose_histories[cls_name].append(raw_robot)
                hist = self.pose_histories[cls_name]

                x_smooth = float(np.mean([p["x"] for p in hist])) * 1000
                y_smooth = float(np.mean([p["y"] for p in hist])) * 1000

                # Work plane Z
                z_table    = get_table_z(x_smooth, y_smooth)
                z_approach = z_table + APPROACH_OFFSET
                z_grip     = z_table + GRIP_OFFSET

                # Convert px → mm
                fx    = self.intrinsics.fx
                dx_mm = pixels_to_mm(w_px, depth_m, fx)
                dy_mm = pixels_to_mm(h_px, depth_m, fx)

                # Compute dz from depth
                dz_mm, dz_source = compute_dz(
                    depth_frame, self.intrinsics,
                    cx, cy, w_px / 2, depth_m
                )

                # Quaternion
                quaternion = angle_to_quaternion(angle_rad)

                # Stability indicator
                count      = self.detection_counts.get(cls_name, 0)
                stable_str = "STABLE" if count >= STABILITY_FRAMES \
                    else f"stabilising {count}/{STABILITY_FRAMES}"
                colour = (0, 255, 0) if count >= STABILITY_FRAMES \
                    else (0, 165, 255)

                # Overlay info on display frame
                if SHOW_DISPLAY:
                    # Class + stability
                    cv2.putText(display,
                                f"{cls_name} | {stable_str}",
                                (cx - 80, cy - 52),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 2)
                    # Raw camera-frame XY (before transform) — for comparison
                    cv2.putText(display,
                                f"[cam]  x={cam_xyz[0]*1000:.0f} "
                                f"y={cam_xyz[1]*1000:.0f} "
                                f"z={cam_xyz[2]*1000:.0f}mm",
                                (cx - 80, cy - 34),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 255), 1)
                    # Published robot-frame XY (after ray intersection + smoothing)
                    cv2.putText(display,
                                f"[pub]  x={x_smooth:.0f} "
                                f"y={y_smooth:.0f} "
                                f"z={z_grip:.0f}mm",
                                (cx - 80, cy - 18),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, colour, 2)
                    # Angle + dimensions
                    cv2.putText(display,
                                f"angle={angle_deg:.1f}deg | "
                                f"dz={dz_mm}mm({dz_source})",
                                (cx - 80, cy - 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.40, colour, 1)

                # Build dimensions — only include dz if measurement succeeded
                dimensions = {
                    "dx_mm": round(dx_mm, 1),
                    "dy_mm": round(dy_mm, 1),
                }
                if dz_mm is not None:
                    dimensions["dz_mm"] = dz_mm

                seen_this_frame.add(cls_name)
                current_detections[cls_name] = {
                    "pose": {
                        "position": {
                            "x": round(x_smooth, 1),
                            "y": round(y_smooth, 1),
                            "z": round(z_grip, 1),
                        },
                        "orientation": quaternion,
                    },
                    "dimensions": dimensions,
                    "classification": {
                        "class":      cls_name,
                        "confidence": round(conf, 3),
                    },
                    "debug": {
                        "z_table_mm":    round(z_table, 1),
                        "z_approach_mm": round(z_approach, 1),
                        "angle_deg":     round(angle_deg, 2),
                        "angle_rad":     round(angle_rad, 4),
                        "depth_m":       round(depth_m, 4),
                        "dz_source":     dz_source,
                    }
                }

        # HUD
        if SHOW_DISPLAY:
            status = "PUBLISHING" if any(
                c >= STABILITY_FRAMES
                for c in self.detection_counts.values()
            ) else "Stabilising..."
            colour = (0, 255, 0) if status == "PUBLISHING" \
                else (0, 165, 255)
            cv2.putText(display, status, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)
            cv2.putText(display, "Q = quit", (10, 460),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)

            with self.frame_lock:
                self.display_frame = display

        # Update consecutive frame counts
        all_classes = set(self.detection_counts.keys()) | seen_this_frame
        for cls_name in all_classes:
            if cls_name in seen_this_frame:
                self.detection_counts[cls_name] = \
                    self.detection_counts.get(cls_name, 0) + 1
            else:
                self.detection_counts[cls_name] = 0
                self.stable_detections.pop(cls_name, None)

        # Only publish stable detections
        stable = []
        for cls_name, count in self.detection_counts.items():
            if count >= STABILITY_FRAMES and cls_name in current_detections:
                self.stable_detections[cls_name] = current_detections[cls_name]
                stable.append(self.stable_detections[cls_name])

        if stable:
            msg = String()
            msg.data = json.dumps(stable)
            self.publisher_.publish(msg)
            self.get_logger().info(
                f'Published {len(stable)} stable detection(s): '
                f'{[d["classification"]["class"] for d in stable]}'
            )

    def destroy_node(self):
        self.running = False
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OBBDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()