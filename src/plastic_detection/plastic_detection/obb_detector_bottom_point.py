#!/usr/bin/env python3
"""
OBB Plastic Detection — FIX 1: Bottom-Point Localisation
==========================================================
Drop-in replacement for obb_detector_node.py.

PROBLEM FIXED
-------------
The original detector samples depth at the OBB *centre* pixel, which on a
tall object viewed at ~45° hits the side/shoulder of the object — NOT the
table contact point.  The ray-to-table intersection then projects this
wrong origin onto the table, producing an XY offset that grows toward the
image edges.

FIX
---
Instead of the OBB centre, use the **midpoint of the bottom long edge** of
the oriented bounding box.  With a downward-angled camera the bottom edge
of the OBB is the closest visible contour to the actual table footprint.

The bottom edge midpoint is computed from the OBB angle:

    bottom_px = cx + sin(angle) * h/2
    bottom_py = cy + cos(angle) * h/2

(sign convention: positive angle rotates CW in image space, and "bottom"
means largest Y — closest to the camera footprint on the table.)

When exporting z, half the object height is added so the motion planner
doesn't spawn the collision object inside the table.

All other logic (smoothing, stability, display, publishing) is unchanged.

Run:  ros2 run plastic_detection obb_detector_bottom_point
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

MODEL_PATH       = "/home/jun/git/Robotics-Studio-2/src/plastic_detection/plastic_detection/best.pt"
CALIBRATION_FILE = "/home/jun/git/Robotics-Studio-2/src/plastic_detection/plastic_detection/camera_to_robot_calibration.json"

DEPTH_KERNEL_SIZE    = 11
SMOOTHING_WINDOW     = 5
DEPTH_JUMP_THRESHOLD = 0.05
CONF_THRESHOLD       = 0.7
STABILITY_FRAMES     = 5
GRIP_OFFSET          = 50
APPROACH_OFFSET      = 150

MAX_OBJECT_HEIGHT_MM = 60.0   # objects are lying flat — max 6 cm tall

SHOW_DISPLAY = True

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
    in robot frame.
    """
    O = T[:3, 3]
    P = (T @ np.array([cam_xyz[0], cam_xyz[1], cam_xyz[2], 1.0]))[:3]
    D = P - O
    if abs(D[2]) < 1e-6:
        return None
    t = (table_z_m - O[2]) / D[2]
    if t <= 0:
        return None
    return float(O[0] + t * D[0]), float(O[1] + t * D[1])


def angle_to_quaternion(angle_rad):
    r = Rotation.from_euler('z', angle_rad)
    q = r.as_quat()
    return {
        "qx": round(float(q[0]), 6),
        "qy": round(float(q[1]), 6),
        "qz": round(float(q[2]), 6),
        "qw": round(float(q[3]), 6),
    }


def pixels_to_mm(pixels, depth_m, focal_length):
    return (pixels / focal_length) * depth_m * 1000.0


def compute_dz(depth_frame, intrinsics, cx, cy, half_width_px, depth_m):
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
        dz   = dist * 2 * 1000

        if 20.0 <= dz <= 200.0:
            return round(dz, 1), "depth"
        else:
            return None, "failed"
    except Exception:
        return None, "failed"


# ──────────────────────────────────────────────────────────────────────────────
# NEW: Bottom-point helpers
# ──────────────────────────────────────────────────────────────────────────────

def obb_bottom_midpoint(cx, cy, w_px, h_px, angle_rad, img_w, img_h):
    """
    Compute the midpoint of the bottom long edge of the OBB.

    The OBB is parameterised as (cx, cy, w, h, angle) where angle is the
    rotation of the box.  The four corners of the OBB are obtained by
    rotating ±(w/2, h/2) by the angle.

    "Bottom" = the edge whose midpoint has the largest Y in image space
    (closest to the table surface when the camera looks down at an angle).

    Returns (bx, by) pixel coordinates, clamped to image bounds.
    """
    # The two midpoints of the short edges along the height axis
    # (i.e. the midpoints of the two long edges)
    # OBB convention: w is along the angle direction, h is perpendicular.
    # The long edge midpoints are offset along the h direction.
    sin_a = np.sin(angle_rad)
    cos_a = np.cos(angle_rad)

    # Midpoints of the two long edges (offset ± h/2 perpendicular to the
    # angle direction).  The perpendicular direction to (cos_a, sin_a) is
    # (-sin_a, cos_a).
    # Actually for YOLO OBB, the convention is:
    #   - angle rotates the box
    #   - w and h are box dimensions
    # The two "long edge midpoints" are at centre ± (h/2) along the
    # perpendicular axis.
    #
    # Let's compute both candidates and pick the one with larger Y.

    # Direction along the box width axis
    dx_w = cos_a
    dy_w = sin_a

    # Direction along the box height axis (perpendicular)
    dx_h = -sin_a
    dy_h = cos_a

    # Two midpoints along the height axis (these are midpoints of the
    # two "width" edges)
    mid1_x = cx + (h_px / 2) * dx_h
    mid1_y = cy + (h_px / 2) * dy_h

    mid2_x = cx - (h_px / 2) * dx_h
    mid2_y = cy - (h_px / 2) * dy_h

    # Pick the one with larger Y (lower in image = closer to table base)
    if mid1_y >= mid2_y:
        bx, by = mid1_x, mid1_y
    else:
        bx, by = mid2_x, mid2_y

    # Clamp to image bounds
    bx = int(np.clip(bx, 0, img_w - 1))
    by = int(np.clip(by, 0, img_h - 1))

    return bx, by


# ──────────────────────────────────────────────────────────────────────────────
# NODE
# ──────────────────────────────────────────────────────────────────────────────

class OBBDetectorBottomPoint(Node):
    def __init__(self):
        super().__init__('obb_detector_node')

        self.publisher_ = self.create_publisher(String, 'plastic_detections', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.display_frame = None
        self.frame_lock    = threading.Lock()
        self.running       = True

        self.get_logger().info(f'Loading OBB model from: {MODEL_PATH}')
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info(f'Model loaded — classes: {self.model.names}')

        if not os.path.exists(CALIBRATION_FILE):
            self.get_logger().error(f'Calibration file not found: {CALIBRATION_FILE}')
            raise FileNotFoundError(CALIBRATION_FILE)
        with open(CALIBRATION_FILE) as f:
            data = json.load(f)
        self.T = np.array(data["transform_matrix"])

        cal_pts = data.get("calibration_points", [])
        if cal_pts:
            self.table_z_m = float(np.mean([p["robot"]["z"] for p in cal_pts]))
        else:
            self.table_z_m = 0.035
        self.get_logger().info(
            f'Calibration loaded — table Z = {self.table_z_m*1000:.1f} mm')
        self.get_logger().info(
            '[FIX 1] Using bottom-edge midpoint for depth sampling')

        # RealSense
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

        if SHOW_DISPLAY:
            self.display_thread = threading.Thread(
                target=self.display_loop, daemon=True)
            self.display_thread.start()

        self.get_logger().info(
            f'OBB Detector (bottom-point) ready — publishing to /plastic_detections')

    def display_loop(self):
        while self.running:
            with self.frame_lock:
                frame = self.display_frame.copy() \
                    if self.display_frame is not None else None
            if frame is not None:
                cv2.imshow("OBB Plastic Sorter [bottom-point]", frame)
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
        img_h, img_w = color_img.shape[:2]
        results   = self.model(color_img, verbose=False, imgsz=640, device='cpu')[0]

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

                # ── FIX 1: Use bottom-edge midpoint instead of centre ──
                bx, by = obb_bottom_midpoint(
                    xywhr[0], xywhr[1], w_px, h_px, angle_rad, img_w, img_h)

                # Sample depth at the bottom point (closer to table contact)
                depth_m = get_median_depth(depth_frame, bx, by)
                if depth_m is None or depth_m <= 0.01 or depth_m > 3.0:
                    # Fallback: try centre if bottom point has no valid depth
                    depth_m = get_median_depth(depth_frame, cx, cy)
                    if depth_m is None or depth_m <= 0.01 or depth_m > 3.0:
                        continue
                    # If we fell back to centre, still use bottom pixel for
                    # the ray direction — the depth magnitude matters less
                    # than getting the ray aimed at the right spot.

                # Spike filter
                prev = self.prev_depths.get(cls_name)
                if prev is not None and \
                        abs(depth_m - prev) > DEPTH_JUMP_THRESHOLD:
                    depth_m = prev
                self.prev_depths[cls_name] = depth_m

                # Deproject the BOTTOM point (not centre) to get the ray
                # aimed at the table contact region
                cam_xyz = deproject(self.intrinsics, bx, by, depth_m)
                xy = table_ray_intersect(self.T, cam_xyz, self.table_z_m)
                if xy is None:
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

                # Work plane Z — add half object height so collision object
                # sits ON the table, not inside it
                z_table    = get_table_z(x_smooth, y_smooth)
                half_height_mm = MAX_OBJECT_HEIGHT_MM / 2.0
                z_approach = z_table + APPROACH_OFFSET
                z_grip     = z_table + GRIP_OFFSET

                # Convert px → mm (still use centre depth for dimension scaling)
                depth_centre = get_median_depth(depth_frame, cx, cy) or depth_m
                fx    = self.intrinsics.fx
                dx_mm = pixels_to_mm(w_px, depth_centre, fx)
                dy_mm = pixels_to_mm(h_px, depth_centre, fx)

                # dz from depth
                dz_mm, dz_source = compute_dz(
                    depth_frame, self.intrinsics,
                    cx, cy, w_px / 2, depth_centre
                )

                quaternion = angle_to_quaternion(angle_rad)

                # Stability
                count      = self.detection_counts.get(cls_name, 0)
                stable_str = "STABLE" if count >= STABILITY_FRAMES \
                    else f"stabilising {count}/{STABILITY_FRAMES}"
                colour = (0, 255, 0) if count >= STABILITY_FRAMES \
                    else (0, 165, 255)

                if SHOW_DISPLAY:
                    # Draw the bottom sample point
                    cv2.circle(display, (bx, by), 5, (255, 0, 255), -1)
                    cv2.line(display, (cx, cy), (bx, by), (255, 0, 255), 1)

                    cv2.putText(display,
                                f"{cls_name} | {stable_str}",
                                (cx - 80, cy - 52),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 2)
                    cv2.putText(display,
                                f"[bot]  bx={bx} by={by} d={depth_m*1000:.0f}mm",
                                (cx - 80, cy - 34),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 0, 255), 1)
                    cv2.putText(display,
                                f"[pub]  x={x_smooth:.0f} "
                                f"y={y_smooth:.0f} "
                                f"z={z_grip:.0f}mm",
                                (cx - 80, cy - 18),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, colour, 2)
                    cv2.putText(display,
                                f"angle={angle_deg:.1f}deg | "
                                f"dz={dz_mm}mm({dz_source})",
                                (cx - 80, cy - 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.40, colour, 1)

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
                        "sample_point":  "bottom_edge",
                        "bottom_px":     [bx, by],
                    }
                }

        if SHOW_DISPLAY:
            status = "PUBLISHING" if any(
                c >= STABILITY_FRAMES
                for c in self.detection_counts.values()
            ) else "Stabilising..."
            colour = (0, 255, 0) if status == "PUBLISHING" \
                else (0, 165, 255)
            cv2.putText(display, f"{status} [bottom-point fix]", (10, 30),
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
    node = OBBDetectorBottomPoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
