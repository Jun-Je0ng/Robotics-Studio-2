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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image as RosImage

_IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
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
GRIP_OFFSET          = 50   # default: mm above table surface to grip
APPROACH_OFFSET      = 150

# Per-class grip heights (mm above table). Shorter objects need a smaller offset
# so the gripper descends far enough to contact them.
CLASS_GRIP_OFFSETS = {
    'hdpe_bottle':  10,   # short/squat bottle — grip close to table
    'pp_container': 20,   # medium-height container
    'pet_bottle':   50,   # tall upright bottle
}
MATCH_RADIUS_PX      = 60   # pixel radius to match a detection to an existing track

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
        self.camera_pub = self.create_publisher(RosImage, '/camera/annotated', _IMAGE_QOS)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Shared frame for display thread
        self.display_frame = None
        self.frame_lock    = threading.Lock()
        self.running       = True

        # Load model
        self.get_logger().info(f'Loading OBB model from: {MODEL_PATH}')
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info(f'Model loaded — classes: {self.model.names}')

        # Load  calibration
        if not os.path.exists(CALIBRATION_FILE):
            self.get_logger().error(
                f'Calibration file not found: {CALIBRATION_FILE}')
            raise FileNotFoundError(CALIBRATION_FILE)
        with open(CALIBRATION_FILE) as f:
            data = json.load(f)
        self.T = np.array(data["transform_matrix"])
        self.cal_pts = data.get("calibration_points", [])

        if "homography_pixel_to_robot_mm" in data:
            self.H_pix = np.array(data["homography_pixel_to_robot_mm"],
                                  dtype=np.float32)
            self.get_logger().info('Homography pixel→robot loaded (primary XY method)')
        else:
            self.H_pix = None
            self.get_logger().warn(
                'No homography in calibration file — re-run camera_calibration.py. '
                'Falling back to 3D ray intersection (may have scale error).'
            )

        # ArUco detector for live marker overlay
        try:
            aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            self.get_logger().info('ArUco detector initialized (DICT_4X4_50)')
        except Exception:
            self.aruco_detector = None
            self.get_logger().warn('cv2.aruco unavailable — live ArUco overlay disabled')

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
        self.track_positions   = {}  # track_key -> (cx_px, cy_px), last seen pixel position

        # Start display thread
        if SHOW_DISPLAY:
            self.display_thread = threading.Thread(
                target=self.display_loop, daemon=True)
            self.display_thread.start()

        self.get_logger().info(
            f'OBB Detector Node ready — publishing to /plastic_detections '
            f'(conf>={CONF_THRESHOLD}, stable>={STABILITY_FRAMES} frames)'
        )

    def _get_track_key(self, cls_name: str, cx: int, cy: int, already_assigned: dict) -> str:
        """
        Match this detection to the nearest existing track of the same class within
        MATCH_RADIUS_PX pixels. Returns a key like 'pet_bottle_0' or 'pet_bottle_1'.
        already_assigned maps track_keys already used in the current frame so two
        detections of the same class cannot steal each other's identity.
        """
        prefix    = cls_name + '_'
        best_key  = None
        best_dist = float('inf')
        for key, (kx, ky) in self.track_positions.items():
            if not key.startswith(prefix):
                continue
            if key in already_assigned:
                continue
            dist = np.hypot(cx - kx, cy - ky)
            if dist < MATCH_RADIUS_PX and dist < best_dist:
                best_dist = dist
                best_key  = key
        if best_key is None:
            idx = 0
            while (f"{prefix}{idx}" in self.track_positions or
                   f"{prefix}{idx}" in already_assigned):
                idx += 1
            best_key = f"{prefix}{idx}"
        return best_key

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
        new_track_positions = {}  # track_key -> (cx, cy) assigned this frame

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

                # Assign a stable per-instance track key (e.g. 'pet_bottle_0', 'pet_bottle_1')
                track_key = self._get_track_key(cls_name, cx, cy, new_track_positions)
                new_track_positions[track_key] = (cx, cy)

                depth_m = get_median_depth(depth_frame, cx, cy)
                if depth_m is None or depth_m <= 0.01 or depth_m > 3.0:
                    continue

                # Spike filter
                prev = self.prev_depths.get(track_key)
                if prev is not None and \
                        abs(depth_m - prev) > DEPTH_JUMP_THRESHOLD:
                    depth_m = prev
                self.prev_depths[track_key] = depth_m

                # Camera → robot XY via homography (pixel → robot mm)
                # Homography is numerically stable for the 4-coplanar-point
                # calibration geometry; the 3D SVD fit is degenerate when all
                # calibration points share the same Z plane.
                if self.H_pix is not None:
                    pt = np.array([[[float(cx), float(cy)]]], dtype=np.float32)
                    rxy = cv2.perspectiveTransform(pt, self.H_pix)[0][0]
                    raw_robot = {
                        "x": float(rxy[0]) / 1000.0,
                        "y": float(rxy[1]) / 1000.0,
                        "z": self.table_z_m,
                    }
                    cam_xyz = deproject(self.intrinsics, cx, cy, depth_m)
                else:
                    cam_xyz = deproject(self.intrinsics, cx, cy, depth_m)
                    xy = table_ray_intersect(self.T, cam_xyz, self.table_z_m)
                    if xy is None:
                        raw_robot = camera_to_robot(self.T, cam_xyz)
                    else:
                        raw_robot = {"x": xy[0], "y": xy[1], "z": self.table_z_m}

                # Temporal smoothing
                if track_key not in self.pose_histories:
                    self.pose_histories[track_key] = deque(maxlen=SMOOTHING_WINDOW)
                self.pose_histories[track_key].append(raw_robot)
                hist = self.pose_histories[track_key]

                x_smooth = float(np.mean([p["x"] for p in hist])) * 1000
                y_smooth = float(np.mean([p["y"] for p in hist])) * 1000

                # Work plane Z
                z_table    = get_table_z(x_smooth, y_smooth)
                z_approach = z_table + APPROACH_OFFSET
                grip_offset = CLASS_GRIP_OFFSETS.get(cls_name, GRIP_OFFSET)
                z_grip     = z_table + grip_offset

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
                count      = self.detection_counts.get(track_key, 0)
                stable_str = "STABLE" if count >= STABILITY_FRAMES \
                    else f"stabilising {count}/{STABILITY_FRAMES}"
                colour = (0, 255, 0) if count >= STABILITY_FRAMES \
                    else (0, 165, 255)

                # Overlay info on display frame
                if SHOW_DISPLAY:
                    # Instance track key + stability (e.g. "pet_bottle_1 | STABLE")
                    cv2.putText(display,
                                f"{track_key} | {stable_str}",
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
                    # Centroid crosshair — pixel location of published pose
                    cv2.drawMarker(display, (cx, cy), colour,
                                   cv2.MARKER_CROSS, 28, 2)
                    cv2.circle(display, (cx, cy), 12, colour, 2)

                # Build dimensions — only include dz if measurement succeeded
                dimensions = {
                    "dx_mm": round(dx_mm, 1),
                    "dy_mm": round(dy_mm, 1),
                }
                if dz_mm is not None:
                    dimensions["dz_mm"] = dz_mm

                seen_this_frame.add(track_key)
                current_detections[track_key] = {
                    "instance_id": track_key,
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

        # Commit this frame's track positions
        for k, pos in new_track_positions.items():
            self.track_positions[k] = pos

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

            # ── Calibration corner overlay (known pixel positions from JSON) ──
            for pt in self.cal_pts:
                mid = pt["marker_id"]
                px  = pt["camera"]["center_x"]
                py  = pt["camera"]["center_y"]
                rx  = pt["robot"]["x"] * 1000
                ry  = pt["robot"]["y"] * 1000
                cv2.rectangle(display, (px - 10, py - 10),
                              (px + 10, py + 10), (0, 255, 255), 2)
                cv2.putText(display,
                            f"M{mid} r({rx:.0f},{ry:.0f})",
                            (px + 12, py + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1)

            # ── Live ArUco detection (depth → robot XY via ray intersection) ──
            if self.aruco_detector is not None:
                try:
                    corners, ids, _ = self.aruco_detector.detectMarkers(
                        color_img)
                    if ids is not None:
                        cv2.aruco.drawDetectedMarkers(display, corners, ids)
                        for i, corner in enumerate(corners):
                            mid = int(ids[i][0])
                            c   = corner[0]
                            mcx = int(c[:, 0].mean())
                            mcy = int(c[:, 1].mean())
                            if self.H_pix is not None:
                                mpt  = np.array([[[float(mcx), float(mcy)]]],
                                                dtype=np.float32)
                                mrxy = cv2.perspectiveTransform(
                                    mpt, self.H_pix)[0][0]
                                mx_mm, my_mm = float(mrxy[0]), float(mrxy[1])
                            else:
                                md = get_median_depth(depth_frame, mcx, mcy, k=7)
                                if md is None or not (0.1 < md < 2.0):
                                    continue
                                mcam = deproject(self.intrinsics, mcx, mcy, md)
                                mxy  = table_ray_intersect(
                                    self.T, mcam, self.table_z_m)
                                if mxy is None:
                                    mr = camera_to_robot(self.T, mcam)
                                    mx_mm = mr["x"] * 1000
                                    my_mm = mr["y"] * 1000
                                else:
                                    mx_mm = mxy[0] * 1000
                                    my_mm = mxy[1] * 1000
                            cv2.putText(
                                display,
                                f"[live]M{mid}({mx_mm:.0f},{my_mm:.0f})",
                                (mcx + 12, mcy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                                (0, 200, 255), 1)
                except Exception:
                    pass

            with self.frame_lock:
                self.display_frame = display

        # ── Publish annotated frame to /camera/annotated for the GUI ──────────
        try:
            img_msg = RosImage()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_color_optical_frame'
            img_msg.height = display.shape[0]
            img_msg.width  = display.shape[1]
            img_msg.encoding = 'bgr8'
            img_msg.is_bigendian = False
            img_msg.step = display.shape[1] * 3
            img_msg.data = display.tobytes()
            self.camera_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish camera frame: {e}')

        # Update consecutive frame counts per track key
        all_keys = set(self.detection_counts.keys()) | seen_this_frame
        for key in all_keys:
            if key in seen_this_frame:
                self.detection_counts[key] = self.detection_counts.get(key, 0) + 1
            else:
                self.detection_counts[key] = 0
                self.stable_detections.pop(key, None)
                self.track_positions.pop(key, None)
                self.pose_histories.pop(key, None)
                self.prev_depths.pop(key, None)

        # Only publish stable detections
        stable = []
        for key, count in self.detection_counts.items():
            if count >= STABILITY_FRAMES and key in current_detections:
                self.stable_detections[key] = current_detections[key]
                stable.append(self.stable_detections[key])

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

# #!/usr/bin/env python3
# """
# OBB Plastic Detection ROS2 Node
# =================================
# Publishes detected plastic objects with:
# - Pose (position + quaternion orientation)
# - Dimensions in mm (dx, dy, dz)
# - Classification (class + confidence)

# Stabilisation:
# - Confidence threshold: 0.7
# - Consecutive frames: 5

# Visual:
# - Live camera feed with OBB boxes and detection info
# - Comment out SHOW_DISPLAY to disable camera feed window
# """

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import json
# import numpy as np
# import pyrealsense2 as rs
# from collections import deque
# from ultralytics import YOLO
# from scipy.spatial.transform import Rotation
# import os
# import time
# import cv2
# import threading

# # ──────────────────────────────────────────────────────────────────────────────
# # CONFIG
# # ──────────────────────────────────────────────────────────────────────────────

# MODEL_PATH       = "/home/jun/git/Robotics-Studio-2/src/plastic_detection/plastic_detection/best.pt"
# CALIBRATION_FILE = "/home/jun/git/Robotics-Studio-2/src/plastic_detection/plastic_detection/camera_to_robot_calibration.json"

# DEPTH_KERNEL_SIZE    = 11
# SMOOTHING_WINDOW     = 5
# DEPTH_JUMP_THRESHOLD = 0.05
# CONF_THRESHOLD       = 0.7
# STABILITY_FRAMES     = 5
# GRIP_OFFSET          = 50
# APPROACH_OFFSET      = 150

# SHOW_DISPLAY = True  # ← comment this out to disable the camera feed window

# # ──────────────────────────────────────────────────────────────────────────────
# # WORK PLANE
# # ──────────────────────────────────────────────────────────────────────────────

# def get_table_z(x_mm, y_mm):
#     return 0.001841 * x_mm + -0.007761 * y_mm + -185.5378

# # ──────────────────────────────────────────────────────────────────────────────
# # HELPERS
# # ──────────────────────────────────────────────────────────────────────────────

# def get_median_depth(depth_frame, cx, cy, k=DEPTH_KERNEL_SIZE):
#     h, w = depth_frame.get_height(), depth_frame.get_width()
#     half = k // 2
#     x0, x1 = max(0, cx - half), min(w, cx + half + 1)
#     y0, y1 = max(0, cy - half), min(h, cy + half + 1)
#     arr = np.asanyarray(depth_frame.get_data())
#     region = arr[y0:y1, x0:x1]
#     valid = region[region > 0]
#     if valid.size == 0:
#         return None
#     return float(np.median(valid)) / 1000.0

# def deproject(intrinsics, u, v, depth_m):
#     return rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth_m)

# def camera_to_robot(T, cam_xyz):
#     p = np.array([cam_xyz[0], cam_xyz[1], cam_xyz[2], 1.0])
#     r = T @ p
#     return {"x": float(r[0]), "y": float(r[1]), "z": float(r[2])}

# def angle_to_quaternion(angle_rad):
#     """Convert 2D OBB rotation angle to 3D quaternion (rotation around Z axis)."""
#     r = Rotation.from_euler('z', angle_rad)
#     q = r.as_quat()  # [x, y, z, w]
#     return {
#         "qx": round(float(q[0]), 6),
#         "qy": round(float(q[1]), 6),
#         "qz": round(float(q[2]), 6),
#         "qw": round(float(q[3]), 6),
#     }

# def pixels_to_mm(pixels, depth_m, focal_length):
#     """Convert pixel dimension to mm using depth and focal length."""
#     return (pixels / focal_length) * depth_m * 1000.0

# def compute_dz(depth_frame, intrinsics, cx, cy, half_width_px, depth_m):
#     """
#     Compute bottle dz (diameter) from depth data.
#     Returns (dz_mm, 'depth') if successful, or (None, 'failed') if unreliable.
#     """
#     try:
#         edge_x = int(cx + half_width_px)
#         edge_x = max(0, min(depth_frame.get_width() - 1, edge_x))

#         depth_centre = get_median_depth(depth_frame, cx, cy, k=5)
#         depth_edge   = get_median_depth(depth_frame, edge_x, cy, k=5)

#         if depth_centre is None or depth_edge is None:
#             return None, "failed"

#         pt_centre = rs.rs2_deproject_pixel_to_point(
#             intrinsics, [cx, cy], depth_centre)
#         pt_edge   = rs.rs2_deproject_pixel_to_point(
#             intrinsics, [edge_x, cy], depth_edge)

#         dist = np.linalg.norm(np.array(pt_centre) - np.array(pt_edge))
#         dz   = dist * 2 * 1000  # metres → mm

#         if 20.0 <= dz <= 200.0:
#             return round(dz, 1), "depth"
#         else:
#             return None, "failed"

#     except Exception:
#         return None, "failed"

# # ──────────────────────────────────────────────────────────────────────────────
# # NODE
# # ──────────────────────────────────────────────────────────────────────────────

# class OBBDetectorNode(Node):
#     def __init__(self):
#         super().__init__('obb_detector_node')

#         self.publisher_ = self.create_publisher(String, 'plastic_detections', 10)
#         self.timer = self.create_timer(0.1, self.timer_callback)

#         # Shared frame for display thread
#         self.display_frame = None
#         self.frame_lock    = threading.Lock()
#         self.running       = True

#         # Load model
#         self.get_logger().info(f'Loading OBB model from: {MODEL_PATH}')
#         self.model = YOLO(MODEL_PATH)
#         self.get_logger().info(f'Model loaded — classes: {self.model.names}')

#         # Load calibration
#         if not os.path.exists(CALIBRATION_FILE):
#             self.get_logger().error(
#                 f'Calibration file not found: {CALIBRATION_FILE}')
#             raise FileNotFoundError(CALIBRATION_FILE)
#         with open(CALIBRATION_FILE) as f:
#             data = json.load(f)
#         self.T = np.array(data["transform_matrix"])
#         self.get_logger().info('Calibration loaded.')

#         # RealSense setup
#         self.pipeline = rs.pipeline()
#         cfg = rs.config()
#         cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#         cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#         profile = self.pipeline.start(cfg)

#         self.align = rs.align(rs.stream.color)
#         depth_profile = profile.get_stream(rs.stream.depth)
#         self.intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()

#         self.pose_histories    = {}
#         self.prev_depths       = {}
#         self.detection_counts  = {}
#         self.stable_detections = {}

#         # Start display thread
#         if SHOW_DISPLAY:
#             self.display_thread = threading.Thread(
#                 target=self.display_loop, daemon=True)
#             self.display_thread.start()

#         self.get_logger().info(
#             f'OBB Detector Node ready — publishing to /plastic_detections '
#             f'(conf>={CONF_THRESHOLD}, stable>={STABILITY_FRAMES} frames)'
#         )

#     def display_loop(self):
#         """Runs in separate thread — handles cv2 window."""
#         while self.running:
#             with self.frame_lock:
#                 frame = self.display_frame.copy() \
#                     if self.display_frame is not None else None
#             if frame is not None:
#                 cv2.imshow("OBB Plastic Sorter", frame)
#                 if cv2.waitKey(1) & 0xFF == ord('q'):
#                     self.running = False
#                     break
#             else:
#                 time.sleep(0.01)
#         cv2.destroyAllWindows()

#     def timer_callback(self):
#         if not self.running:
#             return

#         frames  = self.pipeline.wait_for_frames()
#         aligned = self.align.process(frames)
#         depth_frame = aligned.get_depth_frame()
#         color_frame = aligned.get_color_frame()

#         if not depth_frame or not color_frame:
#             return

#         color_img = np.asanyarray(color_frame.get_data())
#         results   = self.model(color_img, verbose=False, imgsz=640, device='cpu')[0]

#         # Draw OBB boxes on display frame
#         display = results.plot()

#         seen_this_frame    = set()
#         current_detections = {}

#         if results.obb is not None:
#             for obb_box in results.obb:
#                 conf = float(obb_box.conf[0])
#                 if conf < CONF_THRESHOLD:
#                     continue

#                 cls_id   = int(obb_box.cls[0])
#                 cls_name = self.model.names[cls_id]

#                 xywhr     = obb_box.xywhr[0].tolist()
#                 cx, cy    = int(xywhr[0]), int(xywhr[1])
#                 w_px      = float(xywhr[2])
#                 h_px      = float(xywhr[3])
#                 angle_rad = float(xywhr[4])
#                 angle_deg = float(np.degrees(angle_rad))

#                 depth_m = get_median_depth(depth_frame, cx, cy)
#                 if depth_m is None or depth_m <= 0.01 or depth_m > 3.0:
#                     continue

#                 # Spike filter
#                 prev = self.prev_depths.get(cls_name)
#                 if prev is not None and \
#                         abs(depth_m - prev) > DEPTH_JUMP_THRESHOLD:
#                     depth_m = prev
#                 self.prev_depths[cls_name] = depth_m

#                 # Camera → robot
#                 cam_xyz   = deproject(self.intrinsics, cx, cy, depth_m)
#                 raw_robot = camera_to_robot(self.T, cam_xyz)

#                 # Temporal smoothing
#                 if cls_name not in self.pose_histories:
#                     self.pose_histories[cls_name] = deque(maxlen=SMOOTHING_WINDOW)
#                 self.pose_histories[cls_name].append(raw_robot)
#                 hist = self.pose_histories[cls_name]

#                 x_smooth = float(np.mean([p["x"] for p in hist])) * 1000
#                 y_smooth = float(np.mean([p["y"] for p in hist])) * 1000

#                 # Work plane Z
#                 z_table    = get_table_z(x_smooth, y_smooth)
#                 z_approach = z_table + APPROACH_OFFSET
#                 z_grip     = z_table + GRIP_OFFSET

#                 # Convert px → mm
#                 fx    = self.intrinsics.fx
#                 dx_mm = pixels_to_mm(w_px, depth_m, fx)
#                 dy_mm = pixels_to_mm(h_px, depth_m, fx)

#                 # Compute dz from depth
#                 dz_mm, dz_source = compute_dz(
#                     depth_frame, self.intrinsics,
#                     cx, cy, w_px / 2, depth_m
#                 )

#                 # Quaternion
#                 quaternion = angle_to_quaternion(angle_rad)

#                 # Stability indicator
#                 count      = self.detection_counts.get(cls_name, 0)
#                 stable_str = "STABLE" if count >= STABILITY_FRAMES \
#                     else f"stabilising {count}/{STABILITY_FRAMES}"
#                 colour = (0, 255, 0) if count >= STABILITY_FRAMES \
#                     else (0, 165, 255)

#                 # Overlay info on display frame
#                 if SHOW_DISPLAY:
#                     cv2.putText(display,
#                                 f"{cls_name} | x={x_smooth:.0f} "
#                                 f"y={y_smooth:.0f} z={z_grip:.0f}mm",
#                                 (cx - 80, cy - 30),
#                                 cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 2)
#                     cv2.putText(display,
#                                 f"angle={angle_deg:.1f}deg | "
#                                 f"dz={dz_mm}mm({dz_source}) | {stable_str}",
#                                 (cx - 80, cy - 12),
#                                 cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 2)

#                 # Build dimensions — only include dz if measurement succeeded
#                 dimensions = {
#                     "dx_mm": round(dx_mm, 1),
#                     "dy_mm": round(dy_mm, 1),
#                 }
#                 if dz_mm is not None:
#                     dimensions["dz_mm"] = dz_mm

#                 seen_this_frame.add(cls_name)
#                 current_detections[cls_name] = {
#                     "pose": {
#                         "position": {
#                             "x": round(x_smooth, 1),
#                             "y": round(y_smooth, 1),
#                             "z": round(z_grip, 1),
#                         },
#                         "orientation": quaternion,
#                     },
#                     "dimensions": dimensions,
#                     "classification": {
#                         "class":      cls_name,
#                         "confidence": round(conf, 3),
#                     },
#                     "debug": {
#                         "z_table_mm":    round(z_table, 1),
#                         "z_approach_mm": round(z_approach, 1),
#                         "angle_deg":     round(angle_deg, 2),
#                         "angle_rad":     round(angle_rad, 4),
#                         "depth_m":       round(depth_m, 4),
#                         "dz_source":     dz_source,
#                     }
#                 }

#         # HUD
#         if SHOW_DISPLAY:
#             status = "PUBLISHING" if any(
#                 c >= STABILITY_FRAMES
#                 for c in self.detection_counts.values()
#             ) else "Stabilising..."
#             colour = (0, 255, 0) if status == "PUBLISHING" \
#                 else (0, 165, 255)
#             cv2.putText(display, status, (10, 30),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)
#             cv2.putText(display, "Q = quit", (10, 460),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)

#             with self.frame_lock:
#                 self.display_frame = display

#         # Update consecutive frame counts
#         all_classes = set(self.detection_counts.keys()) | seen_this_frame
#         for cls_name in all_classes:
#             if cls_name in seen_this_frame:
#                 self.detection_counts[cls_name] = \
#                     self.detection_counts.get(cls_name, 0) + 1
#             else:
#                 self.detection_counts[cls_name] = 0
#                 self.stable_detections.pop(cls_name, None)

#         # Only publish stable detections
#         stable = []
#         for cls_name, count in self.detection_counts.items():
#             if count >= STABILITY_FRAMES and cls_name in current_detections:
#                 self.stable_detections[cls_name] = current_detections[cls_name]
#                 stable.append(self.stable_detections[cls_name])

#         if stable:
#             msg = String()
#             msg.data = json.dumps(stable)
#             self.publisher_.publish(msg)
#             self.get_logger().info(
#                 f'Published {len(stable)} stable detection(s): '
#                 f'{[d["classification"]["class"] for d in stable]}'
#             )

#     def destroy_node(self):
#         self.running = False
#         self.pipeline.stop()
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = OBBDetectorNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()