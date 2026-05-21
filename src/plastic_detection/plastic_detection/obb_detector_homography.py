#!/usr/bin/env python3
"""
OBB Plastic Detection — FIX 4: Homography-Based XY Localisation
==================================================================
Drop-in replacement for obb_detector_node.py.

PROBLEM FIXED
-------------
Using depth for XY localisation on a tabletop with an angled camera is
inherently error-prone:
  - Depth on curved/reflective plastic is noisy
  - The detected centre pixel often hits the object side, not the base
  - Perspective distortion means pixel-scale changes across the image
  - Small depth errors cause large XY shifts at oblique angles

FIX
---
Since all objects rest on a known table plane, we can map image pixels
directly to table XY using a **homography** — a 3×3 matrix that maps
2D image points to 2D table-plane points.

The homography is computed from the 4 ArUco calibration corners:
  pixel (u, v)  ↔  robot (x, y) on the table

This completely removes depth from the XY pipeline.  Depth is used ONLY
to estimate the object's height above the table for the Z dimension.

For the sample point, the **bottom edge midpoint** of the OBB is used
(same as Fix 1) to approximate the table contact point — this further
improves accuracy since the homography maps table-plane pixels most
accurately.

HOW IT WORKS
------------
1. At startup, load the 4 calibration point pairs (pixel ↔ robot XY).
2. Compute the 3×3 homography H via cv2.findHomography().
3. For each detection, compute the bottom-edge midpoint pixel.
4. Map it through H to get robot-frame (x, y) — no depth needed.
5. Use depth only for estimating object height (dz).

Run:  ros2 run plastic_detection obb_detector_homography
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

MODEL_PATH       = "/home/billy/git/Robotics-Studio-2/src/plastic_detection/plastic_detection/best.pt"
CALIBRATION_FILE = "/home/billy/git/Robotics-Studio-2/src/plastic_detection/plastic_detection/camera_to_robot_calibration.json"

DEPTH_KERNEL_SIZE    = 11
SMOOTHING_WINDOW     = 5
DEPTH_JUMP_THRESHOLD = 0.05
CONF_THRESHOLD       = 0.7
STABILITY_FRAMES     = 5
GRIP_OFFSET          = 50
APPROACH_OFFSET      = 150

MAX_OBJECT_HEIGHT_MM = 60.0

SHOW_DISPLAY = True

# ──────────────────────────────────────────────────────────────────────────────
# TABLE PLANE  (fitted from calibration, for Z only)
# ──────────────────────────────────────────────────────────────────────────────

class TablePlane:
    def __init__(self, calibration_points):
        pts = [(p["robot"]["x"], p["robot"]["y"], p["robot"]["z"])
               for p in calibration_points]
        if len(pts) < 3:
            z_avg = np.mean([p[2] for p in pts]) if pts else 0.035
            self.a, self.b, self.c = 0.0, 0.0, z_avg
        else:
            A = np.array([[x, y, 1.0] for x, y, z in pts])
            z = np.array([z for x, y, z in pts])
            coeffs, _, _, _ = np.linalg.lstsq(A, z, rcond=None)
            self.a, self.b, self.c = float(coeffs[0]), float(coeffs[1]), float(coeffs[2])

    def z_at_mm(self, x_mm, y_mm):
        return (self.a * (x_mm / 1000.0) + self.b * (y_mm / 1000.0) + self.c) * 1000.0


# ──────────────────────────────────────────────────────────────────────────────
# HOMOGRAPHY
# ──────────────────────────────────────────────────────────────────────────────

def compute_homography(calibration_points):
    """
    Compute 3×3 homography from pixel (u,v) → robot (x_m, y_m) on table.

    Uses the calibration_points list from the JSON, each containing:
      camera: {center_x, center_y, ...}
      robot:  {x, y, z}
    """
    src_pts = []  # image pixels
    dst_pts = []  # robot XY in metres

    for p in calibration_points:
        cam = p["camera"]
        rob = p["robot"]
        src_pts.append([cam["center_x"], cam["center_y"]])
        dst_pts.append([rob["x"], rob["y"]])

    src = np.array(src_pts, dtype=np.float64)
    dst = np.array(dst_pts, dtype=np.float64)

    H, status = cv2.findHomography(src, dst, method=0)  # exact with 4 pts
    return H


def homography_map(H, u, v):
    """Map pixel (u, v) → robot (x_m, y_m) using homography."""
    pt = np.array([u, v, 1.0], dtype=np.float64)
    mapped = H @ pt
    mapped /= mapped[2]  # homogeneous → Cartesian
    return float(mapped[0]), float(mapped[1])


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


def obb_bottom_midpoint(cx, cy, w_px, h_px, angle_rad, img_w, img_h):
    """Bottom-edge midpoint of the OBB (largest Y = closest to table)."""
    sin_a = np.sin(angle_rad)
    cos_a = np.cos(angle_rad)

    # Perpendicular to the width axis
    dx_h = -sin_a
    dy_h = cos_a

    mid1_x = cx + (h_px / 2) * dx_h
    mid1_y = cy + (h_px / 2) * dy_h
    mid2_x = cx - (h_px / 2) * dx_h
    mid2_y = cy - (h_px / 2) * dy_h

    if mid1_y >= mid2_y:
        bx, by = mid1_x, mid1_y
    else:
        bx, by = mid2_x, mid2_y

    bx = int(np.clip(bx, 0, img_w - 1))
    by = int(np.clip(by, 0, img_h - 1))
    return bx, by


# ──────────────────────────────────────────────────────────────────────────────
# HOMOGRAPHY-BASED DIMENSION SCALING
# ──────────────────────────────────────────────────────────────────────────────

def obb_dimensions_via_homography(H, cx, cy, w_px, h_px, angle_rad):
    """
    Compute object dimensions in mm using the homography instead of
    depth-based pixel scaling.

    Maps the OBB corner pixels through H and measures the resulting
    robot-frame distances.  This is more accurate than pixels_to_mm()
    because it accounts for perspective distortion across the image.

    Returns (dx_mm, dy_mm).
    """
    sin_a = np.sin(angle_rad)
    cos_a = np.cos(angle_rad)

    # Four half-extents along box axes
    hw = w_px / 2
    hh = h_px / 2

    # Midpoints of the 4 edges in pixel space
    # Width-axis midpoints (along angle direction)
    w1 = (cx + hw * cos_a, cy + hw * sin_a)
    w2 = (cx - hw * cos_a, cy - hw * sin_a)
    # Height-axis midpoints (perpendicular)
    h1 = (cx - hh * sin_a, cy + hh * cos_a)
    h2 = (cx + hh * sin_a, cy - hh * cos_a)

    # Map through homography to robot frame
    w1_r = np.array(homography_map(H, w1[0], w1[1]))
    w2_r = np.array(homography_map(H, w2[0], w2[1]))
    h1_r = np.array(homography_map(H, h1[0], h1[1]))
    h2_r = np.array(homography_map(H, h2[0], h2[1]))

    dx_m = np.linalg.norm(w1_r - w2_r)
    dy_m = np.linalg.norm(h1_r - h2_r)

    return dx_m * 1000.0, dy_m * 1000.0


# ──────────────────────────────────────────────────────────────────────────────
# NODE
# ──────────────────────────────────────────────────────────────────────────────

class OBBDetectorHomography(Node):
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

        # Calibration
        cal_pts = data.get("calibration_points", [])

        # FIX 4: Compute pixel→robot homography from calibration corners
        if len(cal_pts) >= 4:
            self.H = compute_homography(cal_pts)
            self.get_logger().info('[FIX 4] Homography computed from calibration corners')

            # Verify: map calibration pixels back and check error
            for p in cal_pts:
                cam = p["camera"]
                rob = p["robot"]
                pred_x, pred_y = homography_map(
                    self.H, cam["center_x"], cam["center_y"])
                err = np.sqrt((pred_x - rob["x"])**2 + (pred_y - rob["y"])**2)
                self.get_logger().info(
                    f'  Marker {p["marker_id"]}: '
                    f'pred=({pred_x*1000:.1f}, {pred_y*1000:.1f})mm '
                    f'actual=({rob["x"]*1000:.1f}, {rob["y"]*1000:.1f})mm '
                    f'err={err*1000:.1f}mm')
        else:
            self.get_logger().warn(
                'Not enough calibration points for homography — '
                'falling back to ray intersection')
            self.H = None

        # Table plane for Z only
        self.table_plane = TablePlane(cal_pts)
        self.table_z_m = self.table_plane.c

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
            f'OBB Detector (homography) ready — publishing to /plastic_detections')

    def display_loop(self):
        while self.running:
            with self.frame_lock:
                frame = self.display_frame.copy() \
                    if self.display_frame is not None else None
            if frame is not None:
                cv2.imshow("OBB Plastic Sorter [homography]", frame)
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

                # Bottom-edge midpoint for the sample pixel
                bx, by = obb_bottom_midpoint(
                    xywhr[0], xywhr[1], w_px, h_px, angle_rad, img_w, img_h)

                # Depth — used ONLY for height estimation, not XY
                depth_m = get_median_depth(depth_frame, bx, by)
                if depth_m is None:
                    depth_m = get_median_depth(depth_frame, cx, cy)
                if depth_m is None or depth_m <= 0.01 or depth_m > 3.0:
                    # For homography we don't strictly need depth for XY,
                    # but we still want it for height.  Use a default.
                    depth_m = 0.6  # approximate table distance

                # Spike filter
                prev = self.prev_depths.get(cls_name)
                if prev is not None and \
                        abs(depth_m - prev) > DEPTH_JUMP_THRESHOLD:
                    depth_m = prev
                self.prev_depths[cls_name] = depth_m

                # ── FIX 4: XY via homography (no depth!) ──
                if self.H is not None:
                    # Map the bottom-edge pixel directly to robot XY
                    x_m, y_m = homography_map(self.H, bx, by)
                    raw_robot = {"x": x_m, "y": y_m, "z": self.table_z_m}
                else:
                    # Fallback to original ray intersection
                    cam_xyz = rs.rs2_deproject_pixel_to_point(
                        self.intrinsics, [bx, by], depth_m)
                    from obb_detector_node import table_ray_intersect
                    xy = table_ray_intersect(self.T, cam_xyz, self.table_z_m)
                    if xy is None:
                        p = np.array([cam_xyz[0], cam_xyz[1], cam_xyz[2], 1.0])
                        r = self.T @ p
                        raw_robot = {"x": float(r[0]), "y": float(r[1]),
                                     "z": float(r[2])}
                    else:
                        raw_robot = {"x": xy[0], "y": xy[1],
                                     "z": self.table_z_m}

                # Temporal smoothing
                if cls_name not in self.pose_histories:
                    self.pose_histories[cls_name] = deque(maxlen=SMOOTHING_WINDOW)
                self.pose_histories[cls_name].append(raw_robot)
                hist = self.pose_histories[cls_name]

                x_smooth = float(np.mean([p["x"] for p in hist])) * 1000
                y_smooth = float(np.mean([p["y"] for p in hist])) * 1000

                # Z from fitted table plane
                z_table    = self.table_plane.z_at_mm(x_smooth, y_smooth)
                z_approach = z_table + APPROACH_OFFSET
                z_grip     = z_table + GRIP_OFFSET

                # Dimensions via homography (perspective-correct)
                if self.H is not None:
                    dx_mm, dy_mm = obb_dimensions_via_homography(
                        self.H, cx, cy, w_px, h_px, angle_rad)
                else:
                    fx = self.intrinsics.fx
                    dx_mm = pixels_to_mm(w_px, depth_m, fx)
                    dy_mm = pixels_to_mm(h_px, depth_m, fx)

                dz_mm, dz_source = compute_dz(
                    depth_frame, self.intrinsics,
                    cx, cy, w_px / 2, depth_m
                )

                quaternion = angle_to_quaternion(angle_rad)

                count      = self.detection_counts.get(cls_name, 0)
                stable_str = "STABLE" if count >= STABILITY_FRAMES \
                    else f"stabilising {count}/{STABILITY_FRAMES}"
                colour = (0, 255, 0) if count >= STABILITY_FRAMES \
                    else (0, 165, 255)

                if SHOW_DISPLAY:
                    # Bottom point marker
                    cv2.circle(display, (bx, by), 5, (255, 0, 255), -1)
                    cv2.line(display, (cx, cy), (bx, by), (255, 0, 255), 1)

                    cv2.putText(display,
                                f"{cls_name} | {stable_str}",
                                (cx - 80, cy - 52),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 2)
                    method = "homography" if self.H is not None else "ray"
                    cv2.putText(display,
                                f"[{method}] bx={bx} by={by}",
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
                                f"dx={dx_mm:.0f} dy={dy_mm:.0f}mm",
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
                        "xy_method":     "homography" if self.H is not None
                                         else "ray_intersection",
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
            cv2.putText(display, f"{status} [homography]", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)
            cv2.putText(display, "Q = quit", (10, 460),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)

            with self.frame_lock:
                self.display_frame = display

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
    node = OBBDetectorHomography()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
