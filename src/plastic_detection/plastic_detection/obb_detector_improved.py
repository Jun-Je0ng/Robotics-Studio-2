#!/usr/bin/env python3
"""
OBB Plastic Detection — COMBINED: All Fixes Applied
=====================================================
Production-ready replacement for obb_detector_node.py.

This combines all four improvements into a single file:

  FIX 1 — Bottom-point localisation
    Uses the bottom-edge midpoint of the OBB instead of the centre.
    This approximates the table contact point, eliminating the
    perspective-induced XY offset on tall objects.

  FIX 2 — Centroid-based object tracking
    Each detection gets a persistent track ID via nearest-centroid
    association.  Pose history, depth filtering, and stability counts
    are per-track, not per-class.  Multiple objects of the same class
    no longer corrupt each other.

  FIX 3 — Consistent table plane
    A single plane  z = ax + by + c  is fitted from the calibration
    corners and used for BOTH the ray/homography intersection AND the
    z_grip/z_approach output.  No more two competing models.

  FIX 4 — Homography-based XY
    A pixel→robot homography is computed from the 4 ArUco calibration
    corners.  XY localisation uses the homography (no depth needed).
    Depth is used only for object height estimation.
    Dimensions are also computed via the homography for perspective-
    correct scaling across the tray.

The output message format is identical to the original (same JSON
schema on /plastic_detections), with extra debug fields.  The
downstream grip_pose_node and translator work without changes.

Run:  ros2 run plastic_detection obb_detector_improved
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

MAX_OBJECT_HEIGHT_MM = 60.0   # objects lie flat — max 6 cm tall

# Tracker
ASSOC_THRESHOLD_MM = 80.0
TRACK_TIMEOUT      = 10

SHOW_DISPLAY = True

# ──────────────────────────────────────────────────────────────────────────────
# FIX 3 — FITTED TABLE PLANE
# ──────────────────────────────────────────────────────────────────────────────

class TablePlane:
    """z = a*x + b*y + c   (all in metres)."""
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

    def z_at(self, x_m, y_m):
        return self.a * x_m + self.b * y_m + self.c

    def z_at_mm(self, x_mm, y_mm):
        return self.z_at(x_mm / 1000.0, y_mm / 1000.0) * 1000.0

    def __repr__(self):
        return f"TablePlane(z = {self.a:.6f}*x + {self.b:.6f}*y + {self.c:.6f})"


# ──────────────────────────────────────────────────────────────────────────────
# FIX 4 — HOMOGRAPHY
# ──────────────────────────────────────────────────────────────────────────────

def compute_homography(calibration_points):
    src = np.array([[p["camera"]["center_x"], p["camera"]["center_y"]]
                    for p in calibration_points], dtype=np.float64)
    dst = np.array([[p["robot"]["x"], p["robot"]["y"]]
                    for p in calibration_points], dtype=np.float64)
    H, _ = cv2.findHomography(src, dst, method=0)
    return H


def homography_map(H, u, v):
    pt = np.array([u, v, 1.0], dtype=np.float64)
    mapped = H @ pt
    mapped /= mapped[2]
    return float(mapped[0]), float(mapped[1])


def obb_dimensions_via_homography(H, cx, cy, w_px, h_px, angle_rad):
    sin_a, cos_a = np.sin(angle_rad), np.cos(angle_rad)
    hw, hh = w_px / 2, h_px / 2

    w1 = homography_map(H, cx + hw * cos_a, cy + hw * sin_a)
    w2 = homography_map(H, cx - hw * cos_a, cy - hw * sin_a)
    h1 = homography_map(H, cx - hh * sin_a, cy + hh * cos_a)
    h2 = homography_map(H, cx + hh * sin_a, cy - hh * cos_a)

    dx_m = np.sqrt((w1[0]-w2[0])**2 + (w1[1]-w2[1])**2)
    dy_m = np.sqrt((h1[0]-h2[0])**2 + (h1[1]-h2[1])**2)
    return dx_m * 1000.0, dy_m * 1000.0


# ──────────────────────────────────────────────────────────────────────────────
# FIX 1 — BOTTOM-POINT
# ──────────────────────────────────────────────────────────────────────────────

def obb_bottom_midpoint(cx, cy, w_px, h_px, angle_rad, img_w, img_h):
    """Midpoint of the OBB edge with the largest Y (closest to table)."""
    sin_a, cos_a = np.sin(angle_rad), np.cos(angle_rad)
    dx_h, dy_h = -sin_a, cos_a

    mid1_x = cx + (h_px / 2) * dx_h
    mid1_y = cy + (h_px / 2) * dy_h
    mid2_x = cx - (h_px / 2) * dx_h
    mid2_y = cy - (h_px / 2) * dy_h

    if mid1_y >= mid2_y:
        bx, by = mid1_x, mid1_y
    else:
        bx, by = mid2_x, mid2_y

    return int(np.clip(bx, 0, img_w - 1)), int(np.clip(by, 0, img_h - 1))


# ──────────────────────────────────────────────────────────────────────────────
# FIX 2 — CENTROID TRACKER
# ──────────────────────────────────────────────────────────────────────────────

class Track:
    _next_id = 0

    def __init__(self, cls_name, x_mm, y_mm):
        self.track_id = Track._next_id
        Track._next_id += 1
        self.cls_name = cls_name
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.pose_history = deque(maxlen=SMOOTHING_WINDOW)
        self.prev_depth = None
        self.detection_count = 0
        self.frames_missing = 0

    def update(self, raw_robot, depth_m):
        self.pose_history.append(raw_robot)
        self.x_mm = raw_robot["x"] * 1000
        self.y_mm = raw_robot["y"] * 1000
        self.prev_depth = depth_m
        self.detection_count += 1
        self.frames_missing = 0

    def mark_missing(self):
        self.frames_missing += 1
        self.detection_count = 0

    @property
    def is_expired(self):
        return self.frames_missing > TRACK_TIMEOUT

    @property
    def is_stable(self):
        return self.detection_count >= STABILITY_FRAMES

    def smoothed_position_mm(self):
        if not self.pose_history:
            return self.x_mm, self.y_mm
        x = float(np.mean([p["x"] for p in self.pose_history])) * 1000
        y = float(np.mean([p["y"] for p in self.pose_history])) * 1000
        return x, y


class CentroidTracker:
    def __init__(self, threshold=ASSOC_THRESHOLD_MM):
        self.tracks = []
        self.threshold = threshold

    def update(self, detections):
        for t in self.tracks:
            t.mark_missing()

        matched = []
        used = set()

        for det in detections:
            best, best_d = None, float('inf')
            for t in self.tracks:
                if id(t) in used or t.cls_name != det["cls_name"]:
                    continue
                d = np.sqrt((t.x_mm - det["x_mm"])**2 +
                            (t.y_mm - det["y_mm"])**2)
                if d < best_d:
                    best_d, best = d, t

            if best is not None and best_d < self.threshold:
                best.update(det["raw_robot"], det["depth_m"])
                used.add(id(best))
                matched.append((best, det))
            else:
                nt = Track(det["cls_name"], det["x_mm"], det["y_mm"])
                nt.update(det["raw_robot"], det["depth_m"])
                self.tracks.append(nt)
                matched.append((nt, det))

        self.tracks = [t for t in self.tracks if not t.is_expired]
        return matched


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
        pt_c = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth_centre)
        pt_e = rs.rs2_deproject_pixel_to_point(intrinsics, [edge_x, cy], depth_edge)
        dist = np.linalg.norm(np.array(pt_c) - np.array(pt_e))
        dz = dist * 2 * 1000
        if 20.0 <= dz <= 200.0:
            return round(dz, 1), "depth"
        return None, "failed"
    except Exception:
        return None, "failed"


# ──────────────────────────────────────────────────────────────────────────────
# NODE
# ──────────────────────────────────────────────────────────────────────────────

class OBBDetectorImproved(Node):
    def __init__(self):
        super().__init__('obb_detector_node')

        self.publisher_ = self.create_publisher(String, 'plastic_detections', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.display_frame = None
        self.frame_lock    = threading.Lock()
        self.running       = True

        # Model
        self.get_logger().info(f'Loading OBB model from: {MODEL_PATH}')
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info(f'Model loaded — classes: {self.model.names}')

        # Calibration
        if not os.path.exists(CALIBRATION_FILE):
            self.get_logger().error(f'Calibration file not found: {CALIBRATION_FILE}')
            raise FileNotFoundError(CALIBRATION_FILE)
        with open(CALIBRATION_FILE) as f:
            data = json.load(f)
        self.T = np.array(data["transform_matrix"])
        cal_pts = data.get("calibration_points", [])

        # FIX 3: Fitted table plane
        self.table_plane = TablePlane(cal_pts)
        self.get_logger().info(f'[FIX 3] {self.table_plane}')

        # FIX 4: Homography
        self.H = None
        if len(cal_pts) >= 4:
            self.H = compute_homography(cal_pts)
            self.get_logger().info('[FIX 4] Homography computed')
            for p in cal_pts:
                cam, rob = p["camera"], p["robot"]
                px, py = homography_map(self.H, cam["center_x"], cam["center_y"])
                err = np.sqrt((px - rob["x"])**2 + (py - rob["y"])**2) * 1000
                self.get_logger().info(
                    f'  Marker {p["marker_id"]}: err={err:.1f}mm')

        self.get_logger().info('[FIX 1] Bottom-edge midpoint sampling')
        self.get_logger().info('[FIX 2] Per-instance centroid tracking')

        # RealSense
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = self.pipeline.start(cfg)
        self.align = rs.align(rs.stream.color)
        depth_profile = profile.get_stream(rs.stream.depth)
        self.intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()

        # FIX 2: Tracker
        self.tracker = CentroidTracker()

        if SHOW_DISPLAY:
            self.display_thread = threading.Thread(
                target=self.display_loop, daemon=True)
            self.display_thread.start()

        self.get_logger().info(
            'OBB Detector (improved — all fixes) ready')

    def display_loop(self):
        while self.running:
            with self.frame_lock:
                frame = self.display_frame.copy() \
                    if self.display_frame is not None else None
            if frame is not None:
                cv2.imshow("OBB Plastic Sorter [improved]", frame)
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
        results = self.model(color_img, verbose=False, imgsz=640, device='cpu')[0]
        display = results.plot()

        # ── Gather raw detections ──
        raw_detections = []

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

                # FIX 1: Bottom-edge midpoint
                bx, by = obb_bottom_midpoint(
                    xywhr[0], xywhr[1], w_px, h_px, angle_rad, img_w, img_h)

                # Depth at bottom point (for height estimation only)
                depth_m = get_median_depth(depth_frame, bx, by)
                if depth_m is None:
                    depth_m = get_median_depth(depth_frame, cx, cy)
                if depth_m is None or depth_m <= 0.01 or depth_m > 3.0:
                    depth_m = 0.6  # fallback — homography doesn't need it for XY

                # FIX 4: XY via homography
                if self.H is not None:
                    x_m, y_m = homography_map(self.H, bx, by)
                    raw_robot = {"x": x_m, "y": y_m,
                                 "z": self.table_plane.z_at(x_m, y_m)}
                else:
                    # Fallback: ray-plane with fitted plane
                    cam_xyz = rs.rs2_deproject_pixel_to_point(
                        self.intrinsics, [bx, by], depth_m)
                    O = self.T[:3, 3]
                    P = (self.T @ np.array([*cam_xyz, 1.0]))[:3]
                    D = P - O
                    if abs(D[2]) < 1e-6:
                        continue
                    z_guess = self.table_plane.c
                    ray_ok = True
                    for _ in range(3):
                        t = (z_guess - O[2]) / D[2]
                        if t <= 0:
                            ray_ok = False
                            break
                        x_m = O[0] + t * D[0]
                        y_m = O[1] + t * D[1]
                        z_guess = self.table_plane.z_at(x_m, y_m)
                    if not ray_ok:
                        continue
                    raw_robot = {"x": float(x_m), "y": float(y_m),
                                 "z": float(z_guess)}

                raw_detections.append({
                    "cls_name":  cls_name,
                    "x_mm":      raw_robot["x"] * 1000,
                    "y_mm":      raw_robot["y"] * 1000,
                    "raw_robot": raw_robot,
                    "depth_m":   depth_m,
                    "cx": cx, "cy": cy,
                    "bx": bx, "by": by,
                    "w_px": w_px, "h_px": h_px,
                    "angle_rad": angle_rad,
                    "angle_deg": angle_deg,
                    "conf": conf,
                })

        # FIX 2: Update tracker
        matched = self.tracker.update(raw_detections)

        # ── Build outputs ──
        stable = []

        for track, det in matched:
            # Per-track spike filter
            depth_m = det["depth_m"]
            if track.prev_depth is not None and \
                    abs(depth_m - track.prev_depth) > DEPTH_JUMP_THRESHOLD:
                depth_m = track.prev_depth

            cx, cy = det["cx"], det["cy"]
            bx, by = det["bx"], det["by"]
            w_px, h_px = det["w_px"], det["h_px"]
            angle_rad = det["angle_rad"]
            angle_deg = det["angle_deg"]
            conf = det["conf"]
            cls_name = det["cls_name"]

            # FIX 2: Smoothed position from per-track history
            x_smooth, y_smooth = track.smoothed_position_mm()

            # FIX 3: Z from the same fitted plane
            z_table    = self.table_plane.z_at_mm(x_smooth, y_smooth)
            z_approach = z_table + APPROACH_OFFSET
            z_grip     = z_table + GRIP_OFFSET

            # FIX 4: Dimensions via homography (perspective-correct)
            if self.H is not None:
                dx_mm, dy_mm = obb_dimensions_via_homography(
                    self.H, cx, cy, w_px, h_px, angle_rad)
            else:
                fx = self.intrinsics.fx
                dx_mm = pixels_to_mm(w_px, depth_m, fx)
                dy_mm = pixels_to_mm(h_px, depth_m, fx)

            dz_mm, dz_source = compute_dz(
                depth_frame, self.intrinsics, cx, cy, w_px / 2, depth_m)

            quaternion = angle_to_quaternion(angle_rad)

            # Display
            stable_str = "STABLE" if track.is_stable \
                else f"stabilising {track.detection_count}/{STABILITY_FRAMES}"
            colour = (0, 255, 0) if track.is_stable else (0, 165, 255)
            track_colour = [
                (0, 255, 0), (255, 128, 0), (0, 200, 255),
                (255, 0, 255), (128, 255, 0), (0, 128, 255),
            ][track.track_id % 6]

            if SHOW_DISPLAY:
                cv2.circle(display, (bx, by), 5, (255, 0, 255), -1)
                cv2.line(display, (cx, cy), (bx, by), (255, 0, 255), 1)

                cv2.putText(display,
                            f"T{track.track_id} {cls_name} | {stable_str}",
                            (cx - 80, cy - 52),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, track_colour, 2)
                cv2.putText(display,
                            f"[pub]  x={x_smooth:.0f} y={y_smooth:.0f} "
                            f"z={z_grip:.0f}mm",
                            (cx - 80, cy - 34),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, colour, 2)
                cv2.putText(display,
                            f"dx={dx_mm:.0f} dy={dy_mm:.0f} "
                            f"dz={dz_mm}mm a={angle_deg:.1f}",
                            (cx - 80, cy - 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, colour, 1)

            dimensions = {"dx_mm": round(dx_mm, 1), "dy_mm": round(dy_mm, 1)}
            if dz_mm is not None:
                dimensions["dz_mm"] = dz_mm

            detection_msg = {
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
                    "track_id":      track.track_id,
                    "track_age":     track.detection_count,
                    "xy_method":     "homography" if self.H else "ray",
                    "sample_point":  "bottom_edge",
                    "bottom_px":     [bx, by],
                }
            }

            if track.is_stable:
                stable.append(detection_msg)

        # HUD
        if SHOW_DISPLAY:
            n_tracks = len(self.tracker.tracks)
            n_stable = sum(1 for t in self.tracker.tracks if t.is_stable)
            status = f"PUBLISHING ({n_stable}/{n_tracks})" \
                if n_stable > 0 else f"Stabilising ({n_tracks} tracks)"
            colour = (0, 255, 0) if n_stable > 0 else (0, 165, 255)
            cv2.putText(display, f"{status} [improved]", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)
            cv2.putText(display, "Q = quit", (10, 460),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)

            with self.frame_lock:
                self.display_frame = display

        if stable:
            msg = String()
            msg.data = json.dumps(stable)
            self.publisher_.publish(msg)
            self.get_logger().info(
                f'Published {len(stable)} stable: '
                f'{[(d["classification"]["class"], d["debug"]["track_id"]) for d in stable]}'
            )

    def destroy_node(self):
        self.running = False
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OBBDetectorImproved()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
