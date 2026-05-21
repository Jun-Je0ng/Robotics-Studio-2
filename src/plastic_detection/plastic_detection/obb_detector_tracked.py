#!/usr/bin/env python3
"""
OBB Plastic Detection — FIX 2: Centroid-Based Object Tracking
================================================================
Drop-in replacement for obb_detector_node.py.

PROBLEM FIXED
-------------
The original detector keys pose history, depth history, and detection
counts by *class name*.  This means:
  - Two bottles of the same class corrupt each other's history
  - Temporal smoothing mixes positions of different objects
  - Depth spike filtering applies one object's depth to another

FIX
---
Each detection is assigned a persistent **track ID** using nearest-centroid
association.  A simple tracker maintains a set of active tracks, each with
its own pose history, depth history, and detection count.

Association:
  1. Compute robot-frame XY for each new detection.
  2. For each new detection, find the nearest existing track (Euclidean in
     robot-frame mm).
  3. If distance < ASSOC_THRESHOLD_MM, assign to that track.
  4. Otherwise, create a new track.
  5. Tracks not seen for TRACK_TIMEOUT frames are pruned.

Published messages now include a "track_id" field alongside the class name.

Run:  ros2 run plastic_detection obb_detector_tracked
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

# Tracking parameters
ASSOC_THRESHOLD_MM = 80.0   # max distance to associate detection with track
TRACK_TIMEOUT      = 10     # frames without detection before track is pruned

SHOW_DISPLAY = True

# ──────────────────────────────────────────────────────────────────────────────
# WORK PLANE
# ──────────────────────────────────────────────────────────────────────────────

def get_table_z(x_mm, y_mm):
    return 0.001841 * x_mm + -0.007761 * y_mm + -185.5378

# ──────────────────────────────────────────────────────────────────────────────
# HELPERS  (unchanged from original)
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
# SIMPLE CENTROID TRACKER
# ──────────────────────────────────────────────────────────────────────────────

class Track:
    """One tracked object instance."""
    _next_id = 0

    def __init__(self, cls_name, x_mm, y_mm):
        self.track_id = Track._next_id
        Track._next_id += 1

        self.cls_name = cls_name
        self.x_mm = x_mm       # latest robot-frame position (mm)
        self.y_mm = y_mm

        self.pose_history = deque(maxlen=SMOOTHING_WINDOW)
        self.prev_depth = None
        self.detection_count = 0
        self.frames_missing = 0

    def update(self, raw_robot, depth_m):
        """Update track with a new detection."""
        self.pose_history.append(raw_robot)
        self.x_mm = raw_robot["x"] * 1000
        self.y_mm = raw_robot["y"] * 1000
        self.prev_depth = depth_m
        self.detection_count += 1
        self.frames_missing = 0

    def mark_missing(self):
        """Called when the track is not seen in a frame."""
        self.frames_missing += 1
        # Reset detection count (consecutive frames requirement)
        self.detection_count = 0

    @property
    def is_expired(self):
        return self.frames_missing > TRACK_TIMEOUT

    @property
    def is_stable(self):
        return self.detection_count >= STABILITY_FRAMES

    def smoothed_position_mm(self):
        """Return temporally smoothed position in mm."""
        hist = self.pose_history
        if not hist:
            return self.x_mm, self.y_mm
        x = float(np.mean([p["x"] for p in hist])) * 1000
        y = float(np.mean([p["y"] for p in hist])) * 1000
        return x, y


class CentroidTracker:
    """
    Associates detections to persistent tracks using nearest-centroid
    in robot-frame XY (mm).
    """
    def __init__(self, assoc_threshold_mm=ASSOC_THRESHOLD_MM):
        self.tracks = []
        self.threshold = assoc_threshold_mm

    def update(self, detections):
        """
        detections: list of dicts with keys:
            cls_name, x_mm, y_mm, raw_robot, depth_m, ...extra
        Returns: list of (track, detection_dict) pairs for matched detections.
        """
        # Mark all tracks as missing initially
        for t in self.tracks:
            t.mark_missing()

        matched = []
        used_tracks = set()

        # Greedy nearest-neighbour matching
        # Sort detections and try to match each to the nearest track
        for det in detections:
            best_track = None
            best_dist = float('inf')

            for t in self.tracks:
                if id(t) in used_tracks:
                    continue
                # Only match same class
                if t.cls_name != det["cls_name"]:
                    continue
                dist = np.sqrt(
                    (t.x_mm - det["x_mm"])**2 +
                    (t.y_mm - det["y_mm"])**2
                )
                if dist < best_dist:
                    best_dist = dist
                    best_track = t

            if best_track is not None and best_dist < self.threshold:
                best_track.update(det["raw_robot"], det["depth_m"])
                used_tracks.add(id(best_track))
                matched.append((best_track, det))
            else:
                # Create new track
                new_track = Track(det["cls_name"], det["x_mm"], det["y_mm"])
                new_track.update(det["raw_robot"], det["depth_m"])
                self.tracks.append(new_track)
                matched.append((new_track, det))

        # Prune expired tracks
        self.tracks = [t for t in self.tracks if not t.is_expired]

        return matched


# ──────────────────────────────────────────────────────────────────────────────
# NODE
# ──────────────────────────────────────────────────────────────────────────────

class OBBDetectorTracked(Node):
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
            '[FIX 2] Using centroid-based object tracking (per-instance IDs)')

        # RealSense
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = self.pipeline.start(cfg)
        self.align = rs.align(rs.stream.color)
        depth_profile = profile.get_stream(rs.stream.depth)
        self.intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()

        # Tracker instead of class-name dicts
        self.tracker = CentroidTracker()

        if SHOW_DISPLAY:
            self.display_thread = threading.Thread(
                target=self.display_loop, daemon=True)
            self.display_thread.start()

        self.get_logger().info(
            f'OBB Detector (tracked) ready — publishing to /plastic_detections')

    def display_loop(self):
        while self.running:
            with self.frame_lock:
                frame = self.display_frame.copy() \
                    if self.display_frame is not None else None
            if frame is not None:
                cv2.imshow("OBB Plastic Sorter [tracked]", frame)
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
        display   = results.plot()

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

                depth_m = get_median_depth(depth_frame, cx, cy)
                if depth_m is None or depth_m <= 0.01 or depth_m > 3.0:
                    continue

                # Camera → robot via table-plane ray intersection
                cam_xyz = deproject(self.intrinsics, cx, cy, depth_m)
                xy = table_ray_intersect(self.T, cam_xyz, self.table_z_m)
                if xy is None:
                    raw_robot = camera_to_robot(self.T, cam_xyz)
                else:
                    raw_robot = {"x": xy[0], "y": xy[1], "z": self.table_z_m}

                raw_detections.append({
                    "cls_name":  cls_name,
                    "x_mm":      raw_robot["x"] * 1000,
                    "y_mm":      raw_robot["y"] * 1000,
                    "raw_robot":  raw_robot,
                    "depth_m":    depth_m,
                    "cam_xyz":    cam_xyz,
                    "cx": cx, "cy": cy,
                    "w_px": w_px, "h_px": h_px,
                    "angle_rad": angle_rad,
                    "angle_deg": angle_deg,
                    "conf": conf,
                })

        # ── Update tracker ──
        matched = self.tracker.update(raw_detections)

        # ── Build outputs ──
        stable = []

        for track, det in matched:
            # Spike filter using track's own depth history
            depth_m = det["depth_m"]
            if track.prev_depth is not None and \
                    abs(depth_m - track.prev_depth) > DEPTH_JUMP_THRESHOLD:
                depth_m = track.prev_depth

            cx, cy = det["cx"], det["cy"]
            w_px, h_px = det["w_px"], det["h_px"]
            angle_rad = det["angle_rad"]
            angle_deg = det["angle_deg"]
            conf = det["conf"]
            cls_name = det["cls_name"]

            # Smoothed position from track history
            x_smooth, y_smooth = track.smoothed_position_mm()

            z_table    = get_table_z(x_smooth, y_smooth)
            z_approach = z_table + APPROACH_OFFSET
            z_grip     = z_table + GRIP_OFFSET

            fx    = self.intrinsics.fx
            dx_mm = pixels_to_mm(w_px, depth_m, fx)
            dy_mm = pixels_to_mm(h_px, depth_m, fx)

            dz_mm, dz_source = compute_dz(
                depth_frame, self.intrinsics,
                cx, cy, w_px / 2, depth_m
            )

            quaternion = angle_to_quaternion(angle_rad)

            # Stability from track
            stable_str = "STABLE" if track.is_stable \
                else f"stabilising {track.detection_count}/{STABILITY_FRAMES}"
            colour = (0, 255, 0) if track.is_stable else (0, 165, 255)

            # Unique colour per track for visual distinction
            track_colour = [
                (0, 255, 0), (255, 128, 0), (0, 200, 255),
                (255, 0, 255), (128, 255, 0), (0, 128, 255),
            ][track.track_id % 6]

            if SHOW_DISPLAY:
                cv2.putText(display,
                            f"T{track.track_id} {cls_name} | {stable_str}",
                            (cx - 80, cy - 52),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, track_colour, 2)
                cv2.putText(display,
                            f"[pub]  x={x_smooth:.0f} "
                            f"y={y_smooth:.0f} "
                            f"z={z_grip:.0f}mm",
                            (cx - 80, cy - 34),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, colour, 2)
                cv2.putText(display,
                            f"angle={angle_deg:.1f}deg | "
                            f"dz={dz_mm}mm({dz_source})",
                            (cx - 80, cy - 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, colour, 1)

            dimensions = {
                "dx_mm": round(dx_mm, 1),
                "dy_mm": round(dy_mm, 1),
            }
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
                "track_id": track.track_id,
                "debug": {
                    "z_table_mm":    round(z_table, 1),
                    "z_approach_mm": round(z_approach, 1),
                    "angle_deg":     round(angle_deg, 2),
                    "angle_rad":     round(angle_rad, 4),
                    "depth_m":       round(depth_m, 4),
                    "dz_source":     dz_source,
                    "track_id":      track.track_id,
                    "track_age":     track.detection_count,
                }
            }

            if track.is_stable:
                stable.append(detection_msg)

        # HUD
        if SHOW_DISPLAY:
            n_tracks = len(self.tracker.tracks)
            n_stable = sum(1 for t in self.tracker.tracks if t.is_stable)
            status = f"PUBLISHING ({n_stable}/{n_tracks} tracks)" \
                if n_stable > 0 else f"Stabilising... ({n_tracks} tracks)"
            colour = (0, 255, 0) if n_stable > 0 else (0, 165, 255)
            cv2.putText(display, status, (10, 30),
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
                f'Published {len(stable)} stable detection(s): '
                f'{[(d["classification"]["class"], d["track_id"]) for d in stable]}'
            )

    def destroy_node(self):
        self.running = False
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OBBDetectorTracked()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
