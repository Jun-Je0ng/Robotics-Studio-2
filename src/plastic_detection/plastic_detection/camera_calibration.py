"""
camera_calibration.py  —  Automatic Camera-to-Robot Calibration
================================================================
Detects 4 ArUco markers on the tray corners and computes the camera→robot
transform automatically.  No manual pendant entry required — the robot-frame
corner positions are hardcoded below (measure once, reuse forever).

HOW TO USE
----------
1. Fill in TRAY_CORNERS_ROBOT_M with the actual robot-frame XYZ (in metres)
   for each ArUco marker corner.  Do this ONCE by moving the EEF directly
   above each marker and reading the pendant.
2. Place the tray in its fixed position.
3. Run this script:  python3 camera_calibration.py
4. The live feed shows which markers are detected (green = all 4 ready).
5. Press SPACE (or wait for auto-capture) to compute and save the calibration.
6. Press Q to quit without saving.

WHY THIS IS AUTOMATIC
---------------------
The tray is always in the same position relative to the robot (±5 mm), so
the robot-frame corner coordinates are fixed.  Only the camera-frame positions
need to be measured at run-time — and ArUco detection does that automatically.

DEPTH FIX
---------
Uses the same 11×11 median kernel as the detector (not a single raw pixel)
to reduce noise at the calibration corners.
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import json
import os

# ──────────────────────────────────────────────────────────────────────────────
# TRAY CORNER POSITIONS IN ROBOT BASE FRAME  (metres)
#
# Key   = ArUco marker ID (0..3)
# Value = (x, y, z) measured with EEF directly above each corner marker
#
# ▶ Update these values whenever the tray or robot base is repositioned.
#   These come from reading the pendant (or ros2 topic echo /tf) while the
#   EEF is touching the table surface at each corner.
# ──────────────────────────────────────────────────────────────────────────────
TRAY_CORNERS_ROBOT_M = {
    0: (0.20260,  -0.18140, 0.05164),   # front-right  corner   // from camera perspective/side
    1: (0.20003,  -0.42785, 0.05571),   # back-right   corner
    2: ( -0.21212,  -0.18155, 0.05210),   # front-left corner
    3: ( -0.21399,  -0.41729, 0.04784),   # back-left  corner
}

# TRAY_CORNERS_ROBOT_M = { # useful for testing on table without robot
#     0: (0.2075,  -0.035, 0.0),   # front-right  corner   // from camera perspective/side
#     1: (0.2075,  -0.282, 0.0),   # back-right   corner
#     2: ( -0.2075,  -0.035, 0.0),   # front-left corner
#     3: ( -0.2075,  -0.282, 0.0),   # back-left  corner
# }

CALIBRATION_FILE  = "camera_to_robot_calibration.json"
DEPTH_KERNEL_SIZE = 11   # same as detector — median over k×k pixel patch
AUTO_CAPTURE      = True # set False to require SPACE for each marker

# ──────────────────────────────────────────────────────────────────────────────

def median_depth(depth_frame, cx, cy, k=DEPTH_KERNEL_SIZE):
    """Return median depth (metres) over a k×k patch centred on (cx, cy)."""
    h = depth_frame.get_height()
    w = depth_frame.get_width()
    half = k // 2
    x0, x1 = max(0, cx - half), min(w, cx + half + 1)
    y0, y1 = max(0, cy - half), min(h, cy + half + 1)
    arr    = np.asanyarray(depth_frame.get_data())
    region = arr[y0:y1, x0:x1]
    valid  = region[region > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid)) / 1000.0


def detect_all_markers(frame, depth_frame, intr):
    """
    Detect all visible ArUco markers (DICT_4X4_50).
    Returns dict: {marker_id: {'x', 'y', 'z', 'center_x', 'center_y'}}
    Depth measured with median kernel.
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector   = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(frame)

    result = {}
    if ids is None:
        return result

    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    fx, fy = intr.fx, intr.fy
    ppx, ppy = intr.ppx, intr.ppy

    for i, marker_id in enumerate(ids.flatten()):
        mid = int(marker_id)
        if mid not in TRAY_CORNERS_ROBOT_M:
            continue

        mc = corners[i][0]
        cx = int(np.mean(mc[:, 0]))
        cy = int(np.mean(mc[:, 1]))

        z = median_depth(depth_frame, cx, cy)
        if z is None or z <= 0:
            continue

        x = (cx - ppx) * z / fx
        y = (cy - ppy) * z / fy

        result[mid] = {
            'x': x, 'y': y, 'z': z,
            'center_x': cx, 'center_y': cy,
        }

    return result


def compute_transform(camera_points, robot_points):
    """SVD rigid transform (Horn's method): camera frame → robot frame."""
    cam   = np.array([[p['x'], p['y'], p['z']] for p in camera_points])
    robot = np.array([[p['x'], p['y'], p['z']] for p in robot_points])

    cam_c   = np.mean(cam,   axis=0)
    robot_c = np.mean(robot, axis=0)

    H  = (cam - cam_c).T @ (robot - robot_c)
    U, _, Vt = np.linalg.svd(H)
    R  = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T

    t = robot_c - R @ cam_c

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3]  = t
    return T


def compute_homography(calibration_pts):
    """
    2D homography: image pixel (u, v) → robot XY in mm.
    This is the correct approach for 4 coplanar calibration points —
    the 3D SVD fit is degenerate when all robot points share the same Z.
    """
    src = np.array(
        [[p['camera']['center_x'], p['camera']['center_y']] for p in calibration_pts],
        dtype=np.float32)
    dst = np.array(
        [[p['robot']['x'] * 1000, p['robot']['y'] * 1000] for p in calibration_pts],
        dtype=np.float32)
    H, _ = cv2.findHomography(src, dst)
    return H


# ──────────────────────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────────────────────

pipeline = rs.pipeline()
config   = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

profile       = pipeline.get_active_profile()
depth_profile = profile.get_stream(rs.stream.depth)
intr          = depth_profile.as_video_stream_profile().get_intrinsics()

print(f"Camera intrinsics: fx={intr.fx:.2f} fy={intr.fy:.2f} "
      f"cx={intr.ppx:.2f} cy={intr.ppy:.2f}")

align = rs.align(rs.stream.color)

print("\n=== Automatic Camera-to-Robot Calibration ===")
print("Place tray in fixed position.  All 4 ArUco markers must be visible.")
print("SPACE = capture now   |   Q = quit")
if AUTO_CAPTURE:
    print("AUTO-CAPTURE enabled — will save as soon as all 4 markers are stable.\n")

captured = {}          # {marker_id: {'camera': ..., 'robot': ...}}
STABLE_FRAMES = 10     # require marker visible for N consecutive frames before auto-capture
stable_counts = {}     # {marker_id: consecutive frame count}

try:
    while True:
        frames         = pipeline.wait_for_frames()
        aligned        = align.process(frames)
        depth_frame    = aligned.get_depth_frame()
        color_frame    = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        frame    = np.asanyarray(color_frame.get_data())
        detected = detect_all_markers(frame, depth_frame, intr)

        # Update stable counts
        for mid in list(TRAY_CORNERS_ROBOT_M.keys()):
            if mid in detected and mid not in captured:
                stable_counts[mid] = stable_counts.get(mid, 0) + 1
            else:
                stable_counts[mid] = 0

        # Overlay status
        needed = set(TRAY_CORNERS_ROBOT_M.keys()) - set(captured.keys())
        all_stable = all(
            stable_counts.get(mid, 0) >= STABLE_FRAMES for mid in needed
        )

        for mid, cam in detected.items():
            if mid in captured:
                colour = (180, 180, 180)
                label  = f"ID {mid} — DONE"
            elif stable_counts.get(mid, 0) >= STABLE_FRAMES:
                colour = (0, 255, 0)
                label  = f"ID {mid} — READY"
            else:
                n = stable_counts.get(mid, 0)
                colour = (0, 165, 255)
                label  = f"ID {mid} — stabilising {n}/{STABLE_FRAMES}"
            cv2.putText(frame, label,
                        (cam['center_x'] - 60, cam['center_y'] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)

        status_colour = (0, 255, 0) if all_stable and needed else (0, 165, 255)
        status_text   = (
            f"Captured: {sorted(captured.keys())} | "
            f"Pending: {sorted(needed)} | "
            f"{'ALL READY — press SPACE or auto-capture' if all_stable and needed else ''}"
        )
        cv2.putText(frame, status_text, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, status_colour, 1)
        cv2.putText(frame, "SPACE=capture  Q=quit", (10, 460),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)

        cv2.imshow('Calibration', frame)
        key = cv2.waitKey(1) & 0xFF

        def do_capture():
            for mid in list(needed):
                if stable_counts.get(mid, 0) >= STABLE_FRAMES and mid in detected:
                    rx, ry, rz = TRAY_CORNERS_ROBOT_M[mid]
                    captured[mid] = {
                        'marker_id': mid,
                        'camera':    detected[mid],
                        'robot':     {'x': rx, 'y': ry, 'z': rz},
                    }
                    print(f"  ✓ Marker {mid} captured — "
                          f"cam=({detected[mid]['x']:.4f}, "
                          f"{detected[mid]['y']:.4f}, "
                          f"{detected[mid]['z']:.4f})  "
                          f"robot=({rx:.4f}, {ry:.4f}, {rz:.4f})")

        if AUTO_CAPTURE and all_stable and needed:
            print("\nAll markers stable — auto-capturing...")
            do_capture()

        if key == ord(' '):
            if all_stable and needed:
                do_capture()
            elif needed:
                print("Not all needed markers are stable yet.")

        if key == ord('q'):
            print("Quit — no calibration saved.")
            break

        # All captured → compute and save
        if not needed:
            print(f"\nAll {len(captured)} markers captured — computing transform...")
            pts = list(captured.values())
            cam_pts   = [p['camera'] for p in pts]
            robot_pts = [p['robot']  for p in pts]

            T   = compute_transform(cam_pts, robot_pts)
            H_pix = compute_homography(pts)

            # Homography reprojection (pixel → robot XY)
            print("Homography pixel→robot XY test:")
            total_h_err = 0.0
            for p in pts:
                px = np.array([[[p['camera']['center_x'],
                                  p['camera']['center_y']]]], dtype=np.float32)
                pred_xy = cv2.perspectiveTransform(px, H_pix)[0][0]
                actual  = np.array([p['robot']['x'] * 1000,
                                     p['robot']['y'] * 1000])
                err = np.linalg.norm(pred_xy - actual)
                total_h_err += err
                print(f"  Marker {p['marker_id']}: "
                      f"pred=({pred_xy[0]:.1f}, {pred_xy[1]:.1f})mm "
                      f"actual=({actual[0]:.1f}, {actual[1]:.1f})mm "
                      f"err={err:.1f}mm")
            print(f"  Average homography error: {total_h_err/len(pts):.1f}mm")

            # Reprojection error (3D SVD — kept for reference only)
            print("3D SVD reprojection test (reference):")
            total_err = 0.0
            for p in pts:
                cam   = p['camera']
                rob   = p['robot']
                pt    = np.array([cam['x'], cam['y'], cam['z'], 1.0])
                pred  = T @ pt
                err   = np.linalg.norm(pred[:3] - np.array(
                    [rob['x'], rob['y'], rob['z']]))
                total_err += err
                print(f"  Marker {p['marker_id']}: "
                      f"pred=({pred[0]*1000:.1f}, {pred[1]*1000:.1f}, {pred[2]*1000:.1f})mm "
                      f"actual=({rob['x']*1000:.1f}, {rob['y']*1000:.1f}, {rob['z']*1000:.1f})mm "
                      f"err={err*1000:.1f}mm")
            print(f"  Average error: {(total_err/len(pts))*1000:.1f}mm")

            calibration_data = {
                'transform_matrix':          T.tolist(),
                'homography_pixel_to_robot_mm': H_pix.tolist(),
                'calibration_points': pts,
                'timestamp':          str(np.datetime64('now')),
                'description':        'Camera to UR3e base frame — auto calibration',
                'tray_corners_robot': {
                    str(k): list(v)
                    for k, v in TRAY_CORNERS_ROBOT_M.items()
                },
            }
            with open(CALIBRATION_FILE, 'w') as f:
                json.dump(calibration_data, f, indent=2)
            print(f"\n✓ Calibration saved to {CALIBRATION_FILE}")
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()