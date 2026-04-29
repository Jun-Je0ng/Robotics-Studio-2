import pyrealsense2 as rs
import numpy as np
import cv2
import json
import os

# Configuration
CALIBRATION_FILE = "camera_to_robot_calibration.json"

# RealSense setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline.start(config)

# Get camera intrinsics
profile      = pipeline.get_active_profile()
depth_profile = profile.get_stream(rs.stream.depth)
intr         = depth_profile.as_video_stream_profile().get_intrinsics()
fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy

print(f"Camera intrinsics: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")

# Calibration data storage
calibration_points = []

# Which marker ID to capture next
target_marker_id = 0

def detect_target_marker(frame, depth_frame, target_id):
    """
    Detect a specific ArUco marker by ID.
    Uses the live depth frame passed in.
    Returns marker info if the target ID is detected, otherwise None.
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector   = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    corners, ids, rejected = detector.detectMarkers(frame)

    # Draw all detected markers for visual feedback
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    if ids is None or len(ids) == 0:
        return None

    # Look specifically for the target marker ID
    for i, marker_id in enumerate(ids.flatten()):
        if marker_id == target_id:
            marker_corners = corners[i][0]

            center_x = int(np.mean(marker_corners[:, 0]))
            center_y = int(np.mean(marker_corners[:, 1]))

            # Use live depth frame
            depth_img = np.asanyarray(depth_frame.get_data())
            depth_val = depth_img[center_y, center_x]

            if depth_val <= 0:
                return {'detected': True, 'error': 'invalid_depth'}

            z = depth_val / 1000.0
            x = (center_x - cx) * z / fx
            y = (center_y - cy) * z / fy

            return {
                'x': x, 'y': y, 'z': z,
                'detected': True,
                'marker_id': int(marker_id),
                'center_x': center_x,
                'center_y': center_y,
            }

    # Target marker not found
    return None

def get_robot_pose_from_user():
    """Get current robot pose from user input."""
    print("\nEnter current UR3e end-effector pose (in robot base frame):")
    print("(Enter values in MILLIMETRES — will be converted to metres automatically)")
    try:
        x = float(input("X position (mm): ")) / 1000.0
        y = float(input("Y position (mm): ")) / 1000.0
        z = float(input("Z position (mm): ")) / 1000.0
        print(f"  → Converted to metres: x={x:.4f}, y={y:.4f}, z={z:.4f}")
        return {'x': x, 'y': y, 'z': z}
    except ValueError:
        print("Invalid input. Please enter numbers.")
        return None

def compute_transform(camera_points, robot_points):
    """Compute rigid transform from camera to robot frame using Horn's method."""
    if len(camera_points) < 3:
        print("Need at least 3 calibration points")
        return None

    cam_pts   = np.array([[p['x'], p['y'], p['z']] for p in camera_points])
    robot_pts = np.array([[p['x'], p['y'], p['z']] for p in robot_points])

    cam_centroid   = np.mean(cam_pts, axis=0)
    robot_centroid = np.mean(robot_pts, axis=0)

    cam_centered   = cam_pts - cam_centroid
    robot_centered = robot_pts - robot_centroid

    H = cam_centered.T @ robot_centered

    U, s, Vt = np.linalg.svd(H)

    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T

    t = robot_centroid - R @ cam_centroid

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3]  = t

    return T

try:
    print("\n=== Camera-to-Robot Calibration ===")
    print("Instructions:")
    print("1. Move UR3e end-effector directly above marker 0")
    print("2. Confirm marker 0 is detected (green outline) in camera feed")
    print("3. Press SPACE to capture — enter teach pendant XYZ when prompted")
    print("4. Script will automatically move to next marker ID")
    print("5. Repeat until all 4 markers captured")
    print("6. Press C to compute and save calibration")
    print("7. Press Q to quit")
    print(f"\n→ Currently targeting marker ID: {target_marker_id}")

    align = rs.align(rs.stream.color)

    while True:
        frames         = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame    = aligned_frames.get_depth_frame()
        color_frame    = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())

        # Detect only the target marker using live depth frame
        marker_result = detect_target_marker(frame, depth_frame, target_marker_id)

        # Status display
        captured_ids = [p['marker_id'] for p in calibration_points]
        status_text  = (f"Points: {len(calibration_points)}/4 captured "
                        f"{captured_ids} | Target: marker {target_marker_id}")

        if marker_result is None:
            cv2.putText(frame, f"Waiting for marker {target_marker_id}...",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        elif 'error' in marker_result:
            cv2.putText(frame, f"Marker {target_marker_id} detected — INVALID DEPTH",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        else:
            # Draw crosshair at marker centre
            cx_m, cy_m = marker_result['center_x'], marker_result['center_y']
            cv2.drawMarker(frame, (cx_m, cy_m), (0, 255, 0),
                           cv2.MARKER_CROSS, 20, 2)
            cv2.putText(frame,
                        f"Marker {target_marker_id} DETECTED — hover gripper above, press SPACE",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.putText(frame, status_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, "SPACE=capture | C=compute | Q=quit",
                    (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow('Calibration', frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):
            if marker_result is None:
                print(f"Marker {target_marker_id} not detected — make sure it's visible")
            elif 'error' in marker_result:
                print(f"Marker {target_marker_id} detected but depth is invalid — reposition")
            else:
                print(f"\nMarker {target_marker_id} detected at camera position: "
                      f"x={marker_result['x']:.4f}, "
                      f"y={marker_result['y']:.4f}, "
                      f"z={marker_result['z']:.4f}")

                robot_pose = get_robot_pose_from_user()
                if robot_pose:
                    calibration_points.append({
                        'marker_id': target_marker_id,
                        'camera':    marker_result,
                        'robot':     robot_pose,
                    })
                    print(f"✓ Calibration point {len(calibration_points)} saved "
                          f"(marker {target_marker_id})")

                    # Automatically advance to next marker
                    target_marker_id += 1
                    if target_marker_id <= 3:
                        print(f"\n→ Now move to marker {target_marker_id} and press SPACE")
                    else:
                        print("\n→ All 4 markers captured! Press C to compute calibration.")

        elif key == ord('c'):
            if len(calibration_points) >= 3:
                camera_points_list = [p['camera'] for p in calibration_points]
                robot_points_list  = [p['robot']  for p in calibration_points]

                T = compute_transform(camera_points_list, robot_points_list)

                if T is not None:
                    calibration_data = {
                        'transform_matrix':   T.tolist(),
                        'calibration_points': calibration_points,
                        'timestamp':          str(np.datetime64('now')),
                        'description':        'Camera to UR3e base frame calibration'
                    }

                    with open(CALIBRATION_FILE, 'w') as f:
                        json.dump(calibration_data, f, indent=2)

                    print(f"\n✓ Calibration saved to {CALIBRATION_FILE}")
                    print("Transform matrix:")
                    print(T)

                    # Test all captured points
                    print("\nReprojection test (all points):")
                    total_error = 0.0
                    for p in calibration_points:
                        cam   = p['camera']
                        rob   = p['robot']
                        pt    = np.array([cam['x'], cam['y'], cam['z'], 1.0])
                        pred  = T @ pt
                        error = np.linalg.norm(pred[:3] - np.array(
                            [rob['x'], rob['y'], rob['z']]))
                        total_error += error
                        print(f"  Marker {p['marker_id']}: "
                              f"predicted=({pred[0]*1000:.1f}, {pred[1]*1000:.1f}, {pred[2]*1000:.1f})mm "
                              f"measured=({rob['x']*1000:.1f}, {rob['y']*1000:.1f}, {rob['z']*1000:.1f})mm "
                              f"error={error*1000:.1f}mm")
                    print(f"  Average error: {(total_error/len(calibration_points))*1000:.1f}mm")

            else:
                print(f"Need at least 3 points, have {len(calibration_points)}")

        elif key == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

    if os.path.exists(CALIBRATION_FILE):
        print(f"\nCalibration saved to {CALIBRATION_FILE}")
    else:
        print("\nNo calibration saved.")