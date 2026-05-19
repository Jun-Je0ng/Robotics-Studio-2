#!/usr/bin/env python3
"""
Grip Pose Node
===============
Subscribes to /plastic_detections
Computes grip pose for parallel jaw gripper
Publishes to /grip_pose

Logic:
- Grip point = centre of detected bottle
- Jaw orientation = perpendicular to bottle long axis
- Approach direction = from above (upright) or side (horizontal)
- Jaw opening = bottle diameter (dz)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
from scipy.spatial.transform import Rotation

# ──────────────────────────────────────────────────────────────────────────────
# CONFIG
# ──────────────────────────────────────────────────────────────────────────────

# If abs(angle_deg) < this threshold, bottle is considered upright
UPRIGHT_THRESHOLD_DEG = 20.0

# Safety margin added to jaw opening (mm)
JAW_SAFETY_MARGIN_MM = 10.0

# ──────────────────────────────────────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────────────────────────────────────

def compute_grip_pose(detection):
    """
    Given a detection dict from /plastic_detections,
    compute the grip pose for the parallel jaw gripper.
    """
    pos       = detection["pose"]["position"]
    dims      = detection["dimensions"]
    angle_deg = detection["debug"]["angle_deg"]
    angle_rad = detection["debug"]["angle_rad"]
    cls_name  = detection["classification"]["class"]
    conf      = detection["classification"]["confidence"]

    x = pos["x"]
    y = pos["y"]
    z = pos["z"]

    dx = dims["dx_mm"]
    dy = dims["dy_mm"]
    dz = dims.get("dz_mm", None)  # may not exist if depth failed

    # ── Determine bottle orientation ──
    # Long axis = whichever OBB dimension is larger
    long_axis_px  = max(dx, dy)
    short_axis_px = min(dx, dy)

    is_upright = abs(angle_deg) < UPRIGHT_THRESHOLD_DEG

    # ── Jaw orientation ──

    # Perpendicular to long axis = jaw closing direction
    jaw_dir = np.array([-np.sin(angle_rad), np.cos(angle_rad), 0.0])

    # ── Approach direction ──
    if is_upright:
        # Bottle standing up → approach from above
        approach_vec = np.array([0.0, 0.0, -1.0])
        orientation_label = "top_down"
    else:
        # Bottle on its side → approach from above still
        # (safest for UR3e — avoids collisions with table)
        approach_vec = np.array([0.0, 0.0, -1.0])
        orientation_label = "side_down"

    # ── Grip quaternion ──
    # Build rotation matrix from jaw direction and approach vector
    # X axis = jaw closing direction
    # Z axis = approach direction (flipped — pointing away from object)
    x_axis = jaw_dir
    z_axis = -approach_vec
    y_axis = np.cross(z_axis, x_axis)

    # Normalise
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    z_axis = z_axis / np.linalg.norm(z_axis)

    rot_matrix = np.column_stack([x_axis, y_axis, z_axis])
    r = Rotation.from_matrix(rot_matrix)
    q = r.as_quat()  # [x, y, z, w]

    # ── Jaw opening ──
    if dz is not None:
        jaw_opening_mm = dz + JAW_SAFETY_MARGIN_MM
    else:
        # Fall back to short axis dimension if dz unavailable
        jaw_opening_mm = short_axis_px + JAW_SAFETY_MARGIN_MM

    return {
        "class":            cls_name,
        "confidence":       conf,
        "grip_position": {
            "x": round(x, 1),
            "y": round(y, 1),
            "z": round(z, 1),
        },
        "grip_orientation": {
            "qx": round(float(q[0]), 6),
            "qy": round(float(q[1]), 6),
            "qz": round(float(q[2]), 6),
            "qw": round(float(q[3]), 6),
        },
        "jaw_opening_mm":   round(jaw_opening_mm, 1),
        "approach":         orientation_label,
        "debug": {
            "angle_deg":       round(angle_deg, 2),
            "is_upright":      is_upright,
            "long_axis_mm":    round(long_axis_px, 1),
            "short_axis_mm":   round(short_axis_px, 1),
            "jaw_dir":         [round(v, 4) for v in jaw_dir.tolist()],
            "approach_vec":    [round(v, 4) for v in approach_vec.tolist()],
        }
    }

# ──────────────────────────────────────────────────────────────────────────────
# NODE
# ──────────────────────────────────────────────────────────────────────────────

class GripPoseNode(Node):
    def __init__(self):
        super().__init__('grip_pose_node')

        self.subscription = self.create_subscription(
            String,
            'plastic_detections',
            self.detection_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            String,
            'grip_pose',
            10
        )

        self.get_logger().info(
            'Grip Pose Node ready — '
            'subscribing to /plastic_detections, '
            'publishing to /grip_pose'
        )

    def detection_callback(self, msg):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse detection message')
            return

        grip_poses = []

        for detection in detections:
            grip = compute_grip_pose(detection)
            grip_poses.append(grip)
            self.get_logger().info(
                f"Grip computed for {grip['class']} | "
                f"pos=({grip['grip_position']['x']:.0f}, "
                f"{grip['grip_position']['y']:.0f}, "
                f"{grip['grip_position']['z']:.0f}) mm | "
                f"jaw={grip['jaw_opening_mm']}mm | "
                f"approach={grip['approach']}"
            )

        if grip_poses:
            out_msg = String()
            out_msg.data = json.dumps(grip_poses)
            self.publisher_.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()