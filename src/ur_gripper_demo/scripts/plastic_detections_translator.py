#!/usr/bin/env python3
"""
plastic_detections_translator.py

Subscribes to /plastic_detections (std_msgs/String — JSON array)
and republishes as /perception/objects (object_msgs/ObjectArray).

Unit conversion: mm → metres
Classification mapping: 'pet_bottle'/'hdpe_bottle' → 'plastic'
  (extend CLASS_MAP below if you add more ML classes)

Run standalone:
    ros2 run <your_pkg> plastic_detections_translator
Or include in your launch file alongside pick_place_demo.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, Point, Quaternion

from object_msgs.msg import Object, ObjectArray

import json


# Minimum confidence to pass through — detections below this are dropped
MIN_CONFIDENCE = 0.50
ROBOT_FRAME = 'base'
MM_TO_M = 1e-3

# ── Tray boundary clamp (robot base frame, metres) ────────────────────────────
# Objects reported outside these bounds are clamped to the tray edge.
# This catches gross OBB errors (camera angle / calibration drift) that would
# send the robot outside the reachable tray area.
# Set to None to disable clamping on that axis.
#
#         Y_MAX (far edge, away from robot)
#          ┌──────────────────────┐
#   X_MIN  │        TRAY          │  X_MAX
#          └──────────────────────┘
#         Y_MIN (near edge, towards robot)
#
TRAY_X_MIN =  -0.220   # m  — left  edge
TRAY_X_MAX =   0.220   # m  — right edge
TRAY_Y_MIN =  -0.440   # m  — near  edge
TRAY_Y_MAX =  -0.160   # m  — far   edge
# ──────────────────────────────────────────────────────────────────────────────

# Distance from tool0 / onrobot_base_link to gripper_tcp along the gripper Z axis.
# Source: rg2_macro.xacro  <origin xyz="0 0 0.218"/>  on tcp_joint.
# The calibration was performed with the pendant reporting the tool0 frame,
# so perceived z values are offset by this amount relative to the gripper fingertip.
# Adding it here converts to a frame where z=0 is the trolley work surface.
GRIPPER_TCP_OFFSET_M = 0.190


def mm_to_m(v: float) -> float:
    return v * MM_TO_M


class PlasticDetectionsTranslator(Node):

    def __init__(self):
        super().__init__('plastic_detections_translator')

        self.sub = self.create_subscription(
            String,
            'plastic_detections',
            self._callback,
            10
        )

        self.pub = self.create_publisher(
            ObjectArray,
            'perception/objects',
            10
        )

        self.get_logger().info(
            'Translator ready: /plastic_detections → /perception/objects'
        )

    def _callback(self, msg: String):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON parse error: {e}')
            return

        stamp     = self.get_clock().now().to_msg()
        obj_array = ObjectArray()
        # obj_array.header.frame_id = 'base_link'
        obj_array.header.frame_id = ROBOT_FRAME
        obj_array.header.stamp    = stamp

        for i, det in enumerate(detections):
            try:
                obj = self._convert(det, stamp)
            except (KeyError, TypeError) as e:
                self.get_logger().warn(f'Skipping detection [{i}]: {e}')
                continue

            if obj is None:
                continue   # filtered by confidence or unknown class

            obj_array.objects.append(obj)

        # Always publish — including empty arrays.
        # The motion controller uses consecutive empty messages to detect a clear
        # platform and terminate the pick loop.  Suppressing empty arrays here
        # would break that termination signal.
        self.pub.publish(obj_array)
        if obj_array.objects:
            self.get_logger().debug(
                f'Published {len(obj_array.objects)} object(s) on /perception/objects'
            )
        else:
            self.get_logger().debug('Published empty ObjectArray (platform clear)')

    def _convert(self, det: dict, stamp) -> Object | None:
        """
        Convert one /plastic_detections entry to an object_msgs/Object.
        Returns None if the detection should be dropped.
        """
        # ── Confidence filter ─────────────────────────────────────────────────
        confidence = det['classification']['confidence']
        if confidence < MIN_CONFIDENCE:
            self.get_logger().debug(
                f'Dropping detection — confidence {confidence:.2f} < {MIN_CONFIDENCE}'
            )
            return None

        # ── Classification — passed through as-is from ML node ───────────────
        ml_class = det['classification']['class']

        # ── Pose (mm → m) ─────────────────────────────────────────────────────
        pos = det['pose']['position']
        ori = det['pose']['orientation']

        pose = Pose()
        pose.position.x = mm_to_m(pos['x'])                      # camera X is mirrored relative to robot base_link X
        pose.position.y = mm_to_m(pos['y'])                      # UR pendant Y is opposite to URDF base_link Y
        pose.position.z =  mm_to_m(pos['z']) + GRIPPER_TCP_OFFSET_M  # shift from tool0 frame → trolley surface frame
        # Negating x and y positions is a 180° Z rotation; apply the same to orientation.
        # q_new = Rz(π) * q_original  →  (-qy, qx, qw, -qz)
        # pose.orientation.x = -ori['qy']
        # pose.orientation.y =  ori['qx']
        # pose.orientation.z =  ori['qw']
        # pose.orientation.w = -ori['qz']


        # axes mismatch
        pose.orientation.x = -ori['qx']   # negate for Y reflection
        pose.orientation.y =  ori['qy']   # unchanged
        pose.orientation.z = -ori['qz']   # negate for Y reflection
        pose.orientation.w =  ori['qw']   # unchanged

        # ── Dimensions (mm → m) ───────────────────────────────────────────────
        dims = det['dimensions']
        dx = mm_to_m(dims['dx_mm']/1.5)
        dy = mm_to_m(dims['dy_mm']/2)
        # dz_mm is optional — fall back to dy (long axis) if depth failed
        dz = mm_to_m(dims['dz_mm']/2) if 'dz_mm' in dims else dy

        # ── Tray boundary clamp ───────────────────────────────────────────────
        clamped = False
        if TRAY_X_MIN is not None and pose.position.x < TRAY_X_MIN:
            pose.position.x = TRAY_X_MIN; clamped = True
        if TRAY_X_MAX is not None and pose.position.x > TRAY_X_MAX:
            pose.position.x = TRAY_X_MAX; clamped = True
        if TRAY_Y_MIN is not None and pose.position.y < TRAY_Y_MIN:
            pose.position.y = TRAY_Y_MIN; clamped = True
        if TRAY_Y_MAX is not None and pose.position.y > TRAY_Y_MAX:
            pose.position.y = TRAY_Y_MAX; clamped = True
        if clamped:
            self.get_logger().warn(
                f'  {ml_class} clamped to tray bounds: '
                f'({pose.position.x:.3f}, {pose.position.y:.3f})'
            )

        # ── Build message ─────────────────────────────────────────────────────
        obj = Object()
        obj.header.frame_id  = ROBOT_FRAME
        obj.header.stamp     = stamp
        obj.pose             = pose
        obj.classification   = ml_class
        obj.dimensions       = [dx, dy, dz]

        self.get_logger().debug(
            f'  {ml_class} | '
            f'pos=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})m | '
            f'dims=[{dx:.3f}, {dy:.3f}, {dz:.3f}]m | '
            f'conf={confidence:.2f}'
        )

        return obj


# ==============================================================================
# Main
# ==============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = PlasticDetectionsTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
