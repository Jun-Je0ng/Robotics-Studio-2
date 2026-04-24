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

MM_TO_M = 1e-3


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
        obj_array.header.frame_id = 'base_link'
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

        if not obj_array.objects:
            self.get_logger().warn('No valid objects after translation — not publishing')
            return

        self.pub.publish(obj_array)
        self.get_logger().info(
            f'Published {len(obj_array.objects)} object(s) on /perception/objects'
        )

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
        pose.position.x = mm_to_m(pos['x'])
        pose.position.y = mm_to_m(pos['y'])
        pose.position.z = mm_to_m(pos['z'])
        pose.orientation.x = ori['qx']
        pose.orientation.y = ori['qy']
        pose.orientation.z = ori['qz']
        pose.orientation.w = ori['qw']

        # ── Dimensions (mm → m) ───────────────────────────────────────────────
        dims = det['dimensions']
        dx = mm_to_m(dims['dx_mm'])
        dy = mm_to_m(dims['dy_mm'])
        # dz_mm is optional — fall back to dy (long axis) if depth failed
        dz = mm_to_m(dims['dz_mm']) if 'dz_mm' in dims else dy

        # ── Build message ─────────────────────────────────────────────────────
        obj = Object()
        obj.header.frame_id  = 'base_link'
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
