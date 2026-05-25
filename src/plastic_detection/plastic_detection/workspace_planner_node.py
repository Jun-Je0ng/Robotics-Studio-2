#!/usr/bin/env python3
"""
Workspace Planner Node
=======================
Subscribes to /plastic_detections
Uses K-Means clustering to decide which object to pick first.

Logic:
- Cluster detected objects by XY position
- Score each by isolation (mean distance to all others)
- Most isolated = pick_order 1 (easiest to grasp, least collision risk)
- Publishes prioritised list to /pick_queue
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler

class WorkspacePlannerNode(Node):
    def __init__(self):
        super().__init__('workspace_planner_node')

        self.subscription = self.create_subscription(
            String,
            'plastic_detections',
            self.detection_callback,
            10
        )
        self.publisher_ = self.create_publisher(
            String,
            'pick_queue',
            10
        )

        self.get_logger().info(
            'Workspace Planner Node ready — '
            'subscribing to /plastic_detections, '
            'publishing to /pick_queue'
        )

    def detection_callback(self, msg):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse detection message')
            return

        if not detections:
            out_msg = String()
            out_msg.data = json.dumps([])
            self.publisher_.publish(out_msg)
            return

        detections = json.loads(json.dumps(detections))

        if len(detections) == 1:
            detections[0]['cluster_id']      = 0
            detections[0]['isolation_score'] = 1.0
            detections[0]['pick_order']      = 1
        else:
            positions = np.array([
                [d['pose']['position']['x'], d['pose']['position']['y']]
                for d in detections
            ])

            n_clusters = min(len(detections), 3)
            scaler     = StandardScaler()
            pos_scaled = scaler.fit_transform(positions)
            labels     = KMeans(n_clusters=n_clusters, random_state=42, n_init=10).fit_predict(pos_scaled)

            # Isolation score = mean distance to all other objects
            isolation_scores = []
            for i, pos in enumerate(positions):
                distances = [np.linalg.norm(pos - positions[j])
                             for j in range(len(positions)) if j != i]
                isolation_scores.append(float(np.mean(distances)))

            # Normalise 0-1
            max_iso = max(isolation_scores) if max(isolation_scores) > 0 else 1.0
            isolation_scores = [s / max_iso for s in isolation_scores]

            for i, d in enumerate(detections):
                d['cluster_id']      = int(labels[i])
                d['isolation_score'] = round(isolation_scores[i], 3)

            # Sort by isolation descending — least isolated = biggest obj = pick first
            detections = sorted(detections, key=lambda d: d['isolation_score'], reverse=False)
            for i, d in enumerate(detections):
                d['pick_order'] = i + 1

        out_msg = String()
        out_msg.data = json.dumps(detections)
        self.publisher_.publish(out_msg)

        self.get_logger().info(f'Pick queue ({len(detections)} objects):')
        for d in detections:
            self.get_logger().info(
                f"  #{d['pick_order']} {d['classification']['class']} "
                f"| pos=({d['pose']['position']['x']:.0f}, {d['pose']['position']['y']:.0f}) mm "
                f"| isolation={d['isolation_score']:.2f} "
                f"| cluster={d['cluster_id']}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = WorkspacePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()