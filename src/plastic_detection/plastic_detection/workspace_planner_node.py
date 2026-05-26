#!/usr/bin/env python3
"""
Workspace Planner Node
=======================
Subscribes to /plastic_detections
Uses K-Means clustering to group objects spatially.

Logic:
- Determine k by counting natural groups via pairwise distance threshold
  (objects within CLUSTER_DISTANCE_MM of each other → same group)
- Run K-Means on raw mm coordinates with that k
- Robot clears least isolated cluster first (tightest group), most isolated objects last
- Within each cluster, picks least-isolated objects first
- Publishes prioritised list to /pick_queue
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np

# Objects within this distance (mm) are considered the same natural group
CLUSTER_DISTANCE_MM = 200

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
            f'Workspace Planner Node ready — '
            f'cluster distance threshold: {CLUSTER_DISTANCE_MM}mm, '
            f'subscribing to /plastic_detections, '
            f'publishing to /pick_queue'
        )

    def _cluster_by_distance(self, positions: np.ndarray):
        """Assign cluster labels via connected components at CLUSTER_DISTANCE_MM.
        Objects within the threshold of each other are guaranteed the same label."""
        n = len(positions)
        labels = [-1] * n
        cid = 0
        for start in range(n):
            if labels[start] == -1:
                stack = [start]
                while stack:
                    node = stack.pop()
                    if labels[node] == -1:
                        labels[node] = cid
                        for nb in range(n):
                            if labels[nb] == -1 and \
                               np.linalg.norm(positions[node] - positions[nb]) < CLUSTER_DISTANCE_MM:
                                stack.append(nb)
                cid += 1
        return np.array(labels), cid

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

            labels, k = self._cluster_by_distance(positions)

            # Isolation score = normalised mean distance to all other objects
            isolation_scores = []
            for i, pos in enumerate(positions):
                distances = [np.linalg.norm(pos - positions[j])
                             for j in range(len(positions)) if j != i]
                isolation_scores.append(float(np.mean(distances)))

            max_iso = max(isolation_scores) if max(isolation_scores) > 0 else 1.0
            isolation_scores = [s / max_iso for s in isolation_scores]

            for i, d in enumerate(detections):
                d['cluster_id']      = int(labels[i])
                d['isolation_score'] = round(isolation_scores[i], 3)

            # Order clusters by mean isolation score ascending (least isolated cluster first)
            cluster_iso = {}
            for d in detections:
                cluster_iso.setdefault(d['cluster_id'], []).append(d['isolation_score'])
            cluster_order = sorted(
                cluster_iso.keys(),
                key=lambda cid: np.mean(cluster_iso[cid])
            )

            # Sort: least isolated cluster first, then within each cluster by
            # isolation_score ascending (most surrounded by other objects = first)
            detections = sorted(
                detections,
                key=lambda d: (
                    cluster_order.index(d['cluster_id']),
                    d['isolation_score']
                )
            )
            for i, d in enumerate(detections):
                d['pick_order'] = i + 1

            # Log cluster summary
            self.get_logger().info(
                f'Clusters: {k} (distance threshold {CLUSTER_DISTANCE_MM}mm)'
            )
            for cid in cluster_order:
                members = [d for d in detections if d['cluster_id'] == cid]
                names   = ', '.join(
                    d.get('instance_id', d['classification']['class']) for d in members
                )
                self.get_logger().info(
                    f'  Cluster {cid} ({len(members)} objects): [{names}]'
                )

        out_msg = String()
        out_msg.data = json.dumps(detections)
        self.publisher_.publish(out_msg)

        self.get_logger().info(f'Pick queue ({len(detections)} objects):')
        for d in detections:
            label = d.get('instance_id', d['classification']['class'])
            self.get_logger().info(
                f"  #{d['pick_order']} {label} "
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
