#!/usr/bin/env python3
"""
Workspace Planner Node (K-Means Version)
========================================
Subscribes to /plastic_detections
Uses K-Means clustering with smart k estimation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
from sklearn.cluster import KMeans

# Used to estimate the number of natural clusters
CLUSTER_DISTANCE_MM = 200


class WorkspacePlannerNode(Node):
    def __init__(self):
        super().__init__('workspace_planner_node')

        self.subscription = self.create_subscription(
            String, 'plastic_detections', self.detection_callback, 10
        )
        self.publisher_ = self.create_publisher(String, 'pick_queue', 10)

        self.get_logger().info(
            f'Workspace Planner Node (K-Means) ready — '
            f'cluster distance threshold: {CLUSTER_DISTANCE_MM}mm'
        )

    def _estimate_k(self, positions: np.ndarray) -> int:
        """Estimate number of clusters using connected components."""
        if len(positions) <= 1:
            return 1

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
        return cid

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

        # Extract positions in mm
        positions = np.array([
            [d['pose']['position']['x'], d['pose']['position']['y']]
            for d in detections
        ])

        n_objects = len(detections)

        if n_objects == 1:
            detections[0]['cluster_id'] = 0
            detections[0]['isolation_score'] = 1.0
            detections[0]['pick_order'] = 1
        else:
            # Step 1: Estimate good k using distance threshold
            k = self._estimate_k(positions)

            # Step 2: Run actual K-Means
            if k > 1:
                kmeans = KMeans(
                    n_clusters=k,
                    n_init=10,
                    random_state=42,
                    algorithm='lloyd'
                )
                cluster_labels = kmeans.fit_predict(positions)
            else:
                cluster_labels = np.zeros(n_objects, dtype=int)

            # Calculate isolation scores (independent of clustering method)
            isolation_scores = []
            for i, pos in enumerate(positions):
                distances = [np.linalg.norm(pos - positions[j]) for j in range(n_objects) if j != i]
                isolation_scores.append(float(np.mean(distances)))

            max_iso = max(isolation_scores) if max(isolation_scores) > 0 else 1.0
            isolation_scores = [s / max_iso for s in isolation_scores]

            # Attach data to detections
            for i, d in enumerate(detections):
                d['cluster_id'] = int(cluster_labels[i])
                d['isolation_score'] = round(isolation_scores[i], 3)

            # Order clusters by mean isolation (least isolated cluster first)
            cluster_iso = {}
            for d in detections:
                cluster_iso.setdefault(d['cluster_id'], []).append(d['isolation_score'])

            cluster_order = sorted(
                cluster_iso.keys(),
                key=lambda cid: np.mean(cluster_iso[cid])
            )

            # Final sort: cluster order first, then isolation within cluster
            detections = sorted(
                detections,
                key=lambda d: (
                    cluster_order.index(d['cluster_id']),
                    d['isolation_score']
                )
            )

            # Assign pick order
            for i, d in enumerate(detections):
                d['pick_order'] = i + 1

            # Logging
            self.get_logger().info(f'K-Means Clusters: {k} (est. via {CLUSTER_DISTANCE_MM}mm threshold)')
            for cid in cluster_order:
                members = [d for d in detections if d['cluster_id'] == cid]
                names = ', '.join(d.get('instance_id', d['classification']['class']) for d in members)
                self.get_logger().info(f'  Cluster {cid} ({len(members)} objects): [{names}]')

        # Publish
        out_msg = String()
        out_msg.data = json.dumps(detections)
        self.publisher_.publish(out_msg)

        # Summary log
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