# #!/usr/bin/env python3




# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
# from std_srvs.srv import Trigger
# import random

# from object_msgs.msg import Object, ObjectArray



# # ==============================================================================
# # Workspace & Bin Configuration — shared across all nodes, defined once
# # ==============================================================================

# UR3E_WORKSPACE = {
#     'x': (0.1, 0.4),
#     'y': (-0.3, 0.3),
#     'z': (0.05, 0.15)
# }

# # make workspace/pickup tray into negative X to better reflect real setup, but bins still in positive X in front of robot
# UR3E_WORKSPACE['x'] = (-UR3E_WORKSPACE['x'][1], -UR3E_WORKSPACE['x'][0])



# # 3 fixed drop bins — edit here and both nodes reflect the change
# BIN_POSES = [
#     {'x': 0.3, 'y': -0.3, 'z': 0.1},  # bin 0 — left
#     {'x': 0.3, 'y':  0.3, 'z': 0.1},  # bin 1 — right
#     {'x': 0.3, 'y':  0.0, 'z': 0.1},  # bin 2 — centre
# ]

# # Material classification -> bin index mapping
# BIN_MAP = {
#     'metal':   0,  # left bin
#     'plastic': 1,  # right bin
#     'fabric':  2,  # centre bin
# }

# Object_max_size = {
#     'x': 0.08,
#     'y': 0.08,
#     'z': 0.15,
# }

# CLASSIFICATION_OPTIONS = list(BIN_MAP.keys())

# MIN_OBJECT_SEPARATION = 0.08  # metres


# # ==============================================================================
# # Shared Utilities
# # ==============================================================================

# def make_pose(x, y, z) -> Pose:
#     p = Pose()
#     p.position = Point(x=x, y=y, z=z)
#     p.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
#     return p


# def make_bin_pose_array(stamp, frame_id='base_link') -> PoseArray:
#     """Build a PoseArray of all bin poses from the shared BIN_POSES config."""
#     array = PoseArray()
#     array.header.frame_id = frame_id
#     array.header.stamp = stamp
#     array.poses = [make_pose(b['x'], b['y'], b['z']) for b in BIN_POSES]
#     return array


# def make_random_pose(existing_poses=None, min_dist=MIN_OBJECT_SEPARATION, max_attempts=20) -> Pose:
#     """Generate a random pose within UR3e workspace, avoiding overlap with existing poses."""
#     existing_poses = existing_poses or []
#     for _ in range(max_attempts):
#         pose = make_pose(
#             x=random.uniform(*UR3E_WORKSPACE['x']),
#             y=random.uniform(*UR3E_WORKSPACE['y']),
#             z=random.uniform(*UR3E_WORKSPACE['z'])
#         )
#         if not _is_too_close(pose, existing_poses, min_dist):
#             return pose
#     return pose  # best effort after max attempts


# def make_random_geometry() -> tuple:
#     """
#     Simulate perception returning an oriented bounding box.
#     Dimensions are independent of classification — perception measures them directly.
#     Sorted ascending so thin_axis is always index 0.
#     Returns (dims_list, thin_axis_index).
#     """
#     dims = sorted([
#         random.uniform(0.03, 0.08),  # width
#         random.uniform(0.03, 0.08),  # depth
#         random.uniform(0.03, 0.15),  # height — wider range for tall objects
#     ])
#     return dims, 0  # thin_axis always 0 after sorting


# def build_object_msg(stamp, existing_poses, frame_id='base_link') -> Object:
#     """Build a single fully populated Object message."""
#     classification = random.choice(CLASSIFICATION_OPTIONS)
#     dims, thin_axis = make_random_geometry()
#     pose = make_random_pose(existing_poses)

#     obj = Object()
#     obj.header.frame_id = frame_id
#     obj.header.stamp = stamp
#     obj.pose = pose
#     obj.classification = classification
#     obj.dimensions = dims
#     obj.thin_axis = thin_axis
#     obj.bin_index = BIN_MAP[classification]
#     return obj


# def _is_too_close(new_pose, existing_poses, min_dist) -> bool:
#     for p in existing_poses:
#         dx = p.position.x - new_pose.position.x
#         dy = p.position.y - new_pose.position.y
#         if (dx**2 + dy**2) ** 0.5 < min_dist:
#             return True
#     return False


# # ==============================================================================
# # Phase 1 — Single Object
# # Trigger: ros2 service call /send_test_poses std_srvs/srv/Trigger {}
# # ==============================================================================

# class PickPlaceTestHelper(Node):
#     """
#     Phase 1: Single object pick and place test helper.
#     Publishes one random Object and the 3 bin goal poses.
#     """

#     def __init__(self):
#         super().__init__('pick_place_test_helper')

#         self.object_publisher = self.create_publisher(ObjectArray, 'perception/objects', 10)
#         self.goal_publisher   = self.create_publisher(PoseArray,   'perception/goal_poses', 10)

#         self.srv = self.create_service(Trigger, 'send_test_poses', self.send_test_poses_callback)

#         self.get_logger().info(
#             'Phase 1 test helper ready — single object. '
#             'Call /send_test_poses to trigger.'
#         )

#     def send_test_poses_callback(self, request, response):
#         stamp = self.get_clock().now().to_msg()

#         obj = build_object_msg(stamp, existing_poses=[])

#         obj_array = ObjectArray()
#         obj_array.header.frame_id = 'base_link'
#         obj_array.header.stamp = stamp
#         obj_array.objects = [obj]
#         self.object_publisher.publish(obj_array)

#         self.goal_publisher.publish(make_bin_pose_array(stamp))

#         self.get_logger().info(
#             f'[Phase 1] Published 1 object: '
#             f'classification={obj.classification}, bin={obj.bin_index}, '
#             f'dims={[round(d, 3) for d in obj.dimensions]}'
#         )

#         response.success = True
#         response.message = f'Published 1 object -> bin {obj.bin_index}'
#         return response


# # ==============================================================================
# # Phase 2 — Multiple Objects
# # Trigger: ros2 service call /send_test_poses std_srvs/srv/Trigger {}
# # ==============================================================================

# class ObjectClassificationHelper(Node):
#     """
#     Phase 2: Multiple object pick and place test helper.
#     Publishes N random Objects (non-overlapping) and the 3 bin goal poses.
#     """

#     def __init__(self, num_objects=3):
#         super().__init__('object_classification_helper')

#         self.object_publisher = self.create_publisher(ObjectArray, 'perception/objects', 10)
#         self.goal_publisher   = self.create_publisher(PoseArray,   'perception/goal_poses', 10)

#         self.srv = self.create_service(Trigger, 'send_test_poses', self.send_test_poses_callback)

#         self.number_of_objects = num_objects

#         self.get_logger().info(
#             f'Phase 2 test helper ready — {num_objects} objects, 3 bins. '
#             f'Call /send_test_poses to trigger.'
#         )

#     def send_test_poses_callback(self, request, response):
#         stamp = self.get_clock().now().to_msg()

#         # Build all objects, passing growing pose list to enforce separation
#         existing_poses = []
#         objects = []
#         for _ in range(self.number_of_objects):
#             obj = build_object_msg(stamp, existing_poses)
#             existing_poses.append(obj.pose)
#             objects.append(obj)

#         obj_array = ObjectArray()
#         obj_array.header.frame_id = 'base_link'
#         obj_array.header.stamp = stamp
#         obj_array.objects = objects  # assign whole list at once
#         self.object_publisher.publish(obj_array)

#         self.goal_publisher.publish(make_bin_pose_array(stamp))

#         summary = [(o.classification, o.bin_index) for o in objects]
#         self.get_logger().info(
#             f'[Phase 2] Published {self.number_of_objects} objects: '
#             f'{[(c, b) for c, b in summary]}'
#         )

#         response.success = True
#         response.message = f'Published {self.number_of_objects} objects across 3 bins'
#         return response


# # ==============================================================================
# # Main — swap which node to run here
# # ==============================================================================

# def main(args=None):
#     rclpy.init(args=args)

#     # --- Phase 1: single object ---
#     # node = PickPlaceTestHelper()

#     # --- Phase 2: multiple objects ---
#     node = ObjectClassificationHelper(num_objects=3)

#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
















import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_srvs.srv import Trigger
import random

from object_msgs.msg import Object, ObjectArray


# ==============================================================================
# Workspace & Bin Configuration — shared across all nodes, defined once
# ==============================================================================

UR3E_WORKSPACE = {
    'x': (0.25, 0.45),   # directly in front of robot
    'y': (-0.15, 0.15),  # narrow — keeps wrist straight during pickup
    'z': (0.02,  0.05)
}


# Object size limits — all dimensions capped at MAX_OBJECT_SIZE
MAX_OBJECT_SIZE = 0.07  # slightly smaller — safer pregrasp clearance
MIN_OBJECT_SIZE = 0.02

# 3 bins to the right (+Y), staggered in X so the robot sweeps
# cleanly from front to side without retracing the pickup zone
BIN_POSES = [
    {'x':  0.35, 'y': 0.30, 'z': 0.05},  # bin 0 — metal   (front-right)
    {'x':  0.15, 'y': 0.30, 'z': 0.05},  # bin 1 — plastic  (mid-right)
    {'x': -0.05, 'y': 0.30, 'z': 0.05},  # bin 2 — fabric   (rear-right)
]


# Material classification -> bin index mapping
BIN_MAP = {
    'metal':   0,  # left bin
    'plastic': 1,  # centre bin
    'fabric':  2,  # right bin
}

CLASSIFICATION_OPTIONS = list(BIN_MAP.keys())

MIN_OBJECT_SEPARATION = 0.12  # metres — increased to account for larger objects


# ==============================================================================
# Shared Utilities
# ==============================================================================

def make_pose(x, y, z) -> Pose:
    p = Pose()
    p.position = Point(x=x, y=y, z=z)
    p.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    return p


def make_bin_pose_array(stamp, frame_id='base_link') -> PoseArray:
    """Build a PoseArray of all bin poses from the shared BIN_POSES config."""
    array = PoseArray()
    array.header.frame_id = frame_id
    array.header.stamp = stamp
    array.poses = [make_pose(b['x'], b['y'], b['z']) for b in BIN_POSES]
    return array


def make_random_pose(existing_poses=None, min_dist=MIN_OBJECT_SEPARATION, max_attempts=20) -> Pose:
    """Generate a random pose within UR3e workspace, avoiding overlap with existing poses."""
    existing_poses = existing_poses or []
    for _ in range(max_attempts):
        pose = make_pose(
            x=random.uniform(*UR3E_WORKSPACE['x']),
            y=random.uniform(*UR3E_WORKSPACE['y']),
            z=random.uniform(*UR3E_WORKSPACE['z'])
        )
        if not _is_too_close(pose, existing_poses, min_dist):
            return pose
    return pose  # best effort after max attempts


def make_random_geometry() -> tuple:
    """
    Simulate perception returning an oriented bounding box.
    Dimensions are independent of classification — perception measures them directly.
    All dimensions capped at MAX_OBJECT_SIZE.
    Sorted ascending so thin_axis is always index 0.
    Returns (dims_list, thin_axis_index).
    """
    dims = sorted([
        random.uniform(MIN_OBJECT_SIZE, MAX_OBJECT_SIZE),
        random.uniform(MIN_OBJECT_SIZE, MAX_OBJECT_SIZE),
        random.uniform(MIN_OBJECT_SIZE, MAX_OBJECT_SIZE),
    ])
    return dims, 0  # thin_axis always 0 after sorting


def build_object_msg(stamp, existing_poses, frame_id='base_link') -> Object:
    """Build a single fully populated Object message."""
    classification = random.choice(CLASSIFICATION_OPTIONS)
    dims, thin_axis = make_random_geometry()
    pose = make_random_pose(existing_poses)

    obj = Object()
    obj.header.frame_id = frame_id
    obj.header.stamp = stamp
    obj.pose = pose
    obj.classification = classification
    obj.dimensions = dims
    obj.thin_axis = thin_axis
    obj.bin_index = BIN_MAP[classification]
    return obj


def _is_too_close(new_pose, existing_poses, min_dist) -> bool:
    for p in existing_poses:
        dx = p.position.x - new_pose.position.x
        dy = p.position.y - new_pose.position.y
        if (dx**2 + dy**2) ** 0.5 < min_dist:
            return True
    return False


# ==============================================================================
# Phase 1 — Single Object
# Trigger: ros2 service call /send_test_poses std_srvs/srv/Trigger {}
# ==============================================================================

class PickPlaceTestHelper(Node):
    """
    Phase 1: Single object pick and place test helper.
    Publishes one random Object and the 3 bin goal poses.
    """

    def __init__(self):
        super().__init__('pick_place_test_helper')

        self.object_publisher = self.create_publisher(ObjectArray, 'perception/objects', 10)
        self.goal_publisher   = self.create_publisher(PoseArray,   'perception/goal_poses', 10)

        self.srv = self.create_service(Trigger, 'send_test_poses', self.send_test_poses_callback)

        self.get_logger().info(
            'Phase 1 test helper ready — single object. '
            'Call /send_test_poses to trigger.'
        )

    def send_test_poses_callback(self, request, response):
        stamp = self.get_clock().now().to_msg()

        obj = build_object_msg(stamp, existing_poses=[])

        obj_array = ObjectArray()
        obj_array.header.frame_id = 'base_link'
        obj_array.header.stamp = stamp
        obj_array.objects = [obj]
        self.object_publisher.publish(obj_array)

        self.goal_publisher.publish(make_bin_pose_array(stamp))

        self.get_logger().info(
            f'[Phase 1] Published 1 object: '
            f'classification={obj.classification}, bin={obj.bin_index}, '
            f'dims={[round(d, 3) for d in obj.dimensions]}'
        )

        response.success = True
        response.message = f'Published 1 object -> bin {obj.bin_index}'
        return response


# ==============================================================================
# Phase 2 — Multiple Objects
# Trigger: ros2 service call /send_test_poses std_srvs/srv/Trigger {}
# ==============================================================================

class ObjectClassificationHelper(Node):
    """
    Phase 2: Multiple object pick and place test helper.
    Publishes N random Objects (non-overlapping) and the 3 bin goal poses.
    """

    def __init__(self, num_objects=3):
        super().__init__('object_classification_helper')

        self.object_publisher = self.create_publisher(ObjectArray, 'perception/objects', 10)
        self.goal_publisher   = self.create_publisher(PoseArray,   'perception/goal_poses', 10)

        self.srv = self.create_service(Trigger, 'send_test_poses', self.send_test_poses_callback)

        self.number_of_objects = num_objects

        self.get_logger().info(
            f'Phase 2 test helper ready — {num_objects} objects, 3 bins. '
            f'Bins behind robot, spread along Y. '
            f'Max object size: {MAX_OBJECT_SIZE*100:.0f}cm. '
            f'Call /send_test_poses to trigger.'
        )

    def send_test_poses_callback(self, request, response):
        stamp = self.get_clock().now().to_msg()

        existing_poses = []
        objects = []
        for _ in range(self.number_of_objects):
            obj = build_object_msg(stamp, existing_poses)
            existing_poses.append(obj.pose)
            objects.append(obj)

        obj_array = ObjectArray()
        obj_array.header.frame_id = 'base_link'
        obj_array.header.stamp = stamp
        obj_array.objects = objects
        self.object_publisher.publish(obj_array)

        self.goal_publisher.publish(make_bin_pose_array(stamp))

        summary = [(o.classification, o.bin_index, [round(d,3) for d in o.dimensions]) for o in objects]
        for c, b, d in summary:
            self.get_logger().info(f'  {c} -> bin {b} | dims {d}')

        response.success = True
        response.message = f'Published {self.number_of_objects} objects across 3 bins'
        return response


# ==============================================================================
# Main — swap which node to run here
# ==============================================================================

def main(args=None):
    rclpy.init(args=args)

    # --- Phase 1: single object ---
    # node = PickPlaceTestHelper()

    # --- Phase 2: multiple objects ---
    node = ObjectClassificationHelper(num_objects=3)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

