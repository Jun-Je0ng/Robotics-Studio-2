# # # #!/usr/bin/env python3




# # # import rclpy
# # # from rclpy.node import Node
# # # from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
# # # from std_srvs.srv import Trigger
# # # import random

# # # from object_msgs.msg import Object, ObjectArray



# # # # ==============================================================================
# # # # Workspace & Bin Configuration — shared across all nodes, defined once
# # # # ==============================================================================

# # # UR3E_WORKSPACE = {
# # #     'x': (0.1, 0.4),
# # #     'y': (-0.3, 0.3),
# # #     'z': (0.05, 0.15)
# # # }

# # # # make workspace/pickup tray into negative X to better reflect real setup, but bins still in positive X in front of robot
# # # UR3E_WORKSPACE['x'] = (-UR3E_WORKSPACE['x'][1], -UR3E_WORKSPACE['x'][0])



# # # # 3 fixed drop bins — edit here and both nodes reflect the change
# # # BIN_POSES = [
# # #     {'x': 0.3, 'y': -0.3, 'z': 0.1},  # bin 0 — left
# # #     {'x': 0.3, 'y':  0.3, 'z': 0.1},  # bin 1 — right
# # #     {'x': 0.3, 'y':  0.0, 'z': 0.1},  # bin 2 — centre
# # # ]

# # # # Material classification -> bin index mapping
# # # BIN_MAP = {
# # #     'metal':   0,  # left bin
# # #     'plastic': 1,  # right bin
# # #     'fabric':  2,  # centre bin
# # # }

# # # Object_max_size = {
# # #     'x': 0.08,
# # #     'y': 0.08,
# # #     'z': 0.15,
# # # }

# # # CLASSIFICATION_OPTIONS = list(BIN_MAP.keys())

# # # MIN_OBJECT_SEPARATION = 0.08  # metres


# # # # ==============================================================================
# # # # Shared Utilities
# # # # ==============================================================================

# # # def make_pose(x, y, z) -> Pose:
# # #     p = Pose()
# # #     p.position = Point(x=x, y=y, z=z)
# # #     p.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
# # #     return p


# # # def make_bin_pose_array(stamp, frame_id='base_link') -> PoseArray:
# # #     """Build a PoseArray of all bin poses from the shared BIN_POSES config."""
# # #     array = PoseArray()
# # #     array.header.frame_id = frame_id
# # #     array.header.stamp = stamp
# # #     array.poses = [make_pose(b['x'], b['y'], b['z']) for b in BIN_POSES]
# # #     return array


# # # def make_random_pose(existing_poses=None, min_dist=MIN_OBJECT_SEPARATION, max_attempts=20) -> Pose:
# # #     """Generate a random pose within UR3e workspace, avoiding overlap with existing poses."""
# # #     existing_poses = existing_poses or []
# # #     for _ in range(max_attempts):
# # #         pose = make_pose(
# # #             x=random.uniform(*UR3E_WORKSPACE['x']),
# # #             y=random.uniform(*UR3E_WORKSPACE['y']),
# # #             z=random.uniform(*UR3E_WORKSPACE['z'])
# # #         )
# # #         if not _is_too_close(pose, existing_poses, min_dist):
# # #             return pose
# # #     return pose  # best effort after max attempts


# # # def make_random_geometry() -> tuple:
# # #     """
# # #     Simulate perception returning an oriented bounding box.
# # #     Dimensions are independent of classification — perception measures them directly.
# # #     Sorted ascending so thin_axis is always index 0.
# # #     Returns (dims_list, thin_axis_index).
# # #     """
# # #     dims = sorted([
# # #         random.uniform(0.03, 0.08),  # width
# # #         random.uniform(0.03, 0.08),  # depth
# # #         random.uniform(0.03, 0.15),  # height — wider range for tall objects
# # #     ])
# # #     return dims, 0  # thin_axis always 0 after sorting


# # # def build_object_msg(stamp, existing_poses, frame_id='base_link') -> Object:
# # #     """Build a single fully populated Object message."""
# # #     classification = random.choice(CLASSIFICATION_OPTIONS)
# # #     dims, thin_axis = make_random_geometry()
# # #     pose = make_random_pose(existing_poses)

# # #     obj = Object()
# # #     obj.header.frame_id = frame_id
# # #     obj.header.stamp = stamp
# # #     obj.pose = pose
# # #     obj.classification = classification
# # #     obj.dimensions = dims
# # #     obj.thin_axis = thin_axis
# # #     obj.bin_index = BIN_MAP[classification]
# # #     return obj


# # # def _is_too_close(new_pose, existing_poses, min_dist) -> bool:
# # #     for p in existing_poses:
# # #         dx = p.position.x - new_pose.position.x
# # #         dy = p.position.y - new_pose.position.y
# # #         if (dx**2 + dy**2) ** 0.5 < min_dist:
# # #             return True
# # #     return False


# # # # ==============================================================================
# # # # Phase 1 — Single Object
# # # # Trigger: ros2 service call /send_test_poses std_srvs/srv/Trigger {}
# # # # ==============================================================================

# # # class PickPlaceTestHelper(Node):
# # #     """
# # #     Phase 1: Single object pick and place test helper.
# # #     Publishes one random Object and the 3 bin goal poses.
# # #     """

# # #     def __init__(self):
# # #         super().__init__('pick_place_test_helper')

# # #         self.object_publisher = self.create_publisher(ObjectArray, 'perception/objects', 10)
# # #         self.goal_publisher   = self.create_publisher(PoseArray,   'perception/goal_poses', 10)

# # #         self.srv = self.create_service(Trigger, 'send_test_poses', self.send_test_poses_callback)

# # #         self.get_logger().info(
# # #             'Phase 1 test helper ready — single object. '
# # #             'Call /send_test_poses to trigger.'
# # #         )

# # #     def send_test_poses_callback(self, request, response):
# # #         stamp = self.get_clock().now().to_msg()

# # #         obj = build_object_msg(stamp, existing_poses=[])

# # #         obj_array = ObjectArray()
# # #         obj_array.header.frame_id = 'base_link'
# # #         obj_array.header.stamp = stamp
# # #         obj_array.objects = [obj]
# # #         self.object_publisher.publish(obj_array)

# # #         self.goal_publisher.publish(make_bin_pose_array(stamp))

# # #         self.get_logger().info(
# # #             f'[Phase 1] Published 1 object: '
# # #             f'classification={obj.classification}, bin={obj.bin_index}, '
# # #             f'dims={[round(d, 3) for d in obj.dimensions]}'
# # #         )

# # #         response.success = True
# # #         response.message = f'Published 1 object -> bin {obj.bin_index}'
# # #         return response


# # # # ==============================================================================
# # # # Phase 2 — Multiple Objects
# # # # Trigger: ros2 service call /send_test_poses std_srvs/srv/Trigger {}
# # # # ==============================================================================

# # # class ObjectClassificationHelper(Node):
# # #     """
# # #     Phase 2: Multiple object pick and place test helper.
# # #     Publishes N random Objects (non-overlapping) and the 3 bin goal poses.
# # #     """

# # #     def __init__(self, num_objects=3):
# # #         super().__init__('object_classification_helper')

# # #         self.object_publisher = self.create_publisher(ObjectArray, 'perception/objects', 10)
# # #         self.goal_publisher   = self.create_publisher(PoseArray,   'perception/goal_poses', 10)

# # #         self.srv = self.create_service(Trigger, 'send_test_poses', self.send_test_poses_callback)

# # #         self.number_of_objects = num_objects

# # #         self.get_logger().info(
# # #             f'Phase 2 test helper ready — {num_objects} objects, 3 bins. '
# # #             f'Call /send_test_poses to trigger.'
# # #         )

# # #     def send_test_poses_callback(self, request, response):
# # #         stamp = self.get_clock().now().to_msg()

# # #         # Build all objects, passing growing pose list to enforce separation
# # #         existing_poses = []
# # #         objects = []
# # #         for _ in range(self.number_of_objects):
# # #             obj = build_object_msg(stamp, existing_poses)
# # #             existing_poses.append(obj.pose)
# # #             objects.append(obj)

# # #         obj_array = ObjectArray()
# # #         obj_array.header.frame_id = 'base_link'
# # #         obj_array.header.stamp = stamp
# # #         obj_array.objects = objects  # assign whole list at once
# # #         self.object_publisher.publish(obj_array)

# # #         self.goal_publisher.publish(make_bin_pose_array(stamp))

# # #         summary = [(o.classification, o.bin_index) for o in objects]
# # #         self.get_logger().info(
# # #             f'[Phase 2] Published {self.number_of_objects} objects: '
# # #             f'{[(c, b) for c, b in summary]}'
# # #         )

# # #         response.success = True
# # #         response.message = f'Published {self.number_of_objects} objects across 3 bins'
# # #         return response


# # # # ==============================================================================
# # # # Main — swap which node to run here
# # # # ==============================================================================

# # # def main(args=None):
# # #     rclpy.init(args=args)

# # #     # --- Phase 1: single object ---
# # #     # node = PickPlaceTestHelper()

# # #     # --- Phase 2: multiple objects ---
# # #     node = ObjectClassificationHelper(num_objects=3)

# # #     rclpy.spin(node)
# # #     node.destroy_node()
# # #     rclpy.shutdown()


# # # if __name__ == '__main__':
# # #     main()
















# # import rclpy
# # from rclpy.node import Node
# # from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
# # from std_srvs.srv import Trigger
# # import random
# # import math

# # from object_msgs.msg import Object, ObjectArray


# # # ==============================================================================
# # # Workspace & Bin Configuration — shared across all nodes, defined once
# # # ==============================================================================

# # UR3E_WORKSPACE = {
# #     'x': (0.25, 0.45),   # directly in front of robot
# #     'y': (-0.15, 0.15),  # narrow — keeps wrist straight during pickup
# #     'z': (0.02,  0.05)
# # }


# # # Object size limits — all dimensions capped at MAX_OBJECT_SIZE
# # MAX_OBJECT_SIZE = 0.12  # some objects are too big, so side approach will be needed
# # MIN_OBJECT_SIZE = 0.02

# # # 3 bins to the right (+Y), staggered in X so the robot sweeps
# # # cleanly from front to side without retracing the pickup zone
# # BIN_POSES = [
# #     {'x':  0.35, 'y': 0.30, 'z': 0.05},  # bin 0 — metal   (front-right)
# #     {'x':  0.15, 'y': 0.30, 'z': 0.05},  # bin 1 — plastic  (mid-right)
# #     {'x': -0.05, 'y': 0.30, 'z': 0.05},  # bin 2 — fabric   (rear-right)
# # ]


# # # Material classification -> bin index mapping
# # BIN_MAP = {
# #     'metal':   0,  # left bin
# #     'plastic': 1,  # centre bin
# #     'fabric':  2,  # right bin
# # }

# # CLASSIFICATION_OPTIONS = list(BIN_MAP.keys())

# # MIN_OBJECT_SEPARATION = 0.12  # metres — increased to account for larger objects


# # # ==============================================================================
# # # Shared Utilities
# # # ==============================================================================

# # def random_quaternion():
    
# #     # random rotation about Z axis
# #     yaw = random.uniform(-math.pi, math.pi)
# #     return Quaternion(
# #         x=0.0,
# #         y=0.0,
# #         z=math.sin(yaw / 2.0),
# #         w=math.cos(yaw / 2.0)
# #     )

# # def make_pose(x, y, z) -> Pose:
# #     p = Pose()
# #     p.position = Point(x=x, y=y, z=z)
# #     p.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
# #     p.orientation = random_quaternion()
# #     return p


# # def make_bin_pose_array(stamp, frame_id='base_link') -> PoseArray:
# #     """Build a PoseArray of all bin poses from the shared BIN_POSES config."""
# #     array = PoseArray()
# #     array.header.frame_id = frame_id
# #     array.header.stamp = stamp
# #     array.poses = [make_pose(b['x'], b['y'], b['z']) for b in BIN_POSES]
# #     return array


# # def make_random_pose(existing_poses=None, min_dist=MIN_OBJECT_SEPARATION, max_attempts=20) -> Pose:
# #     """Generate a random pose within UR3e workspace, avoiding overlap with existing poses."""
# #     existing_poses = existing_poses or []
# #     for _ in range(max_attempts):
# #         pose = make_pose(
# #             x=random.uniform(*UR3E_WORKSPACE['x']),
# #             y=random.uniform(*UR3E_WORKSPACE['y']),
# #             z=random.uniform(*UR3E_WORKSPACE['z'])
# #         )
# #         if not _is_too_close(pose, existing_poses, min_dist):
# #             return pose
# #     return pose  # best effort after max attempts


# # def make_random_geometry() -> list:
# #     """
# #     Returns dimensions aligned with object frame: [dx, dy, dz]
# #     (NOT sorted)
# #     """
# #     return [
# #         random.uniform(MIN_OBJECT_SIZE, MAX_OBJECT_SIZE),
# #         random.uniform(MIN_OBJECT_SIZE, MAX_OBJECT_SIZE),
# #         random.uniform(MIN_OBJECT_SIZE, MAX_OBJECT_SIZE),
# #     ]


# # def build_object_msg(stamp, existing_poses, frame_id='base_link') -> Object:
# #     classification = random.choice(CLASSIFICATION_OPTIONS)
# #     dims = make_random_geometry()
# #     pose = make_random_pose(existing_poses)

# #     obj = Object()
# #     obj.header.frame_id = frame_id
# #     obj.header.stamp = stamp

# #     obj.pose = pose
# #     obj.classification = classification
# #     obj.dimensions = dims  # [dx, dy, dz] aligned with pose

# #     return obj


# # def _is_too_close(new_pose, existing_poses, min_dist) -> bool:
# #     for p in existing_poses:
# #         dx = p.position.x - new_pose.position.x
# #         dy = p.position.y - new_pose.position.y
# #         if (dx**2 + dy**2) ** 0.5 < min_dist:
# #             return True
# #     return False


# # # ==============================================================================
# # # Phase 1 — Single Object
# # # Trigger: ros2 service call /send_test_poses std_srvs/srv/Trigger {}
# # # ==============================================================================

# # class PickPlaceTestHelper(Node):
# #     """
# #     Phase 1: Single object pick and place test helper.
# #     Publishes one random Object and the 3 bin goal poses.
# #     """

# #     def __init__(self):
# #         super().__init__('pick_place_test_helper')

# #         self.object_publisher = self.create_publisher(ObjectArray, 'perception/objects', 10)
# #         self.goal_publisher   = self.create_publisher(PoseArray,   'perception/goal_poses', 10)

# #         self.srv = self.create_service(Trigger, 'send_test_poses', self.send_test_poses_callback)

# #         self.get_logger().info(
# #             'Phase 1 test helper ready — single object. '
# #             'Call /send_test_poses to trigger.'
# #         )

# #     def send_test_poses_callback(self, request, response):
# #         stamp = self.get_clock().now().to_msg()

# #         obj = build_object_msg(stamp, existing_poses=[])

# #         obj_array = ObjectArray()
# #         obj_array.header.frame_id = 'base_link'
# #         obj_array.header.stamp = stamp
# #         obj_array.objects = [obj]
# #         self.object_publisher.publish(obj_array)

# #         self.goal_publisher.publish(make_bin_pose_array(stamp))

# #         self.get_logger().info(
# #             f'[Phase 1] Published 1 object: '
# #             f'classification={obj.classification}, bin={obj.bin_index}, '
# #             f'dims={[round(d, 3) for d in obj.dimensions]}'
# #         )

# #         response.success = True
# #         response.message = f'Published 1 object -> bin {obj.bin_index}'
# #         return response


# # # ==============================================================================
# # # Phase 2 — Multiple Objects
# # # Trigger: ros2 service call /send_test_poses std_srvs/srv/Trigger {}
# # # ==============================================================================

# # class ObjectClassificationHelper(Node):
# #     """
# #     Phase 2: Multiple object pick and place test helper.
# #     Publishes N random Objects (non-overlapping) and the 3 bin goal poses.
# #     """

# #     def __init__(self, num_objects=3):
# #         super().__init__('object_classification_helper')

# #         self.object_publisher = self.create_publisher(ObjectArray, 'perception/objects', 10)
# #         self.goal_publisher   = self.create_publisher(PoseArray,   'perception/goal_poses', 10)

# #         self.srv = self.create_service(Trigger, 'send_test_poses', self.send_test_poses_callback)

# #         self.number_of_objects = num_objects

# #         self.get_logger().info(
# #             f'Phase 2 test helper ready — {num_objects} objects, 3 bins. '
# #             f'Bins behind robot, spread along Y. '
# #             f'Max object size: {MAX_OBJECT_SIZE*100:.0f}cm. '
# #             f'Call /send_test_poses to trigger.'
# #         )

# #     def send_test_poses_callback(self, request, response):
# #         stamp = self.get_clock().now().to_msg()

# #         existing_poses = []
# #         objects = []
# #         for _ in range(self.number_of_objects):
# #             obj = build_object_msg(stamp, existing_poses)
# #             existing_poses.append(obj.pose)
# #             objects.append(obj)

# #         obj_array = ObjectArray()
# #         obj_array.header.frame_id = 'base_link'
# #         obj_array.header.stamp = stamp
# #         obj_array.objects = objects
# #         self.object_publisher.publish(obj_array)

# #         self.goal_publisher.publish(make_bin_pose_array(stamp))

# #         summary = [(o.classification, o.bin_index, [round(d,3) for d in o.dimensions]) for o in objects]
# #         for c, b, d in summary:
# #             self.get_logger().info(f'  {c} -> bin {b} | dims {d}')

# #         response.success = True
# #         response.message = f'Published {self.number_of_objects} objects across 3 bins'
# #         return response


# # # ==============================================================================
# # # Main — swap which node to run here
# # # ==============================================================================

# # def main(args=None):
# #     rclpy.init(args=args)

# #     # --- Phase 1: single object ---
# #     # node = PickPlaceTestHelper()

# #     # --- Phase 2: multiple objects ---
# #     node = ObjectClassificationHelper(num_objects=3)

# #     rclpy.spin(node)
# #     node.destroy_node()
# #     rclpy.shutdown()


# # if __name__ == '__main__':
# #     main()



# # # new functions





# # # then in motion
# # # compute the thin axis
# # thin_axis = dims.index(min(dims))

# # # map the axis to direction/object frame
# # axes = [
# #     [1, 0, 0],  # X
# #     [0, 1, 0],  # Y
# #     [0, 0, 1],  # Z
# # ]

# # local_axis = axes[thin_axis]

# # # turn into world frame
# # world_axis = R_object * local_axis





#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
# from std_srvs.srv import Trigger
# import random
# import math

# from object_msgs.msg import Object, ObjectArray


# # ==============================================================================
# # Workspace & Bin Configuration
# # ==============================================================================

# UR3E_WORKSPACE = {
#     'x': (0.25, 0.45),
#     'y': (-0.15, 0.15),
#     'z': (0.02, 0.05)
# }

# MAX_OBJECT_SIZE = 0.12
# MIN_OBJECT_SIZE = 0.02

# BIN_POSES = [
#     {'x':  0.35, 'y': 0.30, 'z': 0.05},  # bin 0 — metal
#     {'x':  0.15, 'y': 0.30, 'z': 0.05},  # bin 1 — plastic
#     {'x': -0.05, 'y': 0.30, 'z': 0.05},  # bin 2 — fabric
# ]

# BIN_MAP = {
#     'metal':   0,
#     'plastic': 1,
#     'fabric':  2,
# }

# CLASSIFICATION_OPTIONS = list(BIN_MAP.keys())
# MIN_OBJECT_SEPARATION = 0.12


# # ==============================================================================
# # Utilities
# # ==============================================================================

# def random_quaternion() -> Quaternion:
#     yaw = random.uniform(-math.pi, math.pi)
#     return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


# def make_pose(x, y, z) -> Pose:
#     p = Pose()
#     p.position = Point(x=x, y=y, z=z)
#     p.orientation = random_quaternion()
#     return p


# def make_bin_pose_array(stamp, frame_id='base_link') -> PoseArray:
#     array = PoseArray()
#     array.header.frame_id = frame_id
#     array.header.stamp = stamp
#     array.poses = [make_pose(b['x'], b['y'], b['z']) for b in BIN_POSES]
#     return array


# def make_random_pose(existing_poses=None, min_dist=MIN_OBJECT_SEPARATION, max_attempts=20) -> Pose:
#     existing_poses = existing_poses or []
#     for _ in range(max_attempts):
#         pose = make_pose(
#             x=random.uniform(*UR3E_WORKSPACE['x']),
#             y=random.uniform(*UR3E_WORKSPACE['y']),
#             z=random.uniform(*UR3E_WORKSPACE['z'])
#         )
#         if not _is_too_close(pose, existing_poses, min_dist):
#             return pose
#     return pose


# def make_random_dimensions() -> list:
#     """Returns [dx, dy, dz] aligned to the object frame. Not sorted —
#     thin axis is intentionally left for the motion node to compute."""
#     return [
#         random.uniform(MIN_OBJECT_SIZE, MAX_OBJECT_SIZE),
#         random.uniform(MIN_OBJECT_SIZE, MAX_OBJECT_SIZE),
#         random.uniform(MIN_OBJECT_SIZE, MAX_OBJECT_SIZE),
#     ]


# def build_object_msg(stamp, existing_poses, frame_id='base_link') -> Object:
#     """
#     Builds an Object message containing only what perception would provide:
#       - header
#       - pose  (position + random yaw orientation)
#       - classification  (material type)
#       - dimensions  [dx, dy, dz] in object frame

#     bin_index and thin_axis are NOT set here — the motion node computes them.
#     """
#     obj = Object()
#     obj.header.frame_id = frame_id
#     obj.header.stamp = stamp
#     obj.pose = make_random_pose(existing_poses)
#     obj.classification = random.choice(CLASSIFICATION_OPTIONS)
#     obj.dimensions = make_random_dimensions()
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
# # ==============================================================================

# class PickPlaceTestHelper(Node):
#     def __init__(self):
#         super().__init__('pick_place_test_helper')
#         self.object_publisher = self.create_publisher(ObjectArray, 'perception/objects', 10)
#         self.goal_publisher   = self.create_publisher(PoseArray,   'perception/goal_poses', 10)
#         self.srv = self.create_service(Trigger, 'send_test_poses', self.callback)
#         self.get_logger().info('Phase 1 ready — call /send_test_poses')

#     def callback(self, request, response):
#         stamp = self.get_clock().now().to_msg()
#         obj = build_object_msg(stamp, existing_poses=[])

#         obj_array = ObjectArray()
#         obj_array.header.frame_id = 'base_link'
#         obj_array.header.stamp = stamp
#         obj_array.objects = [obj]
#         self.object_publisher.publish(obj_array)
#         self.goal_publisher.publish(make_bin_pose_array(stamp))

#         self.get_logger().info(
#             f'[Phase 1] class={obj.classification} '
#             f'dims={[round(d, 3) for d in obj.dimensions]}'
#         )
#         response.success = True
#         response.message = 'Published 1 object'
#         return response


# # ==============================================================================
# # Phase 2 — Multiple Objects
# # ==============================================================================

# class ObjectClassificationHelper(Node):
#     def __init__(self, num_objects=3):
#         super().__init__('object_classification_helper')
#         self.object_publisher = self.create_publisher(ObjectArray, 'perception/objects', 10)
#         self.goal_publisher   = self.create_publisher(PoseArray,   'perception/goal_poses', 10)
#         self.srv = self.create_service(Trigger, 'send_test_poses', self.callback)
#         self.number_of_objects = num_objects
#         self.get_logger().info(
#             f'Phase 2 ready — {num_objects} objects. Call /send_test_poses'
#         )

#     def callback(self, request, response):
#         stamp = self.get_clock().now().to_msg()

#         existing_poses = []
#         objects = []
#         for _ in range(self.number_of_objects):
#             obj = build_object_msg(stamp, existing_poses)
#             existing_poses.append(obj.pose)
#             objects.append(obj)

#         obj_array = ObjectArray()
#         obj_array.header.frame_id = 'base_link'
#         obj_array.header.stamp = stamp
#         obj_array.objects = objects
#         self.object_publisher.publish(obj_array)
#         self.goal_publisher.publish(make_bin_pose_array(stamp))

#         for o in objects:
#             self.get_logger().info(
#                 f'  class={o.classification} dims={[round(d, 3) for d in o.dimensions]}'
#             )

#         response.success = True
#         response.message = f'Published {self.number_of_objects} objects'
#         return response


# # ==============================================================================
# # Main
# # ==============================================================================

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectClassificationHelper(num_objects=3)
#     # node = PickPlaceTestHelper()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import String
from std_srvs.srv import Trigger
import random
import math
import json

from object_msgs.msg import Object, ObjectArray


# ==============================================================================
# Workspace & Bin Configuration
# ==============================================================================

UR3E_WORKSPACE = {
    'x': (0.25, 0.45),
    'y': (-0.15, 0.15),
    'z': (0.02, 0.05)
}

MAX_OBJECT_SIZE = 0.12
MIN_OBJECT_SIZE = 0.02

BIN_POSES = [
    {'x':  0.35, 'y': 0.30, 'z': 0.05},  # bin 0 — metal
    {'x':  0.15, 'y': 0.30, 'z': 0.05},  # bin 1 — plastic
    {'x': -0.05, 'y': 0.30, 'z': 0.05},  # bin 2 — fabric
]

BIN_MAP = {
    'metal':   0,
    'plastic': 1,
    'fabric':  2,
}

CLASSIFICATION_OPTIONS = list(BIN_MAP.keys())
MIN_OBJECT_SEPARATION = 0.12

# ==============================================================================
# Deterministic test mode
# Set to True to use fixed objects instead of random ones.
# Gives exactly 3 objects: top-down, side, and an awkward pose.
# ==============================================================================
DETERMINISTIC = True

DETERMINISTIC_OBJECTS = [
    {
        # Object 0 — TOP_DOWN candidate
        # Small flat box well within gripper span, low height, slight yaw
        'classification': 'metal',
        'x': -0.05, 'y': 0.30, 'z': 0.04,
        'yaw': math.radians(30),
        'dims': [0.06, 0.04, 0.03],   # thin axis = z (flat), min(x,y)=0.04 < 0.09
    },
    {
        # Object 1 — SIDE candidate
        # Wide flat object — both X and Y exceed top-down span limit
        'classification': 'plastic',
        'x': 0.10, 'y':  0.35, 'z': 0.03,
        'yaw': math.radians(-45),
        'dims': [0.11, 0.10, 0.025],  # thin axis = z, min(x,y)=0.10 > 0.09 → SIDE
    },
    {
        # Object 2 — UNCOMFORTABLE
        # Tall narrow object standing upright, thin axis is x,
        # height exceeds top-down wrist clearance limit
        'classification': 'fabric',
        'x': -0.10, 'y': 0.40, 'z': 0.10,
        'yaw': math.radians(80),
        'dims': [0.03, 0.055, 0.10],  # thin axis = x, height=0.10 > 0.08 → SIDE
    },
]


# ==============================================================================
# Utilities
# ==============================================================================

def make_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def random_quaternion() -> Quaternion:
    return make_quaternion(random.uniform(-math.pi, math.pi))


def make_pose(x, y, z, yaw=None) -> Pose:
    p = Pose()
    p.position = Point(x=x, y=y, z=z)
    p.orientation = make_quaternion(yaw) if yaw is not None else random_quaternion()
    return p


def make_bin_pose_array(stamp, frame_id='base_link') -> PoseArray:
    array = PoseArray()
    array.header.frame_id = frame_id
    array.header.stamp = stamp
    array.poses = [make_pose(b['x'], b['y'], b['z']) for b in BIN_POSES]
    return array


def make_random_pose(existing_dims=None, existing_poses=None, new_dims=None, max_attempts=50) -> Pose:
    """Generate a pose that doesn't cause the new object's footprint to overlap existing ones."""
    existing_poses = existing_poses or []
    existing_dims  = existing_dims  or []
    new_dims       = new_dims       or [MAX_OBJECT_SIZE, MAX_OBJECT_SIZE, MAX_OBJECT_SIZE]

    for _ in range(max_attempts):
        pose = make_pose(
            x=random.uniform(*UR3E_WORKSPACE['x']),
            y=random.uniform(*UR3E_WORKSPACE['y']),
            z=random.uniform(*UR3E_WORKSPACE['z'])
        )
        if not _overlaps(pose, new_dims, existing_poses, existing_dims):
            return pose

    return pose  # best effort


def make_random_dimensions() -> list:
    """Returns [dx, dy, dz] in object frame. Not sorted — thin axis computed by motion node."""
    return [
        random.uniform(MIN_OBJECT_SIZE, MAX_OBJECT_SIZE),
        random.uniform(MIN_OBJECT_SIZE, MAX_OBJECT_SIZE),
        random.uniform(MIN_OBJECT_SIZE, MAX_OBJECT_SIZE),
    ]


def build_object_msg(stamp, existing_poses, existing_dims, frame_id='base_link') -> Object:
    """
    Builds an Object message with only perception-provided fields:
      - header, pose, classification, dimensions
    bin_index and thin_axis are computed by the motion node.
    """
    dims = make_random_dimensions()
    pose = make_random_pose(existing_dims, existing_poses, dims)

    obj = Object()
    obj.header.frame_id = frame_id
    obj.header.stamp = stamp
    obj.pose = pose
    obj.classification = random.choice(CLASSIFICATION_OPTIONS)
    obj.dimensions = dims
    return obj


def build_deterministic_objects(stamp, frame_id='base_link') -> list:
    """Returns the fixed set of test objects covering top-down, side, and awkward cases."""
    objects = []
    for spec in DETERMINISTIC_OBJECTS:
        obj = Object()
        obj.header.frame_id = frame_id
        obj.header.stamp = stamp
        obj.pose = make_pose(spec['x'], spec['y'], spec['z'], yaw=spec['yaw'])
        obj.classification = spec['classification']
        obj.dimensions = spec['dims']
        objects.append(obj)
    return objects


def _overlaps(new_pose, new_dims, existing_poses, existing_dims) -> bool:
    """
    AABB overlap check in XY using actual object footprints.
    Uses the max of dx/dy as a conservative footprint radius per object.
    """
    new_r = max(new_dims[0], new_dims[1]) / 2.0
    for pose, dims in zip(existing_poses, existing_dims):
        existing_r = max(dims[0], dims[1]) / 2.0
        min_sep = new_r + existing_r + 0.02  # 2cm clearance gap
        dx = new_pose.position.x - pose.position.x
        dy = new_pose.position.y - pose.position.y
        if (dx**2 + dy**2) ** 0.5 < min_sep:
            return True
    return False


# ==============================================================================
# Phase 1 — Single Object
# ==============================================================================

class PickPlaceTestHelper(Node):
    def __init__(self):
        super().__init__('pick_place_test_helper')
        self.object_publisher = self.create_publisher(ObjectArray, 'perception/objects', 10)
        self.goal_publisher   = self.create_publisher(PoseArray,   'perception/goal_poses', 10)
        self.srv = self.create_service(Trigger, 'send_test_poses', self.callback)
        self.get_logger().info('Phase 1 ready — call /send_test_poses')

    def callback(self, request, response):
        stamp = self.get_clock().now().to_msg()
        obj = build_object_msg(stamp, existing_poses=[], existing_dims=[])

        obj_array = ObjectArray()
        obj_array.header.frame_id = 'base_link'
        obj_array.header.stamp = stamp
        obj_array.objects = [obj]
        self.object_publisher.publish(obj_array)
        self.goal_publisher.publish(make_bin_pose_array(stamp))

        self.get_logger().info(
            f'[Phase 1] class={obj.classification} '
            f'dims={[round(d, 3) for d in obj.dimensions]}'
        )
        response.success = True
        response.message = 'Published 1 object'
        return response


# ==============================================================================
# Phase 2 — Multiple Objects
# ==============================================================================

class ObjectClassificationHelper(Node):
    def __init__(self, num_objects=3):
        super().__init__('object_classification_helper')
        self.object_publisher = self.create_publisher(ObjectArray, 'perception/objects', 10)
        self.goal_publisher   = self.create_publisher(PoseArray,   'perception/goal_poses', 10)
        self.srv = self.create_service(Trigger, 'send_test_poses', self.callback)
        self.number_of_objects = num_objects
        self.get_logger().info(
            f'Phase 2 ready — {num_objects} objects. Call /send_test_poses'
        )

    def callback(self, request, response):
        stamp = self.get_clock().now().to_msg()

        if DETERMINISTIC:
            objects = build_deterministic_objects(stamp)
            self.get_logger().info('[Phase 2] DETERMINISTIC mode — fixed test objects')
        else:
            existing_poses = []
            existing_dims  = []
            objects = []
            for _ in range(self.number_of_objects):
                obj = build_object_msg(stamp, existing_poses, existing_dims)
                existing_poses.append(obj.pose)
                existing_dims.append(obj.dimensions)
                objects.append(obj)

        obj_array = ObjectArray()
        obj_array.header.frame_id = 'base_link'
        obj_array.header.stamp = stamp
        obj_array.objects = objects
        self.object_publisher.publish(obj_array)
        self.goal_publisher.publish(make_bin_pose_array(stamp))

        for o in objects:
            self.get_logger().info(
                f'  class={o.classification} dims={[round(d, 3) for d in o.dimensions]}'
            )

        response.success = True
        response.message = f'Published {len(objects)} objects'
        return response


# ==============================================================================
# Coordinate Inject Node
# Publishes a single detection with KNOWN teach-pendant coordinates onto
# /plastic_detections (raw JSON) so it flows through the translator unchanged.
# Use this to verify the translator fix: the object should appear at the
# correct physical location in RViz after spawning.
#
# Trigger:
#   ros2 service call /inject_detection std_srvs/srv/Trigger {}
#
# Edit INJECT_COORDS below to match what your teach pendant showed.
# ==============================================================================

# ── Object 1: flat/side-lying — expects TOP_DOWN grasp ───────────────────────
# Paste your teach-pendant reading here (mm).
# Strategy check: xy_min=80mm < 90mm AND height=60mm ≤ 80mm → TOP_DOWN
INJECT_COORDS = {
    'x_mm':  18.0,     # teach-pendant X
    'y_mm': -292.0,    # teach-pendant Y
    'z_mm': -133.0,    # teach-pendant Z (table surface)
}
INJECT_DIMS = {
    'dx_mm': 80.0,
    'dy_mm': 80.0,
    'dz_mm': 60.0,
}
INJECT_CLASS      = 'hdpe_bottle'
INJECT_CONFIDENCE = 0.95

# ── Object 2: tall upright bottle — expects SIDE_HORIZONTAL grasp ─────────────
# Placed 80 mm in X and 80 mm in Y away from object 1 so they don't overlap.
# Strategy check: xy_min=70mm < 90mm BUT height=250mm > 80mm → SIDE_HORIZONTAL
INJECT_TALL_COORDS = {
    'x_mm':  -200.0,     # offset from object 1
    'y_mm': -400.0,    # offset from object 1
    'z_mm': -70.0,    # same table height
}
INJECT_TALL_DIMS = {
    'dx_mm': 60.0,     # bottle diameter
    'dy_mm': 60.0,
    'dz_mm': 130.0,    # tall upright bottle — well above TOP_DOWN_MAX_HEIGHT (80 mm)
}
INJECT_TALL_CLASS      = 'hdpe_bottle'
INJECT_TALL_CONFIDENCE = 0.92
# ─────────────────────────────────────────────────────────────────────────────


class CoordinateInjectNode(Node):
    """
    Publishes a single detection with exact known coordinates onto
    /plastic_detections so the translator (and the full pipeline) processes it.

    After calling the service, look in RViz: the spawned collision object
    should appear at the physical location matching the teach pendant reading.
    """

    def __init__(self):
        super().__init__('coordinate_inject_node')

        self.pub = self.create_publisher(String, 'plastic_detections', 10)
        self.srv = self.create_service(Trigger, 'inject_detection', self.callback)

        self.get_logger().info(
            f'CoordinateInjectNode ready — injects 2 objects:\n'
            f'  [0] FLAT  ({INJECT_CLASS})  '
            f'pos=({INJECT_COORDS["x_mm"]}, {INJECT_COORDS["y_mm"]}, {INJECT_COORDS["z_mm"]}) mm  '
            f'dims=[{INJECT_DIMS["dx_mm"]}x{INJECT_DIMS["dy_mm"]}x{INJECT_DIMS["dz_mm"]}] mm  '
            f'→ expects TOP_DOWN\n'
            f'  [1] TALL  ({INJECT_TALL_CLASS})  '
            f'pos=({INJECT_TALL_COORDS["x_mm"]}, {INJECT_TALL_COORDS["y_mm"]}, {INJECT_TALL_COORDS["z_mm"]}) mm  '
            f'dims=[{INJECT_TALL_DIMS["dx_mm"]}x{INJECT_TALL_DIMS["dy_mm"]}x{INJECT_TALL_DIMS["dz_mm"]}] mm  '
            f'→ expects SIDE_HORIZONTAL\n'
            f'  Trigger: ros2 service call /inject_detection std_srvs/srv/Trigger {{}}'
        )

    @staticmethod
    def _make_detection(coords, dims, cls, confidence):
        """Build one entry for the /plastic_detections JSON array."""
        return {
            'pose': {
                'position': {
                    'x': coords['x_mm'],
                    'y': coords['y_mm'],
                    'z': coords['z_mm'],
                },
                'orientation': {'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            },
            'dimensions': {
                'dx_mm': dims['dx_mm'],
                'dy_mm': dims['dy_mm'],
                'dz_mm': dims['dz_mm'],
            },
            'classification': {
                'class':      cls,
                'confidence': confidence,
            },
            'debug': {
                'z_table_mm':    coords['z_mm'],
                'z_approach_mm': coords['z_mm'] + 150.0,
                'angle_deg':     0.0,
                'angle_rad':     0.0,
                'depth_m':       abs(coords['z_mm']) / 1000.0,
                'dz_source':     'depth',
            },
        }

    def callback(self, request, response):
        TCP_OFFSET_M = 0.218  # gripper TCP offset added by translator

        flat = self._make_detection(
            INJECT_COORDS, INJECT_DIMS, INJECT_CLASS, INJECT_CONFIDENCE)

        tall = self._make_detection(
            INJECT_TALL_COORDS, INJECT_TALL_DIMS,
            INJECT_TALL_CLASS, INJECT_TALL_CONFIDENCE)

        msg = String()
        # msg.data = json.dumps([flat, tall])
        msg.data = json.dumps([flat])
        self.pub.publish(msg)

        # Log expected positions after translator transform (Y flip + Z offset)
        for label, coords, dims in [
            ('FLAT (TOP_DOWN)',         INJECT_COORDS,      INJECT_DIMS),
            ('TALL (SIDE_HORIZONTAL)',  INJECT_TALL_COORDS, INJECT_TALL_DIMS),
        ]:
            ex = coords['x_mm'] / 1000.0
            ey = -coords['y_mm'] / 1000.0
            ez = coords['z_mm'] / 1000.0 + TCP_OFFSET_M
            self.get_logger().info(
                f'  {label}: inject ({coords["x_mm"]}, {coords["y_mm"]}, {coords["z_mm"]}) mm'
                f'  →  expect ({ex:.3f}, {ey:.3f}, {ez:.3f}) m  '
                f'dims [{dims["dx_mm"]:.0f}x{dims["dy_mm"]:.0f}x{dims["dz_mm"]:.0f}] mm'
            )

        response.success = True
        response.message = 'Two detections injected on /plastic_detections'
        return response


# ==============================================================================
# Main
# ==============================================================================

def main(args=None):
    rclpy.init(args=args)
    # ── Swap node here ──────────────────────────────────────────────────────
    node = CoordinateInjectNode()         # ← full-pipeline coordinate test
    # node = ObjectClassificationHelper(num_objects=3)
    # node = PickPlaceTestHelper()
    # ────────────────────────────────────────────────────────────────────────
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




# # /// helper for try 3
# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from std_srvs.srv import Trigger

# import random
# import math
# import json


# # ==============================================================================
# # Workspace Configuration  (all in metres internally, converted to mm on publish)
# # ==============================================================================

# UR3E_WORKSPACE = {
#     'x': (0.25, 0.45),   # metres
#     'y': (-0.15, 0.15),
#     'z': (0.02, 0.05)
# }

# MAX_OBJECT_DXY = 0.120   # max footprint dimension (metres)
# MIN_OBJECT_DXY = 0.030
# MAX_OBJECT_DZ  = 0.300   # max height — bottles can be tall
# MIN_OBJECT_DZ  = 0.060

# MIN_OBJECT_SEPARATION = 0.14   # metres, centre-to-centre

# CLASSIFICATION_OPTIONS = ['pet_bottle', 'hdpe_bottle']

# # RG2 physical limits (metres)
# RG2_MAX_SPAN             = 0.110
# RG2_FINGER_EXTENSION_MAX = 0.0392

# # Grasp strategy thresholds — mirror pick_place_demo.cpp constants
# TOP_DOWN_MAX_SPAN   = RG2_MAX_SPAN - 0.02   # 0.090 m
# TOP_DOWN_MAX_HEIGHT = 0.120                  # metres


# # ==============================================================================
# # Deterministic test objects
# # Set DETERMINISTIC = True to skip random generation and use these instead.
# # Covers: top-down small bottle, side-grasp wide bottle, tall upright bottle.
# # All positions in metres; converted to mm on publish.
# # ==============================================================================

# DETERMINISTIC = True

# DETERMINISTIC_OBJECTS = [
#     {
#         # Small squat PET bottle — top-down grasp
#         'classification': 'pet_bottle',
#         'confidence': 0.95,
#         'x': 0.30, 'y': 0.05, 'z': 0.03,
#         'yaw_deg': 30.0,
#         'dx': 0.065, 'dy': 0.065, 'dz': 0.120,   # short bottle, fits within top-down span
#     },
#     {
#         # Wide-body HDPE bottle — side grasp (dx > TOP_DOWN_MAX_SPAN)
#         'classification': 'hdpe_bottle',
#         'confidence': 0.88,
#         'x': 0.38, 'y': -0.08, 'z': 0.03,
#         'yaw_deg': -45.0,
#         'dx': 0.095, 'dy': 0.080, 'dz': 0.150,   # dx > 0.09 → SIDE
#     },
#     {
#         # Tall thin PET bottle standing upright — side grasp (height > TOP_DOWN_MAX_HEIGHT)
#         'classification': 'pet_bottle',
#         'confidence': 0.91,
#         'x': 0.28, 'y': -0.12, 'z': 0.03,
#         'yaw_deg': 80.0,
#         'dx': 0.055, 'dy': 0.055, 'dz': 0.250,   # tall → SIDE
#     },
# ]


# # ==============================================================================
# # Grasp geometry — mirrors pick_place_demo.cpp logic
# # ==============================================================================

# def compute_grasp(x_m, y_m, z_m, dx_m, dy_m, dz_m, yaw_rad):
#     """
#     Decide approach strategy and compute jaw opening + grip orientation.
#     Returns a dict matching the /grip_pose JSON schema.

#     Strategy rules (matching cpp constants):
#       TOP_DOWN  — min(dx, dy) <= 0.090 m  AND  dz <= 0.120 m
#       SIDE      — otherwise
#     """
#     xy_min = min(dx_m, dy_m)
#     xy_max = max(dx_m, dy_m)

#     if xy_min <= TOP_DOWN_MAX_SPAN and dz_m <= TOP_DOWN_MAX_HEIGHT:
#         approach = 'top_down'
#     else:
#         approach = 'side_down'

#     if approach == 'top_down':
#         # Jaw opens across the narrow XY dimension
#         jaw_m = min(xy_min - 0.005, RG2_MAX_SPAN)
#         jaw_m = max(jaw_m, 0.0)

#         # Finger angle: align fingers with long axis
#         if dx_m >= dy_m:
#             finger_angle = yaw_rad
#             jaw_dir = [math.cos(yaw_rad), math.sin(yaw_rad), 0.0]
#         else:
#             finger_angle = yaw_rad + math.pi / 2.0
#             jaw_dir = [-math.sin(yaw_rad), math.cos(yaw_rad), 0.0]

#         # Orientation: gripper Z down, rotated by finger_angle
#         qz = math.sin(finger_angle / 2.0)
#         # RPY = (pi, 0, finger_angle)  →  quaternion
#         # Full RPY→quaternion for (pi, 0, finger_angle):
#         #   roll=pi, pitch=0, yaw=finger_angle
#         cr, sr = math.cos(math.pi / 2), math.sin(math.pi / 2)   # roll/2
#         cp, sp = 1.0, 0.0                                         # pitch/2
#         cy = math.cos(finger_angle / 2)
#         sy = math.sin(finger_angle / 2)
#         qw =  cr * cp * cy + sr * sp * sy
#         qx =  sr * cp * cy - cr * sp * sy
#         qy =  cr * sp * cy + sr * cp * sy
#         qz_ = cr * cp * sy - sr * sp * cy

#         approach_vec = [0.0, 0.0, -1.0]
#         is_upright   = False

#     else:  # side_down
#         jaw_m = min(xy_min + 0.005, RG2_MAX_SPAN)
#         jaw_m = max(jaw_m, 0.0)

#         # Approach from side: RPY = (pi/2, 0, approach_angle)
#         thin_axis = 0 if dx_m <= dy_m else 1
#         approach_angle = yaw_rad if thin_axis != 0 else yaw_rad + math.pi / 2.0

#         cr, sr = math.cos(math.pi / 4), math.sin(math.pi / 4)   # roll = pi/2, half = pi/4
#         cp, sp = 1.0, 0.0
#         cy = math.cos(approach_angle / 2)
#         sy = math.sin(approach_angle / 2)
#         qw =  cr * cp * cy + sr * sp * sy
#         qx =  sr * cp * cy - cr * sp * sy
#         qy =  cr * sp * cy + sr * cp * sy
#         qz_ = cr * cp * sy - sr * sp * cy

#         jaw_dir      = [math.cos(approach_angle), math.sin(approach_angle), 0.0]
#         approach_vec = [0.0, 0.0, -1.0]
#         is_upright   = dz_m > TOP_DOWN_MAX_HEIGHT

#     return {
#         'jaw_opening_mm': jaw_m * 1000.0,
#         'approach':        approach,
#         'grip_orientation': {'qx': qx, 'qy': qy, 'qz': qz_, 'qw': qw},
#         'debug': {
#             'angle_deg':    math.degrees(yaw_rad),
#             'is_upright':   is_upright,
#             'long_axis_mm': xy_max * 1000.0,
#             'short_axis_mm': xy_min * 1000.0,
#             'jaw_dir':      jaw_dir,
#             'approach_vec': approach_vec,
#         }
#     }


# # ==============================================================================
# # Message builders
# # ==============================================================================

# def build_detection(x_m, y_m, z_m, dx_m, dy_m, dz_m,
#                     yaw_rad, classification, confidence,
#                     dz_source='depth'):
#     """
#     Build one entry for the /plastic_detections JSON array.
#     Converts metres → mm for all spatial values.
#     """
#     z_table_mm   = z_m * 1000.0               # simplified — treat floor as z_m
#     z_approach_mm = z_table_mm + 150.0         # hover 150 mm above table

#     detection = {
#         'pose': {
#             'position': {
#                 'x': x_m * 1000.0,
#                 'y': y_m * 1000.0,
#                 'z': z_m * 1000.0,
#             },
#             'orientation': {
#                 'qx': 0.0,
#                 'qy': 0.0,
#                 'qz': math.sin(yaw_rad / 2.0),
#                 'qw': math.cos(yaw_rad / 2.0),
#             }
#         },
#         'dimensions': {
#             'dx_mm': dx_m * 1000.0,
#             'dy_mm': dy_m * 1000.0,
#         },
#         'classification': {
#             'class':      classification,
#             'confidence': confidence,
#         },
#         'debug': {
#             'z_table_mm':    z_table_mm,
#             'z_approach_mm': z_approach_mm,
#             'angle_deg':     math.degrees(yaw_rad),
#             'angle_rad':     yaw_rad,
#             'depth_m':       z_m,
#             'dz_source':     dz_source,
#         }
#     }

#     # dz_mm is optional — only include if depth measurement succeeded
#     if dz_source != 'failed':
#         detection['dimensions']['dz_mm'] = dz_m * 1000.0

#     return detection


# def build_grip_pose(x_m, y_m, z_m, dx_m, dy_m, dz_m,
#                     yaw_rad, classification, confidence):
#     """
#     Build one entry for the /grip_pose JSON array.
#     grip_position is in mm to match the detection topic.
#     """
#     grasp = compute_grasp(x_m, y_m, z_m, dx_m, dy_m, dz_m, yaw_rad)

#     return {
#         'class':       classification,
#         'confidence':  confidence,
#         'grip_position': {
#             'x': x_m * 1000.0,
#             'y': y_m * 1000.0,
#             'z': z_m * 1000.0,
#         },
#         'grip_orientation': grasp['grip_orientation'],
#         'jaw_opening_mm':   grasp['jaw_opening_mm'],
#         'approach':         grasp['approach'],
#         'debug':            grasp['debug'],
#     }


# # ==============================================================================
# # Random object generation
# # ==============================================================================

# def random_yaw():
#     return random.uniform(-math.pi, math.pi)


# def make_random_object():
#     """Returns a dict with all fields in metres."""
#     return {
#         'x':  random.uniform(*UR3E_WORKSPACE['x']),
#         'y':  random.uniform(*UR3E_WORKSPACE['y']),
#         'z':  random.uniform(*UR3E_WORKSPACE['z']),
#         'dx': random.uniform(MIN_OBJECT_DXY, MAX_OBJECT_DXY),
#         'dy': random.uniform(MIN_OBJECT_DXY, MAX_OBJECT_DXY),
#         'dz': random.uniform(MIN_OBJECT_DZ,  MAX_OBJECT_DZ),
#         'yaw_rad':        random_yaw(),
#         'classification': random.choice(CLASSIFICATION_OPTIONS),
#         'confidence':     round(random.uniform(0.70, 0.99), 2),
#         'dz_source':      random.choice(['depth', 'depth', 'failed']),  # 2:1 success ratio
#     }


# def is_too_close(candidate, existing, min_sep=MIN_OBJECT_SEPARATION):
#     for e in existing:
#         dx = candidate['x'] - e['x']
#         dy = candidate['y'] - e['y']
#         if math.hypot(dx, dy) < min_sep:
#             return True
#     return False


# def make_random_objects(n, max_attempts=50):
#     placed = []
#     for _ in range(n):
#         for _ in range(max_attempts):
#             obj = make_random_object()
#             if not is_too_close(obj, placed):
#                 placed.append(obj)
#                 break
#         else:
#             placed.append(make_random_object())  # best effort
#     return placed


# def get_deterministic_objects():
#     """Convert DETERMINISTIC_OBJECTS spec (degrees, metres) to internal dicts."""
#     out = []
#     for spec in DETERMINISTIC_OBJECTS:
#         out.append({
#             'x':              spec['x'],
#             'y':              spec['y'],
#             'z':              spec['z'],
#             'dx':             spec['dx'],
#             'dy':             spec['dy'],
#             'dz':             spec['dz'],
#             'yaw_rad':        math.radians(spec['yaw_deg']),
#             'classification': spec['classification'],
#             'confidence':     spec.get('confidence', 0.90),
#             'dz_source':      'depth',
#         })
#     return out


# # ==============================================================================
# # Test helper node
# # ==============================================================================

# class PickPlaceTestHelper(Node):
#     """
#     Publishes /plastic_detections and /grip_pose as JSON strings.

#     Trigger:
#         ros2 service call /send_test_poses std_srvs/srv/Trigger {}

#     Topics published:
#         /plastic_detections  (std_msgs/String) — JSON array, positions in mm
#         /grip_pose           (std_msgs/String) — JSON array, positions in mm
#     """

#     def __init__(self, num_objects=3):
#         super().__init__('pick_place_test_helper')

#         self.num_objects = num_objects

#         self.detections_pub = self.create_publisher(
#             String, 'plastic_detections', 10)
#         self.grip_pub = self.create_publisher(
#             String, 'grip_pose', 10)

#         self.srv = self.create_service(
#             Trigger, 'send_test_poses', self.callback)

#         mode = 'DETERMINISTIC' if DETERMINISTIC else f'RANDOM ({num_objects} objects)'
#         self.get_logger().info(
#             f'Test helper ready [{mode}]. '
#             f'Call /send_test_poses to publish.'
#         )

#     def callback(self, request, response):
#         # ── Generate objects ──────────────────────────────────────────────────
#         if DETERMINISTIC:
#             objects = get_deterministic_objects()
#             self.get_logger().info('DETERMINISTIC mode — using fixed test objects')
#         else:
#             objects = make_random_objects(self.num_objects)

#         # ── Build both JSON arrays ────────────────────────────────────────────
#         detections_arr = []
#         grip_arr       = []

#         for obj in objects:
#             detections_arr.append(build_detection(
#                 x_m=obj['x'], y_m=obj['y'], z_m=obj['z'],
#                 dx_m=obj['dx'], dy_m=obj['dy'], dz_m=obj['dz'],
#                 yaw_rad=obj['yaw_rad'],
#                 classification=obj['classification'],
#                 confidence=obj['confidence'],
#                 dz_source=obj['dz_source'],
#             ))
#             grip_arr.append(build_grip_pose(
#                 x_m=obj['x'], y_m=obj['y'], z_m=obj['z'],
#                 dx_m=obj['dx'], dy_m=obj['dy'], dz_m=obj['dz'],
#                 yaw_rad=obj['yaw_rad'],
#                 classification=obj['classification'],
#                 confidence=obj['confidence'],
#             ))

#         # ── Publish ───────────────────────────────────────────────────────────
#         det_msg = String()
#         det_msg.data = json.dumps(detections_arr, indent=2)
#         self.detections_pub.publish(det_msg)

#         grip_msg = String()
#         grip_msg.data = json.dumps(grip_arr, indent=2)
#         self.grip_pub.publish(grip_msg)

#         # ── Log summary ───────────────────────────────────────────────────────
#         for i, (obj, g) in enumerate(zip(objects, grip_arr)):
#             self.get_logger().info(
#                 f'  [{i}] class={obj["classification"]} '
#                 f'conf={obj["confidence"]:.2f} '
#                 f'approach={g["approach"]} '
#                 f'jaw={g["jaw_opening_mm"]:.1f}mm '
#                 f'pos=({obj["x"]*1000:.0f}, {obj["y"]*1000:.0f}, {obj["z"]*1000:.0f})mm'
#             )

#         response.success = True
#         response.message = f'Published {len(objects)} objects on /plastic_detections and /grip_pose'
#         return response


# # ==============================================================================
# # Main
# # ==============================================================================

# def main(args=None):
#     rclpy.init(args=args)
#     node = PickPlaceTestHelper(num_objects=3)
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
