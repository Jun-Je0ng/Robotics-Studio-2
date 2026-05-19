#     # this is will be a helper node for the subsystem demo. i.e. it can give random pose to pick up the object from and a random classifications for the object.
#     # possibly shape as well so that we can test the grasping strategy.


#     # will develop as class or functions as phases.

#     # phase 1: a model that sends poses to the main demo node, and the main demo node will execute the pick and place sequence based on those poses. 
#     # this will be used to test the basic functionality of the pick and place sequence without worrying about perception or classification.

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
# from std_srvs.srv import Trigger
# from std_msgs.msg import Int32MultiArray, String
# import json
# import random
# from moveit_msgs.msg import CollisionObject
# from shape_msgs.msg import SolidPrimitive



# Ur3e_workspace_limits = {
#     'x': (0.1, 0.4),  # metres, in front of robot
#     'y': (-0.3, 0.3),
#     'z': (0.01, 0.2)  # above table
# }


# # single pick and place unit test
# # assumption we: know object pose, dimensions and classification an goal pose
# # Trigger:
# # ros2 service call /send_test_poses std_srvs/srv/Trigger {}
# class PickPlaceTestHelper(Node):
#     # Simple Pick and Place Test Helper Node
#     def __init__(self):
#         super().__init__('pick_place_test_helper')

#         self.pose_publisher = self.create_publisher(PoseArray, 'perception/object_poses', 10)
#         self.goal_publisher = self.create_publisher(PoseArray, 'perception/goal_poses', 10)

#         # Service to trigger a test case on demand
#         self.srv = self.create_service(Trigger, 'send_test_poses', self.send_test_poses_callback)

#         self.get_logger().info('Test helper ready — call /send_test_poses to trigger')

#     def make_random_pose(self):
#         # Generate a pose within UR3e's reachable workspace
#         pose = Pose()
#         pose.position = Point(
#             x=random.uniform(0.1, 0.4), # metres, in front of robot
#             y=random.uniform(-0.3, 0.3),
#             z=random.uniform(0.01, 0.2) # above table
#         )
#         # Flat downward-facing orientation (w=1 = no rotation)
#         pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
#         return pose
    

#     # turn a pose and classification into a collision object for visualization in RViz
#     def make_random_geometry(self):
#         """
#         Simulate perception giving us an oriented bounding box.
#         Returns dimensions and a random thin-axis orientation.
#         """
#         # Random dimensions within plausible object size range for UR3e workspace
#         dims = [
#             random.uniform(0.03, 0.08),  # width
#             random.uniform(0.03, 0.08),  # depth
#             random.uniform(0.03, 0.15),  # height — wider range for tall objects
#         ]

#         # Thin axis is the smallest dimension's index
#         thin_axis_idx = dims.index(min(dims))

#         return {
#             'dimensions': dims,
#             'thin_axis': thin_axis_idx   # 0=x, 1=y, 2=z
#         }

#     def send_test_poses_callback(self, request, response):
#         # Publish a single random object pose
#         obj_array = PoseArray()
#         obj_array.header.frame_id = 'base_link'
#         obj_array.header.stamp = self.get_clock().now().to_msg()
#         obj_array.poses = [self.make_random_pose()]
#         self.pose_publisher.publish(obj_array)

#         # Publish a fixed goal pose (e.g. a drop bin)
#         goal_array = PoseArray()
#         goal_array.header.frame_id = 'base_link'
#         goal_array.header.stamp = self.get_clock().now().to_msg()

#         # GOal poses
#         goal_pose_1 = Pose()
#         goal_pose_1.position = Point(x=0.3, y=-0.3, z=0.1)
#         goal_pose_1.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

#         # goal_pose_2 = Pose()
#         # goal_pose_2.position = Point(x=0.3, y=0.3, z=0.1)
#         # goal_pose_2.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

#         goal_array.poses = [goal_pose_1]
#         self.goal_publisher.publish(goal_array)

#         response.success = True
#         response.message = 'Test poses published'
#         return response





# class ObjectClassificationHelper(Node):
#     """
#     Phase 2 test helper node for the UR3e pick and place system.
#     Publishes random object poses, classifications, fixed bin goal poses,
#     and an explicit classification-to-bin index mapping.
#     """

#     def __init__(self, num_objects=3):
#         super().__init__('object_classification_helper')

#         self.pose_publisher = self.create_publisher(PoseArray, 'perception/object_poses', 10)
#         self.goal_publisher = self.create_publisher(PoseArray, 'perception/goal_poses', 10)
#         self.classification_publisher = self.create_publisher(String, 'perception/object_classifications', 10)  # the possible classifications for each object, as a JSON list of strings, e.g. ['cube', 'sphere', 'cylinder']
#         self.bin_mapping_publisher = self.create_publisher(String, 'perception/bin_mapping', 10) # the classification of the objects mapped to bin index e.g. ['cube', 'sphere', 'cube'] -> [0, 2, 0]

#         self.srv = self.create_service(Trigger, 'send_test_poses', self.send_test_poses_callback)

#         self.number_of_objects = num_objects

#         # Source list of valid classification types — never overwrite this
#         self.classification_options = ['cube', 'cylinder', 'sphere']

#         # Fixed bin goal poses (3 bins regardless of number of objects)
#         self.bin_poses = self._init_bin_poses()

#         # Each classification maps to a fixed bin index
#         self.bin_map = {
#             'cube':     0,  # left bin
#             'cylinder': 1,  # right bin
#             'sphere':   2,  # centre bin
#         }

#         self.get_logger().info(
#             f'Object classification helper ready with {num_objects} objects and 3 bins. '
#             f'Call /send_test_poses to trigger.'
#         )

#     def _init_bin_poses(self):
#         """Define the 3 fixed drop bin poses."""
#         def make_pose(x, y, z):
#             p = Pose()
#             p.position = Point(x=x, y=y, z=z)
#             p.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
#             return p

#         return [
#             make_pose(0.3, -0.3, 0.1),  # bin 0 — left
#             make_pose(0.3,  0.3, 0.1),  # bin 1 — right
#             make_pose(0.3,  0.0, 0.1),  # bin 2 — centre
#         ]

#     # ------------------------------------------------------------------
#     # Random generation
#     # ------------------------------------------------------------------

#     def _make_single_pose(self, existing_poses, min_dist=0.08, max_attempts=20):
#         """Generate a pose within UR3e workspace, avoiding overlap with existing poses."""
#         for _ in range(max_attempts):
#             pose = Pose()
#             pose.position = Point(
#                 x=random.uniform(0.1, 0.4),
#                 y=random.uniform(-0.3, 0.3),
#                 z=random.uniform(0.01, 0.2)
#             )
#             pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

#             if not self._is_too_close(pose, existing_poses, min_dist):
#                 return pose

#         self.get_logger().warn('Could not find non-overlapping pose after max attempts, using last generated.')
#         return pose

#     def _is_too_close(self, new_pose, existing_poses, min_dist):
#         for p in existing_poses:
#             dx = p.position.x - new_pose.position.x
#             dy = p.position.y - new_pose.position.y
#             if (dx**2 + dy**2) ** 0.5 < min_dist:
#                 return True
#         return False

#     def _make_random_objects(self):
#         """Generate paired poses and classifications in a single pass."""
#         poses, classifications = [], []
#         for _ in range(self.number_of_objects):
#             poses.append(self._make_single_pose(poses))
#             classifications.append(random.choice(self.classification_options))
#         return poses, classifications



#     # Service callback

#     def send_test_poses_callback(self, request, response):
#         stamp = self.get_clock().now().to_msg()
#         poses, classifications = self._make_random_objects()

#         # 1. Object poses
#         obj_array = PoseArray()
#         obj_array.header.frame_id = 'base_link'
#         obj_array.header.stamp = stamp
#         obj_array.poses = poses
#         self.pose_publisher.publish(obj_array)

#         # 2. Classifications (JSON list, paired by index with object poses)
#         classification_msg = String()
#         classification_msg.data = json.dumps(classifications)
#         self.classification_publisher.publish(classification_msg)

#         # 3. Fixed bin goal poses (always 3)
#         goal_array = PoseArray()
#         goal_array.header.frame_id = 'base_link'
#         goal_array.header.stamp = stamp
#         goal_array.poses = self.bin_poses
#         self.goal_publisher.publish(goal_array)

#         # 4. Bin mapping — tells the planner which bin each object goes to
#         # e.g. ['cube', 'sphere', 'cube'] -> [0, 2, 0]
#         bin_indices = [self.bin_map[c] for c in classifications]
#         bin_mapping_msg = String()
#         bin_mapping_msg.data = json.dumps(bin_indices)
#         self.bin_mapping_publisher.publish(bin_mapping_msg)

#         self.get_logger().info(
#             f'Published {self.number_of_objects} objects: {classifications} -> bins {bin_indices}'
#         )

#         response.success = True
#         response.message = f'Published {self.number_of_objects} objects across 3 bins'
#         return response


# # ------------------------------------------------------------------

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectClassificationHelper(num_objects=3)
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()





# custom data type:
# Single detected object
std_msgs/Header header

geometry_msgs/Pose pose          # position and orientation in frame
string classification            # material type e.g. 'metal', 'plastic'

# Bounding box geometry from perception
float64[3] dimensions            # [width, depth, height] in metres
uint8 thin_axis                  # index of thinnest dimension: 0=x, 1=y, 2=z

# Array of detected objects
std_msgs/Header header
Object[] objects


# CMAKE:
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Object.msg"
  "msg/ObjectArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# Package.xml:
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
<depend>geometry_msgs</depend>
<depend>std_msgs</depend>


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_srvs.srv import Trigger
from std_msgs.msg import String
import json
import random


# ==============================================================================
# Workspace & Bin Configuration — shared across all nodes, defined once
# ==============================================================================

UR3E_WORKSPACE = {
    'x': (0.1, 0.4),
    'y': (-0.3, 0.3),
    'z': (0.01, 0.2)
}

# 3 fixed drop bins — edit here and both nodes reflect the change
BIN_POSES = [
    {'x': 0.3, 'y': -0.3, 'z': 0.1},  # bin 0 — left
    {'x': 0.3, 'y':  0.3, 'z': 0.1},  # bin 1 — right
    {'x': 0.3, 'y':  0.0, 'z': 0.1},  # bin 2 — centre
]

# Material classification -> bin index mapping
BIN_MAP = {
    'metal':   0,  # left bin
    'plastic': 1,  # right bin
    'fabric':  2,  # centre bin
}

CLASSIFICATION_OPTIONS = list(BIN_MAP.keys())

MIN_OBJECT_SEPARATION = 0.08  # metres


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


def make_random_geometry() -> dict:
    """
    Simulate perception returning an oriented bounding box.
    Dimensions are independent of classification — perception measures them directly.
    Returns dimensions sorted ascending so thin_axis is always index 0.
    """
    dims = sorted([
        random.uniform(0.03, 0.08),  # width
        random.uniform(0.03, 0.08),  # depth
        random.uniform(0.03, 0.15),  # height — wider range for tall objects
    ])
    return {
        'dimensions': dims,   # [thin, mid, thick] in metres
        'thin_axis': 0        # index of thinnest dimension after sorting: 0=thinnest
    }


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
    Publishes one random object (pose, classification, geometry) and the 3 bin goal poses.
    The planner receives everything it needs to execute one full pick and place cycle.
    """

    def __init__(self):
        super().__init__('pick_place_test_helper')

        self.object_publisher = self.create_publisher(String,    'perception/objects', 10)
        self.goal_publisher   = self.create_publisher(PoseArray, 'perception/goal_poses', 10)

        self.srv = self.create_service(Trigger, 'send_test_poses', self.send_test_poses_callback)

        self.get_logger().info(
            'Phase 1 test helper ready — single object. '
            'Call /send_test_poses to trigger.'
        )

    def send_test_poses_callback(self, request, response):
        stamp = self.get_clock().now().to_msg()

        # Build single object dict (mirrors what real perception would publish)
        classification = random.choice(CLASSIFICATION_OPTIONS)
        obj = {
            'pose': {
                'x': random.uniform(*UR3E_WORKSPACE['x']),
                'y': random.uniform(*UR3E_WORKSPACE['y']),
                'z': random.uniform(*UR3E_WORKSPACE['z']),
            },
            'classification': classification,
            'geometry': make_random_geometry(),
            'bin_index': BIN_MAP[classification]
        }

        # Publish as JSON — replace with custom msg once your_msgs is set up
        obj_msg = String()
        obj_msg.data = json.dumps([obj])  # list for consistency with Phase 2
        self.object_publisher.publish(obj_msg)

        # Publish all 3 bin goal poses
        self.goal_publisher.publish(make_bin_pose_array(stamp))

        self.get_logger().info(
            f'[Phase 1] Published 1 object: '
            f'classification={classification}, bin={obj["bin_index"]}'
        )

        response.success = True
        response.message = f'Published 1 object -> bin {obj["bin_index"]}'
        return response


# ==============================================================================
# Phase 2 — Multiple Objects
# Trigger: ros2 service call /send_test_poses std_srvs/srv/Trigger {}
# ==============================================================================

class ObjectClassificationHelper(Node):
    """
    Phase 2: Multiple object pick and place test helper.
    Publishes N random objects (pose, classification, geometry, bin assignment)
    and the 3 bin goal poses. Objects are guaranteed non-overlapping.
    """

    def __init__(self, num_objects=3):
        super().__init__('object_classification_helper')

        self.object_publisher = self.create_publisher(String,    'perception/objects', 10)
        self.goal_publisher   = self.create_publisher(PoseArray, 'perception/goal_poses', 10)

        self.srv = self.create_service(Trigger, 'send_test_poses', self.send_test_poses_callback)

        self.number_of_objects = num_objects

        self.get_logger().info(
            f'Phase 2 test helper ready — {num_objects} objects, 3 bins. '
            f'Call /send_test_poses to trigger.'
        )

    def _make_all_objects(self) -> list:
        """
        Generate N non-overlapping objects, each with:
          - pose         : random position in UR3e workspace
          - classification: random material type
          - geometry     : random bounding box (independent of classification)
          - bin_index    : derived from classification via BIN_MAP
        """
        existing_poses = []
        objects = []

        for _ in range(self.number_of_objects):
            pose = make_random_pose(existing_poses)
            existing_poses.append(pose)

            classification = random.choice(CLASSIFICATION_OPTIONS)

            objects.append({
                'pose': {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z,
                },
                'classification': classification,
                'geometry': make_random_geometry(),
                'bin_index': BIN_MAP[classification]
            })

        return objects

    def send_test_poses_callback(self, request, response):
        stamp = self.get_clock().now().to_msg()
        objects = self._make_all_objects()

        # Publish full object array as JSON — replace with ObjectArray msg once your_msgs is set up
        obj_msg = String()
        obj_msg.data = json.dumps(objects)
        self.object_publisher.publish(obj_msg)

        # Publish all 3 bin goal poses
        self.goal_publisher.publish(make_bin_pose_array(stamp))

        summary = [(o['classification'], o['bin_index']) for o in objects]
        self.get_logger().info(
            f'[Phase 2] Published {self.number_of_objects} objects: '
            f'{[(c, b) for c, b in summary]}'
        )

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