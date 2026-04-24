// // Pick and place unit test for the UR3e demo
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// #include <geometry_msgs/msg/pose.hpp>
// #include <geometry_msgs/msg/pose_array.hpp>
// #include <std_msgs/msg/float64_multi_array.hpp>
// #include <std_srvs/srv/trigger.hpp>

// #include <object_msgs/msg/object_array.hpp>
// #include <object_msgs/msg/object.hpp>

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include <thread>
// #include <mutex>
// #include <atomic>
// #include <algorithm>

// // Force planner to minimize joint-space distance, not just find any valid path
// #include <moveit/planning_interface/planning_interface.h>
// #include <moveit/kinematic_constraints/utils.h>

// namespace rvt = rviz_visual_tools;

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_demo");

// const std::string ARM_GROUP     = "ur_onrobot_manipulator";
// const std::string GRIPPER_GROUP = "ur_onrobot_gripper";
// const double PREGRASP_HEIGHT    = 0.05;   // metres above object
// const double GRIPPER_OPEN       = 0.085;
// const double GRIPPER_CLOSED     = 0.01;


// // ============================================================
// // Grasp Orientation
// // Computes a gripper quaternion aligned to the object's thin axis.
// // The gripper always points down (Z down), rotated about Z to align
// // with the thinnest dimension of the object.
// // thin_axis: 0=x, 1=y, 2=z (index of thinnest dimension)
// // ============================================================


// // Graps aliong the thin axis of the object so fingers slip along narrow sides rather than spanning wide face
// geometry_msgs::msg::Quaternion computeGraspOrientation(uint8_t thin_axis)
// {
//   tf2::Quaternion q;

//   // RG2 fingers are thin — align them WITH the thin axis of the object
//   // so the fingers slip along the narrow sides rather than spanning the wide face
//   switch (thin_axis)
//   {
//     case 0:
//       // Thin along X — align fingers with X, no rotation needed
//       q.setRPY(M_PI, 0.0, 0.0);
//       break;
//     case 1:
//       // Thin along Y — align fingers with Y, rotate 90deg about Z
//       q.setRPY(M_PI, 0.0, M_PI / 2.0);
//       break;
//     case 2:
//     default:
//       // Thin along Z (flat object) — grasp straight down, finger alignment
//       // doesn't matter much so default orientation is fine
//       q.setRPY(M_PI, 0.0, 0.0);
//       break;
//   }

//   q.normalize();
//   return tf2::toMsg(q);
// }


// double computeGraspWidth(const object_msgs::msg::Object& obj)
// {
//   // Find the two non-thin dimensions and take the smaller one
//   std::vector<double> dims = {obj.dimensions[0], obj.dimensions[1], obj.dimensions[2]};
//   dims.erase(dims.begin() + obj.thin_axis);  // remove the thin axis
//   return std::clamp(*std::min_element(dims.begin(), dims.end()) + 0.005, 0.0, 0.110);
// }


// // ============================================================
// // Gripper Control
// // ============================================================

// void sendGripper(
//     const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& pub,
//     double width)
// {
//   width = std::clamp(width, 0.0, 0.11);
//   std_msgs::msg::Float64MultiArray msg;
//   msg.data = {width};
//   pub->publish(msg);
//   RCLCPP_INFO(LOGGER, "Gripper → %.3f", width);
//   std::this_thread::sleep_for(std::chrono::milliseconds(500));
// }


// // ============================================================
// // Collision Object Helpers
// // ============================================================

// // Add a collision object representing the object in the scene (before picking)
// void addObjectCollision(
//     moveit::planning_interface::PlanningSceneInterface& psi,
//     const object_msgs::msg::Object& obj,
//     const std::string& id)
// {
//   moveit_msgs::msg::CollisionObject co;
//   co.header.frame_id = "base_link";
//   co.id = id;

//   shape_msgs::msg::SolidPrimitive box;
//   box.type = box.BOX;
//   box.dimensions = {obj.dimensions[0], obj.dimensions[1], obj.dimensions[2]};

//   co.primitives.push_back(box);
//   co.primitive_poses.push_back(obj.pose);
//   co.operation = co.ADD;

//   psi.addCollisionObjects({co});
//   RCLCPP_INFO(LOGGER, "Added collision object: %s", id.c_str());
// }


// void attachObject(
//   moveit::planning_interface::MoveGroupInterface& arm,
//   const object_msgs::msg::Object& obj,
//   const std::string& id)
// {
//   // Shrink the attached box slightly on the gripper closing axis (Y)
//   // so MoveIt doesn't see it as colliding with the closed fingers
//   const double GRASP_MARGIN = 0.01;  // 1cm shrink per side

//   moveit_msgs::msg::CollisionObject co;
//   co.header.frame_id = "tool0";
//   co.id = id;

//   shape_msgs::msg::SolidPrimitive box;
//   box.type = box.BOX;
//   box.dimensions = {
//     obj.dimensions[0],
//     obj.dimensions[1] - 2.0 * GRASP_MARGIN,  // shrink on closing axis
//     obj.dimensions[2]
//   };

//   geometry_msgs::msg::Pose local_pose;
//   local_pose.position.z    = obj.dimensions[2] / 2.0;
//   local_pose.orientation.w = 1.0;

//   co.primitives.push_back(box);
//   co.primitive_poses.push_back(local_pose);
//   co.operation = moveit_msgs::msg::CollisionObject::ADD;

//   std::vector<std::string> touch_links = {
//     "tool0",
//     "onrobot_rg2_base_link",
//     "onrobot_rg2_left_outer_knuckle",
//     "onrobot_rg2_left_inner_knuckle",
//     "onrobot_rg2_left_inner_finger",
//     "onrobot_rg2_left_finger_tip",
//     "onrobot_rg2_right_outer_knuckle",
//     "onrobot_rg2_right_inner_knuckle",
//     "onrobot_rg2_right_inner_finger",
//     "onrobot_rg2_right_finger_tip",
//     "onrobot_rg2_finger_width_mock_link",
//     "onrobot_rg2_gripper_tcp"
//   };

//   arm.attachObject(id, "tool0", touch_links);
//   RCLCPP_INFO(LOGGER, "Attached object: %s (dims shrunk on grasp axis)", id.c_str());
// }

// // Detach and remove collision object after placing
// void detachAndRemoveObject(
//     moveit::planning_interface::MoveGroupInterface& arm,
//     moveit::planning_interface::PlanningSceneInterface& psi,
//     const std::string& id)
// {
//   arm.detachObject(id);
//   psi.removeCollisionObjects({id});
//   RCLCPP_INFO(LOGGER, "Detached and removed: %s", id.c_str());
// }

// bool liftObject(
//   moveit::planning_interface::MoveGroupInterface& arm,
//   const geometry_msgs::msg::Pose& pregrasp_pose)
// {
//   // Use joint-space planning for lift — Cartesian fails with attached collision objects
//   arm.setPoseTarget(pregrasp_pose);
//   moveit::planning_interface::MoveGroupInterface::Plan plan;
//   if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
//   {
//     RCLCPP_ERROR(LOGGER, "Lift planning failed");
//     return false;
//   }
//   return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
// }

// // ============================================================
// // Scene Setup
// // ============================================================

// void placeGround(
//     moveit::planning_interface::MoveGroupInterface& arm,
//     moveit::planning_interface::PlanningSceneInterface& psi)
// {
//   moveit_msgs::msg::CollisionObject ground;
//   ground.header.frame_id = arm.getPlanningFrame();
//   ground.id = "ground";

//   shape_msgs::msg::SolidPrimitive primitive;
//   primitive.type       = primitive.BOX;
//   primitive.dimensions = {2.0, 2.0, 0.1};

//   geometry_msgs::msg::Pose pose;
//   pose.orientation.w = 1.0;
//   pose.position.z    = -0.075;

//   ground.primitives.push_back(primitive);
//   ground.primitive_poses.push_back(pose);
//   ground.operation = ground.ADD;

//   psi.addCollisionObjects({ground});
// }

// void returnHome(
//     moveit::planning_interface::MoveGroupInterface& arm,
//     const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub)
// {
//   arm.setNamedTarget("home");
//   moveit::planning_interface::MoveGroupInterface::Plan plan;
//   if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
//     arm.execute(plan);
//   sendGripper(gripper_pub, GRIPPER_OPEN);
// }


// // ============================================================
// // Arm Motion Helpers
// // ============================================================

// bool moveTopose(
//     moveit::planning_interface::MoveGroupInterface& arm,
//     const geometry_msgs::msg::Pose& pose)
// {
//   arm.setPoseTarget(pose);
//   moveit::planning_interface::MoveGroupInterface::Plan plan;
//   if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
//   {
//     RCLCPP_ERROR(LOGGER, "Planning failed");
//     return false;
//   }
//   return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
// }

// bool moveCartesianDown(
//     moveit::planning_interface::MoveGroupInterface& arm,
//     const geometry_msgs::msg::Pose& target)
// {
//   std::vector<geometry_msgs::msg::Pose> waypoints{target};
//   moveit_msgs::msg::RobotTrajectory traj;
//   double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, traj);

//   if (fraction < 0.9)
//   {
//     RCLCPP_ERROR(LOGGER, "Cartesian path only %.0f%% complete", fraction * 100.0);
//     return false;
//   }

//   moveit::planning_interface::MoveGroupInterface::Plan plan;
//   plan.trajectory_ = traj;
//   return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
// }


// // ============================================================
// // Pick and Place Sequence
// // ============================================================

// bool pickObject(
//     moveit::planning_interface::MoveGroupInterface& arm,
//     const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
//     moveit::planning_interface::PlanningSceneInterface& psi,
//     const object_msgs::msg::Object& obj,
//     const std::string& obj_id)
// {
//   auto grasp_orientation = computeGraspOrientation(obj.thin_axis);

//   // Pre-grasp pose — hover above object
//   geometry_msgs::msg::Pose pregrasp = obj.pose;
//   pregrasp.position.z  += PREGRASP_HEIGHT;
//   pregrasp.orientation  = grasp_orientation;

//   // Grasp pose — at object
//   geometry_msgs::msg::Pose grasp = obj.pose;
//   grasp.orientation = grasp_orientation;

//   RCLCPP_INFO(LOGGER, "Moving to pre-grasp for %s", obj_id.c_str());
//   if (!moveTopose(arm, pregrasp))             return false;

//   sendGripper(gripper_pub, GRIPPER_OPEN);

//   RCLCPP_INFO(LOGGER, "Descending to grasp");
//   if (!moveCartesianDown(arm, grasp))         return false;

//   // sendGripper(gripper_pub, GRIPPER_CLOSED);
//   double grasp_width = computeGraspWidth(obj);
//   RCLCPP_INFO(LOGGER, "Closing gripper to %.3fm (object width + margin)", grasp_width);
//   sendGripper(gripper_pub, grasp_width);

//   // Attach collision box to gripper so MoveIt plans around it during transit
//   attachObject(arm, obj, obj_id);

//   // Lift back to pre-grasp height
//   RCLCPP_INFO(LOGGER, "Lifting object");
//   if (!moveCartesianDown(arm, pregrasp))      return false;
//   // if (!liftObject(arm, pregrasp))         return false;

//   return true;
// }

// bool placeObject(
//     moveit::planning_interface::MoveGroupInterface& arm,
//     const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
//     moveit::planning_interface::PlanningSceneInterface& psi,
//     const geometry_msgs::msg::Pose& bin_pose,
//     const std::string& obj_id)
// {
//   // Pre-place pose — hover above bin
//   geometry_msgs::msg::Pose pre_place = bin_pose;
//   pre_place.position.z += PREGRASP_HEIGHT;
//   pre_place.orientation = computeGraspOrientation(2);  // straight down for placing

//   geometry_msgs::msg::Pose place = bin_pose;
//   place.orientation = computeGraspOrientation(2);

//   RCLCPP_INFO(LOGGER, "Moving to pre-place for bin");
//   if (!moveTopose(arm, pre_place))            return false;

//   RCLCPP_INFO(LOGGER, "Descending to place");
//   if (!moveCartesianDown(arm, place))         return false;

//   sendGripper(gripper_pub, GRIPPER_OPEN);

//   // Detach and clean up collision object
//   detachAndRemoveObject(arm, psi, obj_id);

//   // Lift away from bin
//   if (!moveCartesianDown(arm, pre_place))     return false;

//   return true;
// }

// void executePickPlace(
//     moveit::planning_interface::MoveGroupInterface& arm,
//     const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
//     moveit::planning_interface::PlanningSceneInterface& psi,
//     const object_msgs::msg::ObjectArray& object_array,
//     const geometry_msgs::msg::PoseArray& goal_poses)
// {
//   if (object_array.objects.empty())
//   {
//     RCLCPP_WARN(LOGGER, "No objects to pick.");
//     return;
//   }
//   if (goal_poses.poses.empty())
//   {
//     RCLCPP_WARN(LOGGER, "No goal poses received.");
//     return;
//   }

//   // Sort objects by bin_index to minimise travel
//   std::vector<object_msgs::msg::Object> sorted = object_array.objects;
//   std::sort(sorted.begin(), sorted.end(),
//     [](const object_msgs::msg::Object& a, const object_msgs::msg::Object& b){
//       return a.bin_index < b.bin_index;
//     });

//   // Add all objects to the planning scene upfront
//   for (size_t i = 0; i < sorted.size(); ++i)
//   {
//     std::string id = "object_" + std::to_string(i);
//     addObjectCollision(psi, sorted[i], id);
//   }

//   // Execute pick and place for each object
//   for (size_t i = 0; i < sorted.size(); ++i)
//   {
//     const auto& obj = sorted[i];
//     std::string id  = "object_" + std::to_string(i);

//     if (obj.bin_index >= goal_poses.poses.size())
//     {
//       RCLCPP_ERROR(LOGGER, "bin_index %d out of range for goal_poses", obj.bin_index);
//       continue;
//     }

//     const auto& bin_pose = goal_poses.poses[obj.bin_index];

//     RCLCPP_INFO(LOGGER, "--- Object %zu | class: %s | bin: %d ---",
//       i, obj.classification.c_str(), obj.bin_index);

//     if (!pickObject(arm, gripper_pub, psi, obj, id))
//     {
//       RCLCPP_ERROR(LOGGER, "Pick failed for object %zu, skipping.", i);
//       detachAndRemoveObject(arm, psi, id);
//       continue;
//     }

//     if (!placeObject(arm, gripper_pub, psi, bin_pose, id))
//     {
//       RCLCPP_ERROR(LOGGER, "Place failed for object %zu.", i);
//       detachAndRemoveObject(arm, psi, id);
//     }
//   }

//   RCLCPP_INFO(LOGGER, "All objects processed. Returning home.");
//   returnHome(arm, gripper_pub);
// }


// // ============================================================
// // Main
// // ============================================================

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("pick_place_demo");

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   std::thread spinner([&executor]() { executor.spin(); });

//   // Shared state — protected by mutex
//   std::mutex data_mutex;
//   object_msgs::msg::ObjectArray::SharedPtr latest_objects;
//   geometry_msgs::msg::PoseArray::SharedPtr latest_goals;
//   std::atomic<bool> sequence_requested{false};

//   // Subscribers
//   auto object_sub = node->create_subscription<object_msgs::msg::ObjectArray>(
//     "perception/objects", 10,
//     [&](const object_msgs::msg::ObjectArray::SharedPtr msg) {
//       std::lock_guard<std::mutex> lock(data_mutex);
//       latest_objects = msg;
//       RCLCPP_INFO(LOGGER, "Received %zu objects", msg->objects.size());
//     });

//   auto goal_sub = node->create_subscription<geometry_msgs::msg::PoseArray>(
//     "perception/goal_poses", 10,
//     [&](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
//       std::lock_guard<std::mutex> lock(data_mutex);
//       latest_goals = msg;
//       RCLCPP_INFO(LOGGER, "Received %zu goal poses", msg->poses.size());
//     });

//   // Service trigger
//   auto trigger_srv = node->create_service<std_srvs::srv::Trigger>(
//     "start_pick_place",
//     [&](const std_srvs::srv::Trigger::Request::SharedPtr,
//               std_srvs::srv::Trigger::Response::SharedPtr res) {
//       std::lock_guard<std::mutex> lock(data_mutex);
//       if (!latest_objects || !latest_goals) {
//         res->success = false;
//         res->message = "Waiting for perception data — objects or goals not yet received.";
//         return;
//       }
//       sequence_requested = true;
//       res->success = true;
//       res->message = "Pick and place sequence started.";
//     });

//   // Gripper publisher
//   auto gripper_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
//     "/finger_width_controller/commands", 10);

//   moveit::planning_interface::MoveGroupInterface arm(node, ARM_GROUP);
//   moveit::planning_interface::PlanningSceneInterface psi;

  

//   // user RRTConnect as planner
//   arm.setPlannerId("RRTConnect");
//   arm.setMaxVelocityScalingFactor(0.3);
//   arm.setMaxAccelerationScalingFactor(0.3);


//   // allow for more time to plan so better solution can occur
//   arm.setPlanningTime(10.0);

//   // penalise path with big angular difference
//   arm.setGoalJointTolerance(0.01);
//   arm.setGoalOrientationTolerance(0.01);
//   arm.setGoalPositionTolerance(0.005);

//   std::this_thread::sleep_for(std::chrono::seconds(5));

//   moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link");
//   visual_tools.deleteAllMarkers();
//   visual_tools.loadRemoteControl();

//   placeGround(arm, psi);
//   returnHome(arm, gripper_pub);

//   // Open terminal for keypress trigger
//   std::ifstream tty("/dev/tty");

//   RCLCPP_INFO(LOGGER, "Ready. Press ENTER or call /start_pick_place to begin.");
//   RCLCPP_INFO(LOGGER, "Press 'q' to quit.");

//   while (rclcpp::ok())
//   {
//     // Check for keypress
//     std::cout << "\nWaiting for objects... Press ENTER to start, 'q' to quit.\n>> ";
//     std::string line;
//     std::getline(tty, line);
//     char cmd = line.empty() ? '\n' : line[0];

//     if (cmd == 'q') break;

//     // Also check if service triggered
//     bool go = (cmd == '\n') || sequence_requested.exchange(false);

//     if (!go) continue;

//     // Grab latest data under lock
//     object_msgs::msg::ObjectArray objects_copy;
//     geometry_msgs::msg::PoseArray  goals_copy;
//     {
//       std::lock_guard<std::mutex> lock(data_mutex);
//       if (!latest_objects || !latest_goals) {
//         RCLCPP_WARN(LOGGER, "No perception data yet — publish objects first.");
//         continue;
//       }
//       objects_copy = *latest_objects;
//       goals_copy   = *latest_goals;
//     }

//     executePickPlace(arm, gripper_pub, psi, objects_copy, goals_copy);
//   }

//   rclcpp::shutdown();
//   spinner.join();
//   return 0;
// }
















// Pick and place — UR3e + OnRobot RG2
// Receives ObjectArray from perception, computes grasp strategy per object.
//
// Strategy selection:
//   TOP_DOWN — thin axis is X or Y, object fits within gripper span from above
//   SIDE     — object too tall/wide for top-down, approach horizontally along thinnest face

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <object_msgs/msg/object_array.hpp>
#include <object_msgs/msg/object.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>

namespace rvt = rviz_visual_tools;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_demo");

const std::string ARM_GROUP     = "ur_onrobot_manipulator";
const std::string GRIPPER_GROUP = "ur_onrobot_gripper";
const double PREGRASP_HEIGHT    = 0.03;   // metres above object centre
const double GRIPPER_OPEN       = 0.110;
const double GRIPPER_CLOSED     = 0.01;
const double SAFE_Z_HEIGHT      = 0.20;

// RG2 physical limits
const double RG2_MAX_SPAN       = 0.110;  // metres
const double RG2_FINGER_MARGIN  = 0.050;  // safety margin each side

// If the graspable dimension exceeds this, switch to SIDE strategy
const double TOP_DOWN_MAX_SPAN  = RG2_MAX_SPAN - 0.01;  // 0.090m
// If object height exceeds this, top-down risks wrist collision with object top
const double TOP_DOWN_MAX_HEIGHT = 0.080;  // metres

// Material -> bin index
const std::map<std::string, int> BIN_MAP = {
  {"metal",   0},
  {"plastic", 1},
  {"fabric",  2},
};

enum class GraspStrategy { TOP_DOWN, SIDE };


// ============================================================
// Grasp geometry — computed fresh from dimensions + pose
// ============================================================
// CHECKPOINT
struct GraspGeometry
{
  GraspStrategy strategy;
  int           thin_axis;      // 0=x, 1=y, 2=z in object frame
  double        grip_width;     // metres — commanded finger separation
  double        object_yaw;     // radians — yaw extracted from object pose
};

struct ResolvedObject
{
  object_msgs::msg::Object obj;

  int bin_index;
  GraspGeometry grasp;

  std::string id;   // unique scene ID
};


// Extract yaw from a geometry_msgs Quaternion
double extractYaw(const geometry_msgs::msg::Quaternion& q_msg)
{
  tf2::Quaternion q;
  tf2::fromMsg(q_msg, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

GraspGeometry computeGraspGeometry(const object_msgs::msg::Object& obj)
{
  GraspGeometry g;
  const auto& d = obj.dimensions;  // [dx, dy, dz] in object frame

  // Find thin axis (smallest dimension)
  g.thin_axis = 0;
  if (d[1] < d[g.thin_axis]) g.thin_axis = 1;
  if (d[2] < d[g.thin_axis]) g.thin_axis = 2;

  // The two non-thin dimensions
  std::vector<double> other_dims;
  for (int i = 0; i < 3; ++i)
    if (i != g.thin_axis) other_dims.push_back(d[i]);

  double graspable_span = *std::min_element(other_dims.begin(), other_dims.end());
  double height_in_world = d[2];  // z dimension = height for upright objects

  g.object_yaw = extractYaw(obj.pose.orientation);

  // Strategy decision
  if (graspable_span <= TOP_DOWN_MAX_SPAN && height_in_world <= TOP_DOWN_MAX_HEIGHT)
    g.strategy = GraspStrategy::TOP_DOWN;
  else
    g.strategy = GraspStrategy::SIDE;

  // Grip width — thin axis dimension + small clearance, clamped to RG2 range
  g.grip_width = std::clamp(d[g.thin_axis] + 0.005, 0.0, RG2_MAX_SPAN);

  RCLCPP_INFO(LOGGER,
    "Grasp geometry: strategy=%s thin_axis=%d dims=[%.3f %.3f %.3f] grip_width=%.3f yaw=%.2f",
    g.strategy == GraspStrategy::TOP_DOWN ? "TOP_DOWN" : "SIDE",
    g.thin_axis, d[0], d[1], d[2], g.grip_width, g.object_yaw);

  return g;
}


// ============================================================
// Grasp orientation
// ============================================================

// TOP_DOWN: gripper points straight down, rotated about Z to align fingers
// with the thin axis of the object (fingers straddle the thinnest face).
// The object_yaw rotates the whole thing into world frame.
//
// SIDE: gripper approaches horizontally. We tilt 90deg about Y so the
// finger axis is horizontal, then rotate about Z to face the thin side.
geometry_msgs::msg::Quaternion computeGraspOrientation(
  const GraspGeometry& g,
  GraspStrategy strategy_override = GraspStrategy::TOP_DOWN,
  bool use_override = false)
{
  GraspStrategy strat = use_override ? strategy_override : g.strategy;

  tf2::Quaternion q;

  if (strat == GraspStrategy::TOP_DOWN)
  {
    // Base: gripper Z points down (wrist flipped)
    // Rotate about Z by object_yaw + thin_axis offset so fingers align with thin face
    double finger_angle = g.object_yaw;
    if (g.thin_axis == 1) finger_angle += M_PI / 2.0;  // Y-thin needs 90deg offset
    // thin_axis==2 (flat object) or 0: no additional offset

    q.setRPY(M_PI, 0.0, finger_angle);
  }
  else  // SIDE
  {
    // Tilt gripper 90deg so it approaches from the side
    // Then rotate about Z to face the thin axis in world frame
    double approach_angle = g.object_yaw;
    if (g.thin_axis == 0) approach_angle += M_PI / 2.0;

    q.setRPY(M_PI / 2.0, 0.0, approach_angle);
  }

  q.normalize();
  return tf2::toMsg(q);
}


// ============================================================
// Gripper
// ============================================================

void sendGripper(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& pub,
    double width)
{
  width = std::clamp(width, 0.0, RG2_MAX_SPAN);
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {width};
  pub->publish(msg);
  RCLCPP_INFO(LOGGER, "Gripper → %.3fm", width);
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
}


// ============================================================
// Collision objects
// ============================================================

void addObjectCollision(
    moveit::planning_interface::PlanningSceneInterface& psi,
    const object_msgs::msg::Object& obj,
    const std::string& id)
{
  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = "base_link";
  co.id = id;

  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {obj.dimensions[0], obj.dimensions[1], obj.dimensions[2]};

  co.primitives.push_back(box);
  co.primitive_poses.push_back(obj.pose);
  co.operation = co.ADD;

  psi.addCollisionObjects({co});
  RCLCPP_INFO(LOGGER, "Added collision object: %s", id.c_str());
}

// void attachObject(
//     moveit::planning_interface::MoveGroupInterface& arm,
//     const object_msgs::msg::Object& obj,
//     const GraspGeometry& g,
//     const std::string& id)
// {
//   // Shrink on the thin axis so MoveIt doesn't fight the closed fingers
//   std::vector<double> dims = {obj.dimensions[0], obj.dimensions[1], obj.dimensions[2]};
//   // dims[g.thin_axis] = std::max(0.001, dims[g.thin_axis] - 2.0 * RG2_FINGER_MARGIN);

//   moveit_msgs::msg::CollisionObject co;
//   co.header.frame_id = "tool0";
//   co.id = id;

//   shape_msgs::msg::SolidPrimitive box;
//   box.type = box.BOX;
//   box.dimensions = {dims[0]- 2.0 * RG2_FINGER_MARGIN, dims[1]- 2.0 * RG2_FINGER_MARGIN, dims[2]- 2.0 * RG2_FINGER_MARGIN};

//   geometry_msgs::msg::Pose local_pose;
//   local_pose.position.z    = obj.dimensions[2] / 2.0;
//   local_pose.orientation.w = 1.0;

//   co.primitives.push_back(box);
//   co.primitive_poses.push_back(local_pose);
//   co.operation = moveit_msgs::msg::CollisionObject::ADD;

  
//   std::vector<std::string> touch_links = {
//     "tool0",
//     "onrobot_rg2_base_link",
//     "onrobot_rg2_left_outer_knuckle",
//     "onrobot_rg2_left_inner_knuckle",
//     "onrobot_rg2_left_inner_finger",
//     "onrobot_rg2_left_finger_tip",
//     "onrobot_rg2_right_outer_knuckle",
//     "onrobot_rg2_right_inner_knuckle",
//     "onrobot_rg2_right_inner_finger",
//     "onrobot_rg2_right_finger_tip",
//     "onrobot_rg2_finger_width_mock_link",
//     "onrobot_rg2_gripper_tcp"
//   };

  

//   arm.attachObject(id, "tool0", touch_links);
//   RCLCPP_INFO(LOGGER, "Attached: %s (thin_axis=%d shrunk)", id.c_str(), g.thin_axis);
// }

// void attachObject(
//   moveit::planning_interface::MoveGroupInterface& arm,
//   const object_msgs::msg::Object& obj,
//   const GraspGeometry& g,
//   const std::string& id)
// {
//   // Shrink the attached box slightly on the gripper closing axis (Y)
//   // so MoveIt doesn't see it as colliding with the closed fingers
//   const double GRASP_MARGIN = 0.01;  // 1cm shrink per side

//   moveit_msgs::msg::CollisionObject co;
//   co.header.frame_id = "tool0";
//   co.id = id;

//   shape_msgs::msg::SolidPrimitive box;
//   box.type = box.BOX;
//   box.dimensions = {
//     obj.dimensions[0]- 2.0 * GRASP_MARGIN,
//     obj.dimensions[1] - 2.0 * GRASP_MARGIN,  // shrink on closing axis
//     obj.dimensions[2]- 2.0 * GRASP_MARGIN
//   };

//   geometry_msgs::msg::Pose local_pose;
//   local_pose.position.z    = obj.dimensions[2] / 2.0;
//   local_pose.orientation.w = 1.0;

//   co.primitives.push_back(box);
//   co.primitive_poses.push_back(local_pose);
//   co.operation = moveit_msgs::msg::CollisionObject::ADD;

//   std::vector<std::string> touch_links = {
//     "tool0",
//     "onrobot_rg2_base_link",
//     "onrobot_rg2_left_outer_knuckle",
//     "onrobot_rg2_left_inner_knuckle",
//     "onrobot_rg2_left_inner_finger",
//     "onrobot_rg2_left_finger_tip",
//     "onrobot_rg2_right_outer_knuckle",
//     "onrobot_rg2_right_inner_knuckle",
//     "onrobot_rg2_right_inner_finger",
//     "onrobot_rg2_right_finger_tip",
//     "onrobot_rg2_finger_width_mock_link",
//     "onrobot_rg2_gripper_tcp"
//   };

//   arm.attachObject(id, "tool0", touch_links);
//   RCLCPP_INFO(LOGGER, "Attached object: %s (dims shrunk on grasp axis)", id.c_str());
// }

// void attachObject(
//   moveit::planning_interface::MoveGroupInterface& arm,
//   const object_msgs::msg::Object& obj,
//   const GraspGeometry& g,
//   const std::string& id)
// {
//   std::vector<std::string> touch_links = {
//     "tool0",
//     "onrobot_rg2_base_link",
//     "onrobot_rg2_left_outer_knuckle",
//     "onrobot_rg2_left_inner_knuckle",
//     "onrobot_rg2_left_inner_finger",
//     "onrobot_rg2_left_finger_tip",
//     "onrobot_rg2_right_outer_knuckle",
//     "onrobot_rg2_right_inner_knuckle",
//     "onrobot_rg2_right_inner_finger",
//     "onrobot_rg2_right_finger_tip",
//     "onrobot_rg2_finger_width_mock_link",
//     "onrobot_rg2_gripper_tcp"
//   };

//   // Transition the existing world collision object to attached —
//   // do NOT re-add it, just attach by id
//   arm.attachObject(id, "tool0", touch_links);
//   rclcpp::sleep_for(std::chrono::milliseconds(200));
//   RCLCPP_INFO(LOGGER, "Attached: %s", id.c_str());
// }

void attachObject(
  moveit::planning_interface::MoveGroupInterface& arm,
  const std::string& id)
{
  std::vector<std::string> touch_links = {
    "tool0",
    "onrobot_rg2_base_link",
    "onrobot_rg2_left_outer_knuckle",  "onrobot_rg2_left_inner_knuckle",
    "onrobot_rg2_left_inner_finger",   "onrobot_rg2_left_finger_tip",
    "onrobot_rg2_right_outer_knuckle", "onrobot_rg2_right_inner_knuckle",
    "onrobot_rg2_right_inner_finger",  "onrobot_rg2_right_finger_tip",
    "onrobot_rg2_finger_width_mock_link",
    "onrobot_rg2_gripper_tcp"
  };
  arm.attachObject(id, "tool0", touch_links);
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  RCLCPP_INFO(LOGGER, "Attached: %s", id.c_str());
}

void attachObject(
  moveit::planning_interface::MoveGroupInterface& arm,
  moveit::planning_interface::PlanningSceneInterface& psi,
  const object_msgs::msg::Object& obj,
  const std::string& id)
{
  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = "tool0";
  co.id = id;

  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;

  const double SHRINK = 0.01;

  box.dimensions = {
    std::max(0.001, obj.dimensions[0] - SHRINK),
    std::max(0.001, obj.dimensions[1] - SHRINK),
    std::max(0.001, obj.dimensions[2] - SHRINK)
  };

  geometry_msgs::msg::Pose p;
  p.orientation.w = 1.0;
  p.position.z = obj.dimensions[2] / 2.0;

  co.primitives.push_back(box);
  co.primitive_poses.push_back(p);
  co.operation = co.ADD;

  psi.applyCollisionObject(co);

  std::vector<std::string> touch_links = {
    "tool0",
    "onrobot_rg2_base_link",
    "onrobot_rg2_left_outer_knuckle",
    "onrobot_rg2_left_inner_knuckle",
    "onrobot_rg2_left_inner_finger",
    "onrobot_rg2_left_finger_tip",
    "onrobot_rg2_right_outer_knuckle",
    "onrobot_rg2_right_inner_knuckle",
    "onrobot_rg2_right_inner_finger",
    "onrobot_rg2_right_finger_tip",
    "gripper_tcp",
    "wrist_3_link"
  };

  arm.attachObject(id, "tool0", touch_links);
}

void detachAndRemoveObject(
    moveit::planning_interface::MoveGroupInterface& arm,
    moveit::planning_interface::PlanningSceneInterface& psi,
    const std::string& id)
{
  arm.detachObject(id);
  psi.removeCollisionObjects(std::vector<std::string>{id});
  RCLCPP_INFO(LOGGER, "Detached and removed: %s", id.c_str());
}


// void attachObject(
  //     moveit::planning_interface::MoveGroupInterface& arm,
  //     const object_msgs::msg::Object& obj,
  //     const std::string& id)
  // {
  //   moveit_msgs::msg::AttachedCollisionObject aco;
  //   aco.link_name = "tool0";
  //   aco.object.header.frame_id = "tool0";
  //   aco.object.id = id;
  
  //   shape_msgs::msg::SolidPrimitive box;
  //   box.type = box.BOX;
  //   box.dimensions = {obj.dimensions[0], obj.dimensions[1], obj.dimensions[2]};
  
  //   geometry_msgs::msg::Pose local_pose;
  //   local_pose.position.z    = obj.dimensions[2] / 2.0;  // centred below tool0
  //   local_pose.orientation.w = 1.0;
  
  //   aco.object.primitives.push_back(box);
  //   aco.object.primitive_poses.push_back(local_pose);
  //   aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  //   aco.touch_links = {"tool0", "onrobot_rg2_base_link"};  // links allowed to touch
  
  //   // All gripper links that will be in contact with the object when grasping
  //   // this is done to avoid collision checking between gripper and object during the grasp — we want MoveIt to plan as if the object is part of the gripper
  //   std::vector<std::string> touch_links = {
  //     "tool0",
  //     "onrobot_rg2_base_link",          // base of gripper
  //     "onrobot_rg2_left_outer_knuckle", // left finger chain
  //     "onrobot_rg2_left_inner_knuckle",
  //     "onrobot_rg2_left_inner_finger",
  //     "onrobot_rg2_left_finger_tip",
  //     "onrobot_rg2_right_outer_knuckle", // right finger chain
  //     "onrobot_rg2_right_inner_knuckle",
  //     "onrobot_rg2_right_inner_finger",
  //     "onrobot_rg2_right_finger_tip",
  //     "onrobot_rg2_finger_width_mock_link", // prismatic mock link
  //     "onrobot_rg2_gripper_tcp"             // TCP frame
  //   };
  //   arm.attachObject(id, "tool0", touch_links);
  //   RCLCPP_INFO(LOGGER, "Attached object: %s", id.c_str());
  // }

// ============================================================
// Scene setup
// ============================================================

void placeGround(
    moveit::planning_interface::MoveGroupInterface& arm,
    moveit::planning_interface::PlanningSceneInterface& psi)
{
  moveit_msgs::msg::CollisionObject ground;
  ground.header.frame_id = arm.getPlanningFrame();
  ground.id = "ground";

  
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type       = primitive.BOX;
  primitive.dimensions = {2.0, 2.0, 0.10};

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.z    = -0.075;

  ground.primitives.push_back(primitive);
  ground.primitive_poses.push_back(pose);
  ground.operation = ground.ADD;

  psi.addCollisionObjects({ground});

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

}

void returnHome(
    moveit::planning_interface::MoveGroupInterface& arm,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub)
{
  arm.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    arm.execute(plan);
  sendGripper(gripper_pub, GRIPPER_OPEN);
}


// ============================================================
// Motion primitives
// ============================================================

bool moveToPose(
    moveit::planning_interface::MoveGroupInterface& arm,
    const geometry_msgs::msg::Pose& pose)
{
  arm.setPoseTarget(pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Joint-space planning failed");
    return false;
  }
  return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool moveCartesian(
    moveit::planning_interface::MoveGroupInterface& arm,
    const geometry_msgs::msg::Pose& target,
    double min_fraction = 0.9)
{
  std::vector<geometry_msgs::msg::Pose> waypoints{target};
  moveit_msgs::msg::RobotTrajectory traj;
  double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, traj);

  if (fraction < min_fraction)
  // if (fraction < 0.9)
  {
    RCLCPP_ERROR(LOGGER, "Cartesian path only %.0f%% complete", fraction * 100.0);
    return false;
  }
  
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = traj;
  return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}


// ============================================================
// Pick strategies
// ============================================================

// TOP_DOWN: hover above → open → descend → close → lift
bool pickTopDown(
    moveit::planning_interface::MoveGroupInterface& arm,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    moveit::planning_interface::PlanningSceneInterface& psi,
    const object_msgs::msg::Object& obj,
    const GraspGeometry& g,
    const std::string& obj_id)
{
  auto orientation = computeGraspOrientation(g);

  geometry_msgs::msg::Pose pregrasp = obj.pose;
  // pregrasp.position.z += PREGRASP_HEIGHT;
  pregrasp.position.z += obj.dimensions[2] / 2.0 + PREGRASP_HEIGHT;
  pregrasp.orientation = orientation;

  geometry_msgs::msg::Pose grasp = obj.pose;
  grasp.orientation = orientation;

  RCLCPP_INFO(LOGGER, "[TOP_DOWN] Moving to pre-grasp");
  if (!moveToPose(arm, pregrasp))      return false;

  sendGripper(gripper_pub, GRIPPER_OPEN);

  RCLCPP_INFO(LOGGER, "[TOP_DOWN] Descending");
  if (!moveCartesian(arm, grasp))      return false;

  // For now Attach first before gripping to avoid collision flags/ henceforth need to implement grasp verification here after send gripper or lifting
  // attachObject(arm, obj, g, obj_id);
  // sendGripper(gripper_pub, g.grip_width);

  
  // // force start state
  // arm.setStartStateToCurrentState();
  // // rclcpp::sleep_for(std::chrono::milliseconds(20000));

  // RCLCPP_INFO(LOGGER, "[TOP_DOWN] Lifting");
  // geometry_msgs::msg::Pose lift = arm.getCurrentPose().pose;
  // lift.position.z = SAFE_Z_HEIGHT;  // fixed world height, known to be clear
  // if (!moveCartesian(arm, lift)) {
  //   // if cartesian fails. fallback onto joint space
  //   if (!moveToPose(arm, lift)) return false;
  // }
  

  // attachObject(arm, obj_id);
  attachObject(arm, psi, obj, obj_id);
  
  sendGripper(gripper_pub, g.grip_width);
  auto scene = psi.getObjects({obj_id});
  auto attached = psi.getAttachedObjects({obj_id});
  RCLCPP_INFO(LOGGER, "world has %zu, attached has %zu copies of %s",
              scene.size(), attached.size(), obj_id.c_str());

  // Let joint_states + planning scene catch up before re-planning
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  arm.setStartStateToCurrentState();

  RCLCPP_INFO(LOGGER, "[TOP_DOWN] Lifting");
  geometry_msgs::msg::Pose lift = arm.getCurrentPose().pose;
  lift.position.z = SAFE_Z_HEIGHT;
  if (!moveCartesian(arm, lift)) {
    if (!moveToPose(arm, lift)) return false;
  }
  // grasp verification here. possibly service call from perception to check... or slowly decrease width until there is mismatch between goal width and actual width.

  return true;
}

// SIDE: approach horizontally from thin face, close, lift vertically
// The gripper comes in from the side, closes around the object,
// then lifts straight up. Object collision is disabled during approach
// so the finger can slide under if needed.
bool pickSide(
    moveit::planning_interface::MoveGroupInterface& arm,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    moveit::planning_interface::PlanningSceneInterface& psi,
    const object_msgs::msg::Object& obj,
    const GraspGeometry& g,
    const std::string& obj_id)
{
  auto orientation = computeGraspOrientation(g);

  // Approach offset — come from the side along the thin axis direction
  // 0.15m standoff so gripper clears the object before descending to grasp height
  const double SIDE_STANDOFF = 0.15;

  // Approach direction in world frame based on thin axis + object yaw
  double approach_yaw = g.object_yaw;
  if (g.thin_axis == 1) approach_yaw += M_PI / 2.0;

  double dx = -std::cos(approach_yaw) * SIDE_STANDOFF;
  double dy = -std::sin(approach_yaw) * SIDE_STANDOFF;

  // Grasp height: mid-height of the object
  double grasp_z = obj.pose.position.z + obj.dimensions[2] / 2.0;

  geometry_msgs::msg::Pose approach = obj.pose;
  approach.position.x  += dx;
  approach.position.y  += dy;
  approach.position.z   = grasp_z;
  approach.orientation  = orientation;

  geometry_msgs::msg::Pose grasp = obj.pose;
  grasp.position.z      = grasp_z;
  grasp.orientation     = orientation;

  geometry_msgs::msg::Pose lifted = grasp;
  lifted.position.z    += PREGRASP_HEIGHT;
  lifted.orientation    = computeGraspOrientation(g, GraspStrategy::TOP_DOWN, true);

  RCLCPP_INFO(LOGGER, "[SIDE] Moving to side approach");
  if (!moveToPose(arm, approach))      return false;

  sendGripper(gripper_pub, GRIPPER_OPEN);

  // Remove object collision — finger needs to slide under the edge
  // psi.removeCollisionObjects({obj_id});
  psi.removeCollisionObjects(std::vector<std::string>{obj_id});
  RCLCPP_INFO(LOGGER, "[SIDE] Object collision disabled for approach");

  RCLCPP_INFO(LOGGER, "[SIDE] Horizontal advance to grasp");
  if (!moveCartesian(arm, grasp))
  {
    // Re-add collision and abort if we can't reach
    addObjectCollision(psi, obj, obj_id);
    return false;
  }

  // For now Attach first before gripping to avoid collision flags/ henceforth need to implement grasp verification here after send gripper or lifting
  // attachObject(arm, obj, g, obj_id);
  // sendGripper(gripper_pub, g.grip_width);
  // // attachObject(arm, obj, g, obj_id);

  // // force start state
  // arm.setStartStateToCurrentState();
  // rclcpp::sleep_for(std::chrono::milliseconds(200));

  // RCLCPP_INFO(LOGGER, "[SIDE] Lifting");
  // // if (!moveCartesian(arm, lifted)){
  // //   if (!moveToPose(arm, lifted)) return false; 
  // // }

  // geometry_msgs::msg::Pose lift = arm.getCurrentPose().pose;
  // lift.position.z = SAFE_Z_HEIGHT;  // fixed world height, known to be clear
  // if (!moveCartesian(arm, lift)) {
  //   // if cartesian fails. fallback onto joint space
  //   if (!moveToPose(arm, lift)) return false;
  // }


  // attachObject(arm, obj_id);
attachObject(arm, psi, obj, obj_id);
  sendGripper(gripper_pub, g.grip_width);

  // Let joint_states + planning scene catch up before re-planning
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  arm.setStartStateToCurrentState();

  RCLCPP_INFO(LOGGER, "[SIDE] Lifting");
  geometry_msgs::msg::Pose lift = arm.getCurrentPose().pose;
  lift.position.z = SAFE_Z_HEIGHT;
  if (!moveCartesian(arm, lift)) {
    if (!moveToPose(arm, lift)) return false;
  }
  return true;
}


// ============================================================
// Place (shared for both strategies — always place top-down)
// ============================================================

bool placeObject(
    moveit::planning_interface::MoveGroupInterface& arm,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    moveit::planning_interface::PlanningSceneInterface& psi,
    const geometry_msgs::msg::Pose& bin_pose,
    const std::string& obj_id)
{
  // Neutral top-down orientation for placing
  tf2::Quaternion q_down;
  q_down.setRPY(M_PI, 0.0, 0.0);
  auto orientation = tf2::toMsg(q_down);

  geometry_msgs::msg::Pose pre_place = bin_pose;
  pre_place.position.z += PREGRASP_HEIGHT;
  pre_place.orientation = orientation;

  geometry_msgs::msg::Pose place = bin_pose;
  place.orientation = orientation;

  RCLCPP_INFO(LOGGER, "Moving to pre-place");
  if (!moveToPose(arm, pre_place))         return false;

  RCLCPP_INFO(LOGGER, "Descending to bin");
  if (!moveCartesian(arm, place))          return false;

  sendGripper(gripper_pub, GRIPPER_OPEN);
  detachAndRemoveObject(arm, psi, obj_id);

  RCLCPP_INFO(LOGGER, "Retreating from bin");
  if (!moveCartesian(arm, pre_place))      return false;

  return true;
}

// Resolve bin index from classification and sort by bin to minimise travel
std::vector<ResolvedObject> resolveObjects(
const object_msgs::msg::ObjectArray& object_array){
  std::vector<ResolvedObject> out;

  for (size_t i = 0; i < object_array.objects.size(); ++i)
  {
    const auto& obj = object_array.objects[i];

    auto it = BIN_MAP.find(obj.classification);
    if (it == BIN_MAP.end())
    {
      RCLCPP_WARN(LOGGER,
        "Unknown class '%s' → skipping",
        obj.classification.c_str());
      continue;
    }

    ResolvedObject r;
    r.obj        = obj;
    r.bin_index  = it->second;
    r.grasp      = computeGraspGeometry(obj);
    r.id         = "object_" + std::to_string(i);

    out.push_back(r);
  }

  return out;
}

// ============================================================
// Main pick-place loop
// ============================================================

void executePickPlace(
    moveit::planning_interface::MoveGroupInterface& arm,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    moveit::planning_interface::PlanningSceneInterface& psi,
    const object_msgs::msg::ObjectArray& object_array,
    const geometry_msgs::msg::PoseArray& goal_poses)
{
  if (object_array.objects.empty())  { RCLCPP_WARN(LOGGER, "No objects."); return; }
  if (goal_poses.poses.empty())      { RCLCPP_WARN(LOGGER, "No goal poses."); return; }

  

  auto resolved = resolveObjects(object_array);

  // sort by bin
  std::sort(resolved.begin(), resolved.end(),
    [](const ResolvedObject& a, const ResolvedObject& b)
  {
    return a.bin_index < b.bin_index;
  });

  // add scene
  for (const auto& r : resolved)
    addObjectCollision(psi, r.obj, r.id);

  // execute
  for (const auto& r : resolved)
  {
    if (r.bin_index >= static_cast<int>(goal_poses.poses.size()))
    {
      RCLCPP_ERROR(LOGGER, "Invalid bin index %d", r.bin_index);
      continue;
    }

    RCLCPP_INFO(LOGGER,
      "--- %s | bin %d | %s ---",
      r.obj.classification.c_str(),
      r.bin_index,
      r.grasp.strategy == GraspStrategy::TOP_DOWN ? "TOP_DOWN" : "SIDE");

    bool picked = (r.grasp.strategy == GraspStrategy::TOP_DOWN)
      ? pickTopDown(arm, gripper_pub, psi, r.obj, r.grasp, r.id)
      : pickSide(arm, gripper_pub, psi, r.obj, r.grasp, r.id);

    if (!picked)
    {
      RCLCPP_ERROR(LOGGER, "Pick failed: %s", r.id.c_str());
      arm.detachObject(r.id);
      psi.removeCollisionObjects(std::vector<std::string>{r.id});
      continue;
    }

    if (!placeObject(
          arm,
          gripper_pub,
          psi,
          goal_poses.poses[r.bin_index],
          r.id))
    {
      RCLCPP_ERROR(LOGGER, "Place failed: %s", r.id.c_str());
      detachAndRemoveObject(arm, psi, r.id);
    }
  }

  RCLCPP_INFO(LOGGER, "All objects processed. Returning home.");
  returnHome(arm, gripper_pub);
}


// ============================================================
// Main
// ============================================================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pick_place_demo");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  std::mutex data_mutex;
  object_msgs::msg::ObjectArray::SharedPtr latest_objects;
  geometry_msgs::msg::PoseArray::SharedPtr latest_goals;
  std::atomic<bool> sequence_requested{false};

  auto object_sub = node->create_subscription<object_msgs::msg::ObjectArray>(
    "perception/objects", 10,
    [&](const object_msgs::msg::ObjectArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex);
      latest_objects = msg;
      RCLCPP_INFO(LOGGER, "Received %zu objects", msg->objects.size());
    });

  auto goal_sub = node->create_subscription<geometry_msgs::msg::PoseArray>(
    "perception/goal_poses", 10,
    [&](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex);
      latest_goals = msg;
      RCLCPP_INFO(LOGGER, "Received %zu goal poses", msg->poses.size());
    });

  auto trigger_srv = node->create_service<std_srvs::srv::Trigger>(
    "start_pick_place",
    [&](const std_srvs::srv::Trigger::Request::SharedPtr,
              std_srvs::srv::Trigger::Response::SharedPtr res) {
      std::lock_guard<std::mutex> lock(data_mutex);
      if (!latest_objects || !latest_goals) {
        res->success = false;
        res->message = "No perception data yet.";
        return;
      }
      sequence_requested = true;
      res->success = true;
      res->message = "Sequence started.";
    });

  auto gripper_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/finger_width_controller/commands", 10);

  moveit::planning_interface::MoveGroupInterface arm(node, ARM_GROUP);
  moveit::planning_interface::PlanningSceneInterface psi;

  // arm.setPlannerId("RRTConnect");
  arm.setMaxVelocityScalingFactor(0.3);
  arm.setMaxAccelerationScalingFactor(0.3);
  arm.setPlanningTime(10.0);
  arm.setGoalJointTolerance(0.01);
  arm.setGoalOrientationTolerance(0.01);
  arm.setGoalPositionTolerance(0.005);

  std::this_thread::sleep_for(std::chrono::seconds(5));

  arm.setStartStateToCurrentState();   // warm call to force monitor init
  RCLCPP_INFO(LOGGER, "Current EEF: %s", arm.getEndEffectorLink().c_str());

  moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  placeGround(arm, psi);
  returnHome(arm, gripper_pub);

  std::ifstream tty("/dev/tty");
  RCLCPP_INFO(LOGGER, "Ready. Press ENTER to start, 'q' to quit.");

  while (rclcpp::ok())
  {
    std::cout << "\nPress ENTER to start, 'q' to quit.\n>> ";
    std::string line;
    std::getline(tty, line);
    char cmd = line.empty() ? '\n' : line[0];
    if (cmd == 'q') break;

    bool go = (cmd == '\n') || sequence_requested.exchange(false);
    if (!go) continue;

    object_msgs::msg::ObjectArray objects_copy;
    geometry_msgs::msg::PoseArray  goals_copy;
    {
      std::lock_guard<std::mutex> lock(data_mutex);
      if (!latest_objects || !latest_goals) {
        RCLCPP_WARN(LOGGER, "No perception data yet.");
        continue;
      }
      objects_copy = *latest_objects;
      goals_copy   = *latest_goals;
    }

    executePickPlace(arm, gripper_pub, psi, objects_copy, goals_copy);
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}