// Pick and place unit test for the UR3e demo
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>

// Force planner to minimize joint-space distance, not just find any valid path
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>

namespace rvt = rviz_visual_tools;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_demo");

const std::string ARM_GROUP     = "ur_onrobot_manipulator";
const std::string GRIPPER_GROUP = "ur_onrobot_gripper";
const double PREGRASP_HEIGHT    = 0.05;   // metres above object
const double GRIPPER_OPEN       = 0.085;
const double GRIPPER_CLOSED     = 0.01;


// ============================================================
// Grasp Orientation
// Computes a gripper quaternion aligned to the object's thin axis.
// The gripper always points down (Z down), rotated about Z to align
// with the thinnest dimension of the object.
// thin_axis: 0=x, 1=y, 2=z (index of thinnest dimension)
// ============================================================

// 
// geometry_msgs::msg::Quaternion computeGraspOrientation(uint8_t thin_axis)
// {
//   tf2::Quaternion q;

//   switch (thin_axis)
//   {
//     case 0:
//       // Thin along X — gripper fingers should span Y, rotate 90deg about Z
//       q.setRPY(M_PI, 0.0, M_PI / 2.0);
//       break;
//     case 1:
//       // Thin along Y — gripper fingers should span X, no rotation needed
//       q.setRPY(M_PI, 0.0, 0.0);
//       break;
//     case 2:
//     default:
//       // Thin along Z (flat object) — grasp straight down
//       q.setRPY(M_PI, 0.0, 0.0);
//       break;
//   }

//   q.normalize();
//   return tf2::toMsg(q);
// }


// Graps aliong the thin axis of the object so fingers slip along narrow sides rather than spanning wide face
geometry_msgs::msg::Quaternion computeGraspOrientation(uint8_t thin_axis)
{
  tf2::Quaternion q;

  // RG2 fingers are thin — align them WITH the thin axis of the object
  // so the fingers slip along the narrow sides rather than spanning the wide face
  switch (thin_axis)
  {
    case 0:
      // Thin along X — align fingers with X, no rotation needed
      q.setRPY(M_PI, 0.0, 0.0);
      break;
    case 1:
      // Thin along Y — align fingers with Y, rotate 90deg about Z
      q.setRPY(M_PI, 0.0, M_PI / 2.0);
      break;
    case 2:
    default:
      // Thin along Z (flat object) — grasp straight down, finger alignment
      // doesn't matter much so default orientation is fine
      q.setRPY(M_PI, 0.0, 0.0);
      break;
  }

  q.normalize();
  return tf2::toMsg(q);
}

// double computeGraspWidth(const object_msgs::msg::Object& obj)
// {
//   // RG2 closes along Y axis by default
//   // The grasp width is the object dimension on the closing axis
//   // thin_axis tells us the thinnest — fingers align with thin axis,
//   // so they close around the dimension perpendicular to it
  
//   // After sorting, dimensions = [thin, mid, thick]
//   // thin_axis = 0 always (after sorting in test helper)
//   // So closing axis dimension is dimensions[1] (mid) — the next smallest

//   double grasp_width = obj.dimensions[1];  // dimension fingers close around

//   // Add a small margin so the gripper doesn't fully crush the object
//   const double GRASP_MARGIN = 0.005;  // 5mm margin
//   grasp_width += GRASP_MARGIN;

//   // Clamp to physical gripper limits
//   return std::clamp(grasp_width, 0.0, 0.110);
// }

double computeGraspWidth(const object_msgs::msg::Object& obj)
{
  // Find the two non-thin dimensions and take the smaller one
  std::vector<double> dims = {obj.dimensions[0], obj.dimensions[1], obj.dimensions[2]};
  dims.erase(dims.begin() + obj.thin_axis);  // remove the thin axis
  return std::clamp(*std::min_element(dims.begin(), dims.end()) + 0.005, 0.0, 0.110);
}


// ============================================================
// Gripper Control
// ============================================================

void sendGripper(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& pub,
    double width)
{
  width = std::clamp(width, 0.0, 0.11);
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {width};
  pub->publish(msg);
  RCLCPP_INFO(LOGGER, "Gripper → %.3f", width);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}


// ============================================================
// Collision Object Helpers
// ============================================================

// Add a collision object representing the object in the scene (before picking)
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

// Attach collision object to the gripper after grasping
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

void attachObject(
  moveit::planning_interface::MoveGroupInterface& arm,
  const object_msgs::msg::Object& obj,
  const std::string& id)
{
  // Shrink the attached box slightly on the gripper closing axis (Y)
  // so MoveIt doesn't see it as colliding with the closed fingers
  const double GRASP_MARGIN = 0.01;  // 1cm shrink per side

  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = "tool0";
  co.id = id;

  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {
    obj.dimensions[0],
    obj.dimensions[1] - 2.0 * GRASP_MARGIN,  // shrink on closing axis
    obj.dimensions[2]
  };

  geometry_msgs::msg::Pose local_pose;
  local_pose.position.z    = obj.dimensions[2] / 2.0;
  local_pose.orientation.w = 1.0;

  co.primitives.push_back(box);
  co.primitive_poses.push_back(local_pose);
  co.operation = moveit_msgs::msg::CollisionObject::ADD;

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
    "onrobot_rg2_finger_width_mock_link",
    "onrobot_rg2_gripper_tcp"
  };

  arm.attachObject(id, "tool0", touch_links);
  RCLCPP_INFO(LOGGER, "Attached object: %s (dims shrunk on grasp axis)", id.c_str());
}

// Detach and remove collision object after placing
void detachAndRemoveObject(
    moveit::planning_interface::MoveGroupInterface& arm,
    moveit::planning_interface::PlanningSceneInterface& psi,
    const std::string& id)
{
  arm.detachObject(id);
  psi.removeCollisionObjects({id});
  RCLCPP_INFO(LOGGER, "Detached and removed: %s", id.c_str());
}

bool liftObject(
  moveit::planning_interface::MoveGroupInterface& arm,
  const geometry_msgs::msg::Pose& pregrasp_pose)
{
  // Use joint-space planning for lift — Cartesian fails with attached collision objects
  arm.setPoseTarget(pregrasp_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Lift planning failed");
    return false;
  }
  return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

// ============================================================
// Scene Setup
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
  primitive.dimensions = {2.0, 2.0, 0.1};

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.z    = -0.075;

  ground.primitives.push_back(primitive);
  ground.primitive_poses.push_back(pose);
  ground.operation = ground.ADD;

  psi.addCollisionObjects({ground});
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
// Arm Motion Helpers
// ============================================================

bool moveTopose(
    moveit::planning_interface::MoveGroupInterface& arm,
    const geometry_msgs::msg::Pose& pose)
{
  arm.setPoseTarget(pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Planning failed");
    return false;
  }
  return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool moveCartesianDown(
    moveit::planning_interface::MoveGroupInterface& arm,
    const geometry_msgs::msg::Pose& target)
{
  std::vector<geometry_msgs::msg::Pose> waypoints{target};
  moveit_msgs::msg::RobotTrajectory traj;
  double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, traj);

  if (fraction < 0.9)
  {
    RCLCPP_ERROR(LOGGER, "Cartesian path only %.0f%% complete", fraction * 100.0);
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = traj;
  return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}


// ============================================================
// Pick and Place Sequence
// ============================================================

bool pickObject(
    moveit::planning_interface::MoveGroupInterface& arm,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    moveit::planning_interface::PlanningSceneInterface& psi,
    const object_msgs::msg::Object& obj,
    const std::string& obj_id)
{
  auto grasp_orientation = computeGraspOrientation(obj.thin_axis);

  // Pre-grasp pose — hover above object
  geometry_msgs::msg::Pose pregrasp = obj.pose;
  pregrasp.position.z  += PREGRASP_HEIGHT;
  pregrasp.orientation  = grasp_orientation;

  // Grasp pose — at object
  geometry_msgs::msg::Pose grasp = obj.pose;
  grasp.orientation = grasp_orientation;

  RCLCPP_INFO(LOGGER, "Moving to pre-grasp for %s", obj_id.c_str());
  if (!moveTopose(arm, pregrasp))             return false;

  sendGripper(gripper_pub, GRIPPER_OPEN);

  RCLCPP_INFO(LOGGER, "Descending to grasp");
  if (!moveCartesianDown(arm, grasp))         return false;

  // sendGripper(gripper_pub, GRIPPER_CLOSED);
  double grasp_width = computeGraspWidth(obj);
  RCLCPP_INFO(LOGGER, "Closing gripper to %.3fm (object width + margin)", grasp_width);
  sendGripper(gripper_pub, grasp_width);

  // Attach collision box to gripper so MoveIt plans around it during transit
  attachObject(arm, obj, obj_id);

  // Lift back to pre-grasp height
  RCLCPP_INFO(LOGGER, "Lifting object");
  if (!moveCartesianDown(arm, pregrasp))      return false;
  // if (!liftObject(arm, pregrasp))         return false;

  return true;
}

bool placeObject(
    moveit::planning_interface::MoveGroupInterface& arm,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    moveit::planning_interface::PlanningSceneInterface& psi,
    const geometry_msgs::msg::Pose& bin_pose,
    const std::string& obj_id)
{
  // Pre-place pose — hover above bin
  geometry_msgs::msg::Pose pre_place = bin_pose;
  pre_place.position.z += PREGRASP_HEIGHT;
  pre_place.orientation = computeGraspOrientation(2);  // straight down for placing

  geometry_msgs::msg::Pose place = bin_pose;
  place.orientation = computeGraspOrientation(2);

  RCLCPP_INFO(LOGGER, "Moving to pre-place for bin");
  if (!moveTopose(arm, pre_place))            return false;

  RCLCPP_INFO(LOGGER, "Descending to place");
  if (!moveCartesianDown(arm, place))         return false;

  sendGripper(gripper_pub, GRIPPER_OPEN);

  // Detach and clean up collision object
  detachAndRemoveObject(arm, psi, obj_id);

  // Lift away from bin
  if (!moveCartesianDown(arm, pre_place))     return false;

  return true;
}

void executePickPlace(
    moveit::planning_interface::MoveGroupInterface& arm,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    moveit::planning_interface::PlanningSceneInterface& psi,
    const object_msgs::msg::ObjectArray& object_array,
    const geometry_msgs::msg::PoseArray& goal_poses)
{
  if (object_array.objects.empty())
  {
    RCLCPP_WARN(LOGGER, "No objects to pick.");
    return;
  }
  if (goal_poses.poses.empty())
  {
    RCLCPP_WARN(LOGGER, "No goal poses received.");
    return;
  }

  // Sort objects by bin_index to minimise travel
  std::vector<object_msgs::msg::Object> sorted = object_array.objects;
  std::sort(sorted.begin(), sorted.end(),
    [](const object_msgs::msg::Object& a, const object_msgs::msg::Object& b){
      return a.bin_index < b.bin_index;
    });

  // Add all objects to the planning scene upfront
  for (size_t i = 0; i < sorted.size(); ++i)
  {
    std::string id = "object_" + std::to_string(i);
    addObjectCollision(psi, sorted[i], id);
  }

  // Execute pick and place for each object
  for (size_t i = 0; i < sorted.size(); ++i)
  {
    const auto& obj = sorted[i];
    std::string id  = "object_" + std::to_string(i);

    if (obj.bin_index >= goal_poses.poses.size())
    {
      RCLCPP_ERROR(LOGGER, "bin_index %d out of range for goal_poses", obj.bin_index);
      continue;
    }

    const auto& bin_pose = goal_poses.poses[obj.bin_index];

    RCLCPP_INFO(LOGGER, "--- Object %zu | class: %s | bin: %d ---",
      i, obj.classification.c_str(), obj.bin_index);

    if (!pickObject(arm, gripper_pub, psi, obj, id))
    {
      RCLCPP_ERROR(LOGGER, "Pick failed for object %zu, skipping.", i);
      detachAndRemoveObject(arm, psi, id);
      continue;
    }

    if (!placeObject(arm, gripper_pub, psi, bin_pose, id))
    {
      RCLCPP_ERROR(LOGGER, "Place failed for object %zu.", i);
      detachAndRemoveObject(arm, psi, id);
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

  // Shared state — protected by mutex
  std::mutex data_mutex;
  object_msgs::msg::ObjectArray::SharedPtr latest_objects;
  geometry_msgs::msg::PoseArray::SharedPtr latest_goals;
  std::atomic<bool> sequence_requested{false};

  // Subscribers
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

  // Service trigger
  auto trigger_srv = node->create_service<std_srvs::srv::Trigger>(
    "start_pick_place",
    [&](const std_srvs::srv::Trigger::Request::SharedPtr,
              std_srvs::srv::Trigger::Response::SharedPtr res) {
      std::lock_guard<std::mutex> lock(data_mutex);
      if (!latest_objects || !latest_goals) {
        res->success = false;
        res->message = "Waiting for perception data — objects or goals not yet received.";
        return;
      }
      sequence_requested = true;
      res->success = true;
      res->message = "Pick and place sequence started.";
    });

  // Gripper publisher
  auto gripper_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/finger_width_controller/commands", 10);

  moveit::planning_interface::MoveGroupInterface arm(node, ARM_GROUP);
  moveit::planning_interface::PlanningSceneInterface psi;

  

  // user RRTConnect as planner
  arm.setPlannerId("RRTConnect");
  arm.setMaxVelocityScalingFactor(0.3);
  arm.setMaxAccelerationScalingFactor(0.3);


  // allow for more time to plan so better solution can occur
  arm.setPlanningTime(10.0);

  // penalise path with big angular difference
  arm.setGoalJointTolerance(0.01);
  arm.setGoalOrientationTolerance(0.01);
  arm.setGoalPositionTolerance(0.005);

  std::this_thread::sleep_for(std::chrono::seconds(5));

  moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  placeGround(arm, psi);
  returnHome(arm, gripper_pub);

  // Open terminal for keypress trigger
  std::ifstream tty("/dev/tty");

  RCLCPP_INFO(LOGGER, "Ready. Press ENTER or call /start_pick_place to begin.");
  RCLCPP_INFO(LOGGER, "Press 'q' to quit.");

  while (rclcpp::ok())
  {
    // Check for keypress
    std::cout << "\nWaiting for objects... Press ENTER to start, 'q' to quit.\n>> ";
    std::string line;
    std::getline(tty, line);
    char cmd = line.empty() ? '\n' : line[0];

    if (cmd == 'q') break;

    // Also check if service triggered
    bool go = (cmd == '\n') || sequence_requested.exchange(false);

    if (!go) continue;

    // Grab latest data under lock
    object_msgs::msg::ObjectArray objects_copy;
    geometry_msgs::msg::PoseArray  goals_copy;
    {
      std::lock_guard<std::mutex> lock(data_mutex);
      if (!latest_objects || !latest_goals) {
        RCLCPP_WARN(LOGGER, "No perception data yet — publish objects first.");
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



// namespace rvt = rviz_visual_tools;

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_demo");

// const std::string ARM_GROUP     = "ur_onrobot_manipulator";
// const std::string GRIPPER_GROUP = "ur_onrobot_gripper";
// const double PREGRASP_HEIGHT    = 0.05;   // metres above object
// const double GRIPPER_OPEN       = 0.085;
// const double GRIPPER_CLOSED     = 0.01;


// // ============================================================
// // FIX 1: Wraparound correction applied to every planned trajectory.
// //
// // When MoveIt picks an IK solution where a joint is at e.g. +π but
// // the robot is currently at -π (same physical pose, opposite
// // representation), it commands a full 2π rotation.  This function
// // detects that case in the first trajectory point and shifts every
// // subsequent point by ±2π so the controller sees a near-zero delta.
// // ============================================================
// bool executeWithWrapFix(
//     moveit::planning_interface::MoveGroupInterface& arm,
//     moveit::planning_interface::MoveGroupInterface::Plan& plan)
// {
//   std::vector<double> current = arm.getCurrentJointValues();
//   auto& traj = plan.trajectory_.joint_trajectory;

//   if (!traj.points.empty())
//   {
//     auto& first = traj.points.front().positions;
//     for (size_t i = 0; i < current.size() && i < first.size(); ++i)
//     {
//       double diff = first[i] - current[i];
//       // If the planned first point is ~2π away from where we actually are,
//       // shift the whole trajectory by ±2π on that joint.
//       if (std::abs(std::abs(diff) - 2.0 * M_PI) < 0.3)
//       {
//         double offset = (diff > 0) ? -2.0 * M_PI : 2.0 * M_PI;
//         RCLCPP_WARN(LOGGER,
//           "Joint %zu wraparound detected (diff=%.4f rad) — shifting trajectory by %.4f",
//           i, diff, offset);
//         for (auto& pt : traj.points)
//           if (i < pt.positions.size())
//             pt.positions[i] += offset;
//       }
//     }
//   }

//   return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
// }


// // ============================================================
// // Startup recovery: if any joint drifted outside (-π, +π] due to
// // a previous session, send a joint-space move to bring it back
// // before attempting any Cartesian or named-target moves.
// // ============================================================
// void recoverWrappedJoints(moveit::planning_interface::MoveGroupInterface& arm)
// {
//   std::vector<double> joints = arm.getCurrentJointValues();
//   bool needs_fix = false;

//   for (size_t i = 0; i < joints.size(); ++i)
//   {
//     while (joints[i] >  M_PI) { joints[i] -= 2.0 * M_PI; needs_fix = true; }
//     while (joints[i] < -M_PI) { joints[i] += 2.0 * M_PI; needs_fix = true; }
//   }

//   if (needs_fix)
//   {
//     RCLCPP_WARN(LOGGER, "Joints outside (-π, π] detected — sending normalisation move");
//     // FIX 2 applied here too: fresh start state before planning
//     arm.setStartStateToCurrentState();
//     arm.setJointValueTarget(joints);
//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
//       executeWithWrapFix(arm, plan);
//   }
//   else
//   {
//     RCLCPP_INFO(LOGGER, "All joints within (-π, π] — no recovery needed");
//   }
// }


// // ============================================================
// // Grasp Orientation
// // ============================================================
// geometry_msgs::msg::Quaternion computeGraspOrientation(uint8_t thin_axis)
// {
//   tf2::Quaternion q;
//   switch (thin_axis)
//   {
//     case 0:
//       q.setRPY(M_PI, 0.0, 0.0);
//       break;
//     case 1:
//       q.setRPY(M_PI, 0.0, M_PI / 2.0);
//       break;
//     case 2:
//     default:
//       q.setRPY(M_PI, 0.0, 0.0);
//       break;
//   }
//   q.normalize();
//   return tf2::toMsg(q);
// }

// double computeGraspWidth(const object_msgs::msg::Object& obj)
// {
//   std::vector<double> dims = {obj.dimensions[0], obj.dimensions[1], obj.dimensions[2]};
//   dims.erase(dims.begin() + obj.thin_axis);
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
//     moveit::planning_interface::MoveGroupInterface& arm,
//     const object_msgs::msg::Object& obj,
//     const std::string& id)
// {
//   const double GRASP_MARGIN = 0.01;

//   moveit_msgs::msg::CollisionObject co;
//   co.header.frame_id = "tool0";
//   co.id = id;

//   shape_msgs::msg::SolidPrimitive box;
//   box.type = box.BOX;
//   box.dimensions = {
//     obj.dimensions[0],
//     obj.dimensions[1] - 2.0 * GRASP_MARGIN,
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
//     moveit::planning_interface::MoveGroupInterface& arm,
//     const geometry_msgs::msg::Pose& pregrasp_pose)
// {
//   // FIX 2: always sync start state before planning
//   arm.setStartStateToCurrentState();
//   arm.setPoseTarget(pregrasp_pose);
//   moveit::planning_interface::MoveGroupInterface::Plan plan;
//   if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
//   {
//     RCLCPP_ERROR(LOGGER, "Lift planning failed");
//     return false;
//   }
//   return executeWithWrapFix(arm, plan);
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
//   // FIX 2: sync start state so "home" target plans from actual current pose
//   arm.setStartStateToCurrentState();
//   arm.setNamedTarget("home");
//   moveit::planning_interface::MoveGroupInterface::Plan plan;
//   if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
//     executeWithWrapFix(arm, plan);
//   sendGripper(gripper_pub, GRIPPER_OPEN);
// }


// // ============================================================
// // Arm Motion Helpers
// // ============================================================
// bool moveTopose(
//     moveit::planning_interface::MoveGroupInterface& arm,
//     const geometry_msgs::msg::Pose& pose)
// {
//   // FIX 2: sync start state before every plan to prevent stale-state wraparound
//   arm.setStartStateToCurrentState();
//   arm.setPoseTarget(pose);
//   moveit::planning_interface::MoveGroupInterface::Plan plan;
//   if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
//   {
//     RCLCPP_ERROR(LOGGER, "Planning failed");
//     return false;
//   }
//   return executeWithWrapFix(arm, plan);
// }

// bool moveCartesianDown(
//     moveit::planning_interface::MoveGroupInterface& arm,
//     const geometry_msgs::msg::Pose& target)
// {
//   // FIX 2: Cartesian planning also needs a fresh start state
//   arm.setStartStateToCurrentState();

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
//   return executeWithWrapFix(arm, plan);
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

//   geometry_msgs::msg::Pose pregrasp = obj.pose;
//   pregrasp.position.z  += PREGRASP_HEIGHT;
//   pregrasp.orientation  = grasp_orientation;

//   geometry_msgs::msg::Pose grasp = obj.pose;
//   grasp.orientation = grasp_orientation;

//   RCLCPP_INFO(LOGGER, "Moving to pre-grasp for %s", obj_id.c_str());
//   if (!moveTopose(arm, pregrasp))             return false;

//   sendGripper(gripper_pub, GRIPPER_OPEN);

//   RCLCPP_INFO(LOGGER, "Descending to grasp");
//   if (!moveCartesianDown(arm, grasp))         return false;

//   double grasp_width = computeGraspWidth(obj);
//   RCLCPP_INFO(LOGGER, "Closing gripper to %.3fm (object width + margin)", grasp_width);
//   sendGripper(gripper_pub, grasp_width);

//   attachObject(arm, obj, obj_id);

//   RCLCPP_INFO(LOGGER, "Lifting object");
//   if (!moveCartesianDown(arm, pregrasp))      return false;

//   return true;
// }

// bool placeObject(
//     moveit::planning_interface::MoveGroupInterface& arm,
//     const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
//     moveit::planning_interface::PlanningSceneInterface& psi,
//     const geometry_msgs::msg::Pose& bin_pose,
//     const std::string& obj_id)
// {
//   geometry_msgs::msg::Pose pre_place = bin_pose;
//   pre_place.position.z += PREGRASP_HEIGHT;
//   pre_place.orientation = computeGraspOrientation(2);

//   geometry_msgs::msg::Pose place = bin_pose;
//   place.orientation = computeGraspOrientation(2);

//   RCLCPP_INFO(LOGGER, "Moving to pre-place for bin");
//   if (!moveTopose(arm, pre_place))            return false;

//   RCLCPP_INFO(LOGGER, "Descending to place");
//   if (!moveCartesianDown(arm, place))         return false;

//   sendGripper(gripper_pub, GRIPPER_OPEN);

//   detachAndRemoveObject(arm, psi, obj_id);

//   if (!moveCartesianDown(arm, pre_place))     return false;

//   return true;
// }


// object_msgs::msg::ObjectArray transformToBaseLink(
//     const object_msgs::msg::ObjectArray& object_array,
//     const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
// {
//   if (object_array.header.frame_id == "base_link")
//   {
//     RCLCPP_INFO(LOGGER, "Object poses already in base_link frame");
//     return object_array;
//   }

//   RCLCPP_INFO(LOGGER, "Transforming object poses from %s to base_link",
//     object_array.header.frame_id.c_str());

//   if (!tf_buffer->canTransform("base_link", object_array.header.frame_id, tf2::TimePointZero))
//   {
//     RCLCPP_ERROR(LOGGER, "Frame %s not in TF tree — cannot transform",
//       object_array.header.frame_id.c_str());
//     return object_array;
//   }

//   object_msgs::msg::ObjectArray transformed = object_array;

//   for (size_t i = 0; i < transformed.objects.size(); ++i)
//   {
//     geometry_msgs::msg::PoseStamped pose_in, pose_out;
//     pose_in.header.frame_id = object_array.header.frame_id;
//     pose_in.header.stamp    = object_array.header.stamp;
//     pose_in.pose            = transformed.objects[i].pose;

//     try {
//       tf_buffer->transform(pose_in, pose_out, "base_link", tf2::durationFromSec(1.0));
//       transformed.objects[i].pose            = pose_out.pose;
//       transformed.objects[i].header.frame_id = "base_link";
//     } catch (const tf2::TransformException& ex) {
//       RCLCPP_ERROR(LOGGER, "Transform failed for object %zu: %s", i, ex.what());
//       return object_array;
//     }
//   }

//   transformed.header.frame_id = "base_link";
//   return transformed;
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

//   std::vector<object_msgs::msg::Object> sorted = object_array.objects;
//   std::sort(sorted.begin(), sorted.end(),
//     [](const object_msgs::msg::Object& a, const object_msgs::msg::Object& b){
//       return a.bin_index < b.bin_index;
//     });

//   for (size_t i = 0; i < sorted.size(); ++i)
//   {
//     std::string id = "object_" + std::to_string(i);
//     addObjectCollision(psi, sorted[i], id);
//   }

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

//   std::mutex data_mutex;
//   object_msgs::msg::ObjectArray::SharedPtr latest_objects;
//   geometry_msgs::msg::PoseArray::SharedPtr latest_goals;
//   std::atomic<bool> sequence_requested{false};

//   auto tf_buffer   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
//   auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

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

//   auto gripper_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
//     "/finger_width_controller/commands", 10);

//   moveit::planning_interface::MoveGroupInterface arm(node, ARM_GROUP);
//   moveit::planning_interface::PlanningSceneInterface psi;

//   arm.setPlanningTime(10.0);
//   arm.setMaxVelocityScalingFactor(0.3);
//   arm.setMaxAccelerationScalingFactor(0.3);

//   std::this_thread::sleep_for(std::chrono::seconds(5));

//   moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link");
//   visual_tools.deleteAllMarkers();
//   visual_tools.loadRemoteControl();

//   placeGround(arm, psi);

//   // FIX 2 (startup): recover any joint that is outside (-π, π] before
//   // attempting any motion — this is what causes the hard freeze on launch
//   // when the previous session left joint 5 at e.g. +π+ε.
//   recoverWrappedJoints(arm);

//   returnHome(arm, gripper_pub);

//   std::ifstream tty("/dev/tty");

//   RCLCPP_INFO(LOGGER, "Ready. Press ENTER or call /start_pick_place to begin.");
//   RCLCPP_INFO(LOGGER, "Press 'q' to quit.");

//   while (rclcpp::ok())
//   {
//     std::cout << "\nWaiting for objects... Press ENTER to start, 'q' to quit.\n>> ";
//     std::string line;
//     std::getline(tty, line);
//     char cmd = line.empty() ? '\n' : line[0];

//     if (cmd == 'q') break;

//     bool go = (cmd == '\n') || sequence_requested.exchange(false);
//     if (!go) continue;

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

//     auto transformed = transformToBaseLink(objects_copy, tf_buffer);
//     executePickPlace(arm, gripper_pub, psi, transformed, goals_copy);
//   }

//   rclcpp::shutdown();
//   spinner.join();
//   return 0;
// }