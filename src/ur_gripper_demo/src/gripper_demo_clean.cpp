#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <nlohmann/json.hpp>

#include <iostream>
#include <fstream>
#include <thread>

using json = nlohmann::json;
namespace rvt = rviz_visual_tools;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur3e_demo");

// Planning groups
const std::string ARM_GROUP = "ur_onrobot_manipulator";
const std::string GRIPPER_GROUP = "ur_onrobot_gripper";
static std::atomic<bool> estop_requested{false};
static std::atomic<bool> estop_latched{false};





// ============================================================
// Gripper Control
// ============================================================

void sendGripper(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& pub,
    double width)
{
  if (estop_latched.load()) return;
  width = std::clamp(width, 0.0, 0.11);

  std_msgs::msg::Float64MultiArray msg;
  msg.data = {width};
  pub->publish(msg);

  RCLCPP_INFO(LOGGER, "Gripper → %.3f", width);
  for (int i = 0; i < 50; ++i) {
    if (estop_latched.load()) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  // std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

// ============================================================
// Pose Storage
// ============================================================

struct PoseData {
  geometry_msgs::msg::Pose pose;
  std::vector<double> arm_joints;
  std::vector<double> gripper_joints;
};

void savePose(
    moveit::planning_interface::MoveGroupInterface& arm,
    double gripper_width,
    std::vector<PoseData>& storage)
{
  
  PoseData p;
  p.pose = arm.getCurrentPose("tool0").pose;
  p.arm_joints = arm.getCurrentJointValues();
  p.gripper_joints = { gripper_width };

  storage.push_back(p);
  
  // Print saved pose info
  std::cout << "Saved joint values: ";
  for (double j : p.arm_joints)
    std::cout << j << " ";
  std::cout << "\n";

  

  RCLCPP_INFO(LOGGER, "Saved pose #%zu", storage.size());
}

// ============================================================
// Execution
// ============================================================

// bool executeArmJoint(
//     moveit::planning_interface::MoveGroupInterface& arm,
//     const std::vector<double>& joints)
// {
//   arm.setJointValueTarget(joints);

//   moveit::planning_interface::MoveGroupInterface::Plan plan;
//   if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
//     return false;

//   return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
// }

bool executeArmAsyncInterruptible(
  moveit::planning_interface::MoveGroupInterface& arm,
  const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  // Start async execution
  moveit::core::MoveItErrorCode ec = arm.asyncExecute(plan);
  if (ec != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "Failed to start async execution");
      return false;
  }

  rclcpp::Rate rate(100);
  auto start_time = std::chrono::steady_clock::now();

  // Approximate: poll until execution duration exceeded
  double traj_duration = 0.0;
  if (!plan.trajectory_.joint_trajectory.points.empty()) {
      traj_duration = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec
                    + plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9;
  }

  while (rclcpp::ok()) {
      if (estop_latched.load()) {
          RCLCPP_ERROR(LOGGER, "E-STOP → stopping robot NOW");
          arm.stop();
          return false;
      }

      auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
      if (elapsed >= traj_duration) {
          return true; // assume finished
      }

      rate.sleep();
  }

  return false;
}


bool executeArmJoint(
  moveit::planning_interface::MoveGroupInterface& arm,
  const std::vector<double>& joints)
{
  arm.setJointValueTarget(joints);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    return false;

  return executeArmAsyncInterruptible(arm, plan);
}

bool executeArmCartesian(
  moveit::planning_interface::MoveGroupInterface& arm,
  const geometry_msgs::msg::Pose& pose)
{
  std::vector<geometry_msgs::msg::Pose> waypoints{pose};
  moveit_msgs::msg::RobotTrajectory traj;

  double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, traj);
  if (fraction < 0.9)
    return false;

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = traj;

  return executeArmAsyncInterruptible(arm, plan);
}
// bool executeArmCartesian(
//     moveit::planning_interface::MoveGroupInterface& arm,
//     const geometry_msgs::msg::Pose& pose)
// {
//   std::vector<geometry_msgs::msg::Pose> waypoints{pose};
//   moveit_msgs::msg::RobotTrajectory traj;

//   double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, traj);
//   if (fraction < 0.9)
//     return false;

//   moveit::planning_interface::MoveGroupInterface::Plan plan;
//   plan.trajectory_ = traj;

//   return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
// }



void executeSequence(
    moveit::planning_interface::MoveGroupInterface& arm,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    const std::vector<PoseData>& storage,
    bool cartesian,
    std::ifstream& tty)
{

  if (estop_latched.load()) {
    RCLCPP_ERROR(LOGGER, "E-stop active → aborting sequence");
    return;
  }

  if (storage.empty()) {
    RCLCPP_WARN(LOGGER, "No poses saved.");
    return;
  }

  // for (size_t i = 0; i < storage.size(); ++i)
  // {

  //   if (estop_latched.load()) {
  //     RCLCPP_ERROR(LOGGER, "E-stop active → aborting sequence");
  //     return;
  //   }
  //   const auto& p = storage[i];

  //   bool success = cartesian
  //     ? executeArmCartesian(arm, p.pose)
  //     : executeArmJoint(arm, p.arm_joints);

  //   if (!success) {
  //     RCLCPP_ERROR(LOGGER, "Arm failed at step %zu", i+1);
  //     continue;
  //   }

  //   if (!p.gripper_joints.empty())
  //     sendGripper(gripper_pub, p.gripper_joints[0]);

  //   RCLCPP_INFO(LOGGER, "Step %zu done", i+1);
  // }
  for (size_t i = 0; i < storage.size(); ++i)
  {
    if (estop_latched.load()) {
      RCLCPP_ERROR(LOGGER, "E-stop active → aborting sequence");
      return;
    }

    const auto& p = storage[i];

    bool success = cartesian
      ? executeArmCartesian(arm, p.pose)
      : executeArmJoint(arm, p.arm_joints);

    if (!success) {
      RCLCPP_ERROR(LOGGER, "Execution stopped at step %zu", i+1);
      return;
    }

    if (!p.gripper_joints.empty())
      sendGripper(gripper_pub, p.gripper_joints[0]);
  }
}

// ============================================================
// File I/O
// ============================================================

void saveToFile(const std::vector<PoseData>& data)
{
  json j = json::array();

  for (const auto& p : data)
  {
    j.push_back({
      {"pose", {
        {"pos", {p.pose.position.x, p.pose.position.y, p.pose.position.z}},
        {"ori", {p.pose.orientation.x, p.pose.orientation.y,
                 p.pose.orientation.z, p.pose.orientation.w}}
      }},
      {"arm", p.arm_joints},
      {"gripper", p.gripper_joints}
    });
  }

  std::ofstream("poses.json") << j.dump(2);
}

void loadFromFile(std::vector<PoseData>& data)
{
  std::ifstream f("poses.json");
  if (!f.is_open()) return;

  json j; f >> j;
  data.clear();

  for (auto& e : j)
  {
    PoseData p;

    auto pos = e["pose"]["pos"];
    auto ori = e["pose"]["ori"];

    p.pose.position.x = pos[0];
    p.pose.position.y = pos[1];
    p.pose.position.z = pos[2];

    p.pose.orientation.x = ori[0];
    p.pose.orientation.y = ori[1];
    p.pose.orientation.z = ori[2];
    p.pose.orientation.w = ori[3];

    p.arm_joints = e["arm"].get<std::vector<double>>();
    p.gripper_joints = e["gripper"].get<std::vector<double>>();

    data.push_back(p);
  }
}

// ============================================================
// Jogging
// ============================================================

void jogArm(moveit::planning_interface::MoveGroupInterface& arm, char direction)
{}

// ============================================================
// Utility
// ============================================================

void printMenu()
{
  std::cout << "\n------------------- UR3e Demo ------------------\n"
            << "s: Save pose\n"
            << "g: Execute sequence\n"
            << "c: Toggle Cartesian\n"
            << "o: Open gripper\n"
            << "p: Close gripper\n"
            << "b: Add obstacle\n"
            << "l: Load poses\n"
            << "u: Save poses\n"
            << "j: Toggle Jog mode (not implemented)\n"
            << "h: Return Home\n"
            << "e: E-Stop\n"
            << "r: Reset after E-Stop\n"
            << "q: Quit\n>> ";
}

//  b : add box obstacle
void handleAddBox(
    moveit::planning_interface::MoveGroupInterface& move_group,
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    int& box_count)
{
  ++box_count;
  moveit_msgs::msg::CollisionObject box;
  box.header.frame_id = move_group.getPlanningFrame();
  box.id = "box_" + std::to_string(box_count);

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type       = primitive.BOX;
  primitive.dimensions = { 0.1, 0.1, 0.9 };

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x    = 0.4 + (box_count - 1) * -0.8;
  box_pose.position.y    = 0.0;
  box_pose.position.z    = 0.45;

  box.primitives.push_back(primitive);
  box.primitive_poses.push_back(box_pose);
  box.operation = box.ADD;

  planning_scene_interface.addCollisionObjects({ box });
  visual_tools.trigger();
}


// gripper_move_group.setGoalJointTolerance(0.05); // 5cm tolerance on finger_width  this  allows the robot to consider the goal reached even if the fingers are not perfectly at the target position, which can help with execution success when the gripper is near the desired open/close state but not exact ddue to gripping something small or due to wear on the gripper itself.
// we can also command it to close then get the current finger width to check if it successfully closed on an object (i.e. if the finger width is smaller than the open position but larger than the fully closed position, we know it is gripping something)

void placeGround(
    moveit::planning_interface::MoveGroupInterface& move_group,
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
    moveit_visual_tools::MoveItVisualTools& visual_tools)
{
  moveit_msgs::msg::CollisionObject ground;
  ground.header.frame_id = move_group.getPlanningFrame();
  ground.id = "ground";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type       = primitive.BOX;
  primitive.dimensions = { 2.0, 2.0, 0.1 };

  geometry_msgs::msg::Pose ground_pose;
  ground_pose.orientation.w = 1.0;
  ground_pose.position.x    = 0.0;
  ground_pose.position.y    = 0.0;
  ground_pose.position.z    = -0.1;

  ground.primitives.push_back(primitive);
  ground.primitive_poses.push_back(ground_pose);
  ground.operation = ground.ADD;

  planning_scene_interface.addCollisionObjects({ ground });
}

void returnHome(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub){
  move_group.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS){
    move_group.execute(plan);
  }

  sendGripper(gripper_pub, 0.088); // open gripper when returning home
}

void triggerEmergencyStop(
  rclcpp::Node::SharedPtr node,
  moveit::planning_interface::MoveGroupInterface& ur_move_group,
  moveit::planning_interface::MoveGroupInterface& gripper_move_group
  )
{
  estop_requested.store(true);
  estop_latched.store(true);

  RCLCPP_ERROR(LOGGER, "EMERGENCY STOP TRIGGERED");

  // Stop MoveIt execution immediately
  ur_move_group.stop();
  gripper_move_group.stop();

  // Optional: call a UR service if you have one available
  // If your stack exposes a real stop / halt service, call it here.
  // Example placeholder:
  // callTriggerService(node, "/some_stop_service");

}

void resetEmergencyStop()
{
  estop_requested.store(false);
  estop_latched.store(false);

  RCLCPP_INFO(LOGGER, "Emergency stop reset");

}

void emergencyStopMonitor(
  rclcpp::Node::SharedPtr node,
  moveit::planning_interface::MoveGroupInterface* ur_move_group,
  moveit::planning_interface::MoveGroupInterface* gripper_move_group,
  std::atomic<bool>* monitor_running)
{
  rclcpp::WallRate rate(50.0);  // 50 Hz

  while (rclcpp::ok() && monitor_running->load()) {
    if (estop_requested.load()) {
      ur_move_group->stop();
      gripper_move_group->stop();

      estop_requested.store(false);  // one-shot trigger, latched state remains
    }
    rate.sleep();
  }
}


// ============================================================
// Main
// ============================================================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ur3e_demo");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // Publisher for gripper commands
  auto gripper_pub =
    node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/finger_width_controller/commands", 10);

  moveit::planning_interface::MoveGroupInterface arm(node, ARM_GROUP);
  moveit::planning_interface::MoveGroupInterface gripper(node, GRIPPER_GROUP);

  // one planning scene covers both arm and gripper
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  std::vector<PoseData> storage;
  bool cartesian = false;
  bool jog_mode = false;
  int box_count = 0;

  double latest_gripper_width = 0.0;
  
  // Gripper width subscriber (for feedback)
  auto gripper_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/finger_width_controller/commands",
    10,
    [&latest_gripper_width](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (!msg->data.empty()) {
            latest_gripper_width = msg->data[0];
        }
  });


  // visualisation stuff:
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();


  // ground
  placeGround(arm, planning_scene_interface, visual_tools);

  std::ifstream tty("/dev/tty");



  // E stop thread:
  std::atomic<bool> estop_monitor_running{true};

  std::thread estop_thread(
      emergencyStopMonitor,
      node,
      &arm,
      &gripper,
      &estop_monitor_running);

  estop_monitor_running.store(false);
  if (estop_thread.joinable()) {
    estop_thread.join();
  }

  while (rclcpp::ok())
  {
    printMenu();

    std::string line;
    std::getline(tty, line);
    char cmd = line.empty() ? ' ' : line[0];

    switch (cmd)
    {
      case 's':
        if (estop_latched.load()) { RCLCPP_WARN(LOGGER, "System is E-stopped. Reset before continuing."); break;
        }savePose(arm, latest_gripper_width, storage); break;
      case 'g':
        if (estop_latched.load()) { 
            RCLCPP_WARN(LOGGER, "System is E-stopped. Reset before continuing."); 
            break;
        }
        // Run sequence in a detached thread
        std::thread([&](){
            executeSequence(arm, gripper_pub, storage, cartesian, tty);
        }).detach();
        break;
      case 'c': 
        if (estop_latched.load()) { RCLCPP_WARN(LOGGER, "System is E-stopped. Reset before continuing."); break;
        }cartesian = !cartesian; break;
      case 'o': 
        if (estop_latched.load()) { RCLCPP_WARN(LOGGER, "System is E-stopped. Reset before continuing."); break;
        }sendGripper(gripper_pub, 0.085); break;
      case 'p': 
        if (estop_latched.load()) { RCLCPP_WARN(LOGGER, "System is E-stopped. Reset before continuing."); break;
        }sendGripper(gripper_pub, 0.01); break;
      case 'b': 
        if (estop_latched.load()) { RCLCPP_WARN(LOGGER, "System is E-stopped. Reset before continuing."); break;
        }handleAddBox(arm, planning_scene_interface, visual_tools, box_count); break;
      case 'u': saveToFile(storage); break;
      case 'l': loadFromFile(storage); break;
      case 'j': std::cout << "Jog mode not implemented yet\n"; break;
      case 'h': 
        if (estop_latched.load()) { RCLCPP_WARN(LOGGER, "System is E-stopped. Reset before continuing."); break;
        }returnHome(arm, gripper_pub); break;
      case 'q': return 0;
      case 'e': triggerEmergencyStop(node, arm, gripper); break;
      case 'r': resetEmergencyStop(); break;
      default: std::cout << "Unknown\n";
    }
  }

  

  rclcpp::shutdown();
  spinner.join();
  return 0;
}