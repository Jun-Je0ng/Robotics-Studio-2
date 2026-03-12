#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <std_msgs/msg/float64_multi_array.hpp> 

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

namespace rvt = rviz_visual_tools;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur3e_demo_node");

void printMenu()
{
  std::cout << "\n========== UR3e Demo Menu ==========\n"
            << "  f  →  Enable  freedrive\n"
            << "  d  →  Disable freedrive\n"
            << "  s  →  Save current pose\n"
            << "  g  →  Execute stored pose sequence\n"
            << "  c  →  Toggle Cartesian / joint-space planning\n"
            << "  b  →  Add box obstacle\n"
            << "  q  →  Quit\n"
            << "  o  →  Open gripper\n"
            << "  p  →  Close gripper\n"
            << "  w  →  Set custom gripper width\n"
            << ">> ";
  std::cout.flush();
}


// helper to call ROS SerVices
bool callTriggerService(rclcpp::Node::SharedPtr node, const std::string& service_name)
{
  auto client = node->create_client<std_srvs::srv::Trigger>(service_name);
  if (!client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(LOGGER, "Service '%s' not available.", service_name.c_str());
    return false;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future  = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Service call to '%s' failed.", service_name.c_str());
    return false;
  }
  auto result = future.get();
  RCLCPP_INFO(LOGGER, "Service '%s': %s", service_name.c_str(), result->message.c_str());
  return result->success;
}

//  f : enable freedrive 
void handleEnableFreedrive(
    rclcpp::Node::SharedPtr node,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose)
{
  RCLCPP_INFO(LOGGER, "Enabling freedrive...");
  visual_tools.publishText(text_pose, "Freedrive ON", rvt::GREEN, rvt::XLARGE);
  visual_tools.trigger();
  callTriggerService(node, "/ur_hardware_interface/start_freedrive");
}

//  d : disable freedrive 
void handleDisableFreedrive(
    rclcpp::Node::SharedPtr node,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose)
{
  RCLCPP_INFO(LOGGER, "Disabling freedrive...");
  visual_tools.publishText(text_pose, "Freedrive OFF", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();
  callTriggerService(node, "/ur_hardware_interface/stop_freedrive");
}

//  s : save current pose 
void handleSavePose(
    moveit::planning_interface::MoveGroupInterface& move_group,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose,
    std::vector<geometry_msgs::msg::Pose>& saved_poses,
    std::vector<std::vector<double>>& joint_values)
{
  geometry_msgs::msg::Pose current_pose =
      move_group.getCurrentPose("tool0").pose;
  saved_poses.push_back(current_pose);
  joint_values.push_back(move_group.getCurrentJointValues());

  RCLCPP_INFO(LOGGER,
      "Pose #%zu saved  [x=%.3f  y=%.3f  z=%.3f]",
      saved_poses.size(),
      current_pose.position.x,
      current_pose.position.y,
      current_pose.position.z);

  std::string label = "pose_" + std::to_string(saved_poses.size());
  visual_tools.publishAxisLabeled(current_pose, label);
  visual_tools.publishText(
      text_pose,
      "Saved pose #" + std::to_string(saved_poses.size()),
      rvt::WHITE, rvt::XLARGE);

  visual_tools.publishText(
      text_pose,
      "Saved joint config #" + std::to_string(joint_values.size()),
      rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
}

//  g : execute stored pose sequence ─
void handleExecuteSequence(
    moveit::planning_interface::MoveGroupInterface& move_group,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose,
    const std::vector<geometry_msgs::msg::Pose>& saved_poses,
    const std::vector<std::vector<double>>& joint_values,
    const std::string& PLANNING_GROUP,
    bool use_cartesian)
{
  if (saved_poses.empty()) {
    RCLCPP_WARN(LOGGER, "No poses saved yet – use 's' first.");
    return;
  }

  RCLCPP_INFO(LOGGER, "Executing %zu saved pose(s)...", saved_poses.size());

  for (size_t i = 0; i < saved_poses.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "Moving to pose #%zu ...", i + 1);
    visual_tools.publishText(
        text_pose,
        "Going to pose #" + std::to_string(i + 1),
        rvt::ORANGE, rvt::XLARGE);
    visual_tools.trigger();
    
      
    move_group.setStartStateToCurrentState();

    // Relax tolerances to avoid false path violations
    move_group.setGoalPositionTolerance(0.1);       // 10 cm
    move_group.setGoalOrientationTolerance(0.1);     // ~5.7 degrees
    move_group.setPlanningTime(10.0);                 // give planner more time

    if (use_cartesian)
    {

      // TODO: test cartesian movements. check for joint limits and self collision
      std::vector<geometry_msgs::msg::Pose> waypoints = { saved_poses[i] };
      moveit_msgs::msg::RobotTrajectory trajectory;
      double fraction = move_group.computeCartesianPath(
          waypoints, 0.01, 0.0, trajectory);

      if (fraction < 0.9) {
        RCLCPP_WARN(LOGGER, "Cartesian path only %.1f%% – skipping.", fraction * 100.0);
        continue;
      }
      moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
      cart_plan.trajectory_ = trajectory;
      move_group.execute(cart_plan);
    }
    else
    {
      // move_group.setPoseTarget(saved_poses[i], "tool0");  // fails due to tolances, so for now we will try random pose

      // getRandonPose gets a random pose that is acheivable 
      // move_group.setPoseTarget(move_group.getRandomPose(), "tool0");


      move_group.setJointValueTarget(joint_values[i]);  // use joint values instead of pose target to bypass IK tolerances

      // Get a planning scene monitor and check for collisions at the target pose before planning
      // moveit::core::RobotStatePtr goal_state = move_group.getCurrentState();
      // bool ik_found = goal_state->setFromIK(jmg, your_target_pose.pose, 10, 0.1);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool ok = (move_group.plan(plan) ==
           moveit::core::MoveItErrorCode::SUCCESS);
      if (ok) {
        visual_tools.publishText(text_pose, "Planning Success!", rvt::GREEN, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(
            plan.trajectory_,
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP));
        visual_tools.trigger();
        // move_group.execute(plan);


        // execute and check if execution was successful
        auto execute_result = move_group.execute(plan);

        if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(LOGGER, "Execution completed successfully!");
            visual_tools.publishText(text_pose, "Motion Complete!", rvt::GREEN, rvt::XLARGE);
        }
        else
        {
            RCLCPP_ERROR(LOGGER, "Execution failed!");
            visual_tools.publishText(text_pose, "Execution Failed!", rvt::RED, rvt::XLARGE);
        }
      } else {
        RCLCPP_ERROR(LOGGER, "Planning failed for pose #%zu", i + 1);
      }
    }
  }

  visual_tools.publishText(text_pose, "Sequence complete!", rvt::GREEN, rvt::XLARGE);
  visual_tools.trigger();
}

// c : toggle planning mode ─
void handleTogglePlanningMode(
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose,
    bool& use_cartesian)
{
  use_cartesian = !use_cartesian;
  std::string mode = use_cartesian ? "Cartesian" : "Joint-space"; // enumerating based on bool

  // log
  RCLCPP_INFO(LOGGER, "Planning mode → %s", mode.c_str());
  visual_tools.publishText(text_pose, "Mode: " + mode, rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
}

//  b : add box obstacle 
void handleAddBox(
    moveit::planning_interface::MoveGroupInterface& move_group,
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose,
    int& box_count)
{
  ++box_count;
  moveit_msgs::msg::CollisionObject box;

  // get the frame/world in which planning is being performed (OR get the frame of the robot)
  box.header.frame_id = move_group.getPlanningFrame();
  box.id = "box_" + std::to_string(box_count);

  // defining shape
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type       = primitive.BOX;
  primitive.dimensions = { 0.1, 0.1, 0.1 };


  // pose
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x    = 0.3;
  box_pose.position.y    = 0.0 + (box_count - 1) * 0.15;
  box_pose.position.z    = 0.2;

  // adding shhape and pose and what this box will do to the simulation world. (i.e. add = adds, remove = removes anything that is in that area?? move = moves the box to a new location, append..) 
  box.primitives.push_back(primitive);
  box.primitive_poses.push_back(box_pose);
  box.operation = box.ADD;

  planning_scene_interface.addCollisionObjects({ box });

  RCLCPP_INFO(LOGGER, "Added obstacle '%s'", box.id.c_str());
  visual_tools.publishText(
      text_pose, "Added " + box.id, rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
}


void placeGround(
  moveit::planning_interface::MoveGroupInterface& move_group,
  moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
  moveit_visual_tools::MoveItVisualTools& visual_tools){

  moveit_msgs::msg::CollisionObject ground;
  ground.header.frame_id = move_group.getPlanningFrame();
  ground.id = "ground";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = { 2.0, 2.0, 0.1 };   // 2m x 2m x 10cm box

  geometry_msgs::msg::Pose ground_pose;
  ground_pose.orientation.w = 1.0;
  ground_pose.position.x    = 0.0;
  ground_pose.position.y    = 0.0;
  ground_pose.position.z    = -0.05;  // half the height below the origin

  ground.primitives.push_back(primitive);
  ground.primitive_poses.push_back(ground_pose);
  ground.operation = ground.ADD;

  planning_scene_interface.addCollisionObjects({ ground });
}

void publishGripperWidth(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose,
    double width_m)
{
  // Clamp to a safe range.
  // Adjust if your RG2 setup supports a different practical opening range.
  width_m = std::max(0.0, std::min(0.11, width_m));

  std_msgs::msg::Float64MultiArray msg;
  msg.data = {width_m};
  gripper_pub->publish(msg);

  RCLCPP_INFO(LOGGER, "Published gripper width command: %.3f m", width_m);

  visual_tools.publishText(
      text_pose,
      "Gripper width: " + std::to_string(width_m) + " m",
      rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
}


void handleOpenGripper(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose)
{
  RCLCPP_INFO(LOGGER, "Opening gripper...");
  publishGripperWidth(gripper_pub, visual_tools, text_pose, 0.085);  // example open width
}

void handleCloseGripper(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose)
{
  RCLCPP_INFO(LOGGER, "Closing gripper...");
  publishGripperWidth(gripper_pub, visual_tools, text_pose, 0.0);  // closed
}

void handleCustomGripperWidth(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose,
    std::ifstream& tty)
{
  std::cout << "Enter gripper width in metres (e.g. 0.05): ";
  std::cout.flush();

  std::string line;
  if (!std::getline(tty, line) || line.empty()) {
    RCLCPP_WARN(LOGGER, "No width entered.");
    return;
  }

  try {
    double width = std::stod(line);
    publishGripperWidth(gripper_pub, visual_tools, text_pose, width);
  } catch (const std::exception&) {
    RCLCPP_WARN(LOGGER, "Invalid width input.");
  }
}






//  main 
int main(int argc, char** argv)
{
  // typical ROS2 node setup
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ur3e_demo_node");

  // gripper command publisher
  auto gripper_pub =
      node->create_publisher<std_msgs::msg::Float64MultiArray>(
          "/finger_width_controller/commands", 10);

  // a single-threaded executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // spin the executor in a separate thread so that we can call ROS services
  std::thread([&executor]() { executor.spin(); }).detach();

  // name of the planning group we want to control
  static const std::string PLANNING_GROUP = "ur_manipulator";

  // MoveGroupInterface is the primary interface to the planner
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // Planning scene interface
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // RViz visualisation tool
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link");

  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // test text visualisation
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "UR3e Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // state variables for the demo
  std::vector<geometry_msgs::msg::Pose> saved_poses;
  std::vector<std::vector<double>> joint_values;
  bool use_cartesian = false;
  int box_count = 0;

  // open /dev/tty so input works even under ros2 launch
  std::ifstream tty("/dev/tty");
  if (!tty.is_open()) {
    RCLCPP_FATAL(LOGGER, "Could not open /dev/tty - cannot read keyboard input.");
    rclcpp::shutdown();
    return 1;
  }

  // place ground plane
  placeGround(move_group, planning_scene_interface, visual_tools);

  // menu loop
  bool running = true;
  while (running && rclcpp::ok())
  {
    printMenu();

    std::string line;
    while (line.empty()) {
      std::cout << ">> ";
      std::cout.flush();
      if (!std::getline(tty, line)) {
        RCLCPP_WARN(LOGGER, "Error reading line");
        break;
      }
    }

    if (line.empty()) {
      continue;
    }

    char cmd = line[0];

    switch (cmd)
    {
      case 'f':
        handleEnableFreedrive(node, visual_tools, text_pose);
        break;

      case 'd':
        handleDisableFreedrive(node, visual_tools, text_pose);
        break;

      case 's':
        handleSavePose(move_group, visual_tools, text_pose, saved_poses, joint_values);
        break;

      case 'g':
        handleExecuteSequence(
            move_group, visual_tools, text_pose, saved_poses,
            joint_values, PLANNING_GROUP, use_cartesian);
        break;

      case 'c':
        handleTogglePlanningMode(visual_tools, text_pose, use_cartesian);
        break;

      case 'b':
        handleAddBox(move_group, planning_scene_interface, visual_tools, text_pose, box_count);
        break;

      case 'o':
        handleOpenGripper(gripper_pub, visual_tools, text_pose);
        break;

      case 'p':
        handleCloseGripper(gripper_pub, visual_tools, text_pose);
        break;

      case 'w':
        handleCustomGripperWidth(gripper_pub, visual_tools, text_pose, tty);
        break;

      case 'q':
        RCLCPP_INFO(LOGGER, "Quitting demo.");
        running = false;
        break;

      default:
        std::cout << "Unknown command '" << cmd << "' - try again.\n";
        break;
    }
  }

  // cleanup
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  rclcpp::shutdown();
  return 0;
}


// TO DO: 
// - research how to check for self collision so that goal state via end effector pose is viable
// - read into robot joint limits i.e. velocity, configuration limits, etc and enforce those in the planner
// - add gripper open/close commands and visualize gripper state in RViz