// // TO DO: 
// // - research how to check for self collision so that goal state via end effector pose is viable
// // - read into robot joint limits i.e. velocity, configuration limits, etc and enforce those in the planner
// // - add gripper open/close commands and visualize gripper state in RViz


#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>
#include <chrono>
#include <algorithm>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <atomic>
#include <mutex>

namespace rvt = rviz_visual_tools;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur3e_demo_node");

// *** CHANGED: updated planning group name to match ur_onrobot SRDF ***
static const std::string PLANNING_GROUP = "ur_onrobot_manipulator";
static const std::string GRIPPER_PLANNING_GROUP = "ur_onrobot_gripper";
static std::atomic<bool> estop_requested{false};
static std::atomic<bool> estop_latched{false};

void printMenu()
{
  std::cout << "\n========== UR3e + OnRobot Demo Menu ==========\n"
            << "  f  Enable  freedrive\n"
            << "  d  Disable freedrive\n"
            << "  s  Save current pose (arm + gripper)\n"
            << "  g  Execute stored pose sequence\n"
            << "  c  Toggle Cartesian / joint-space planning\n"
            << "  o  Open gripper\n"
            << "  p  Close gripper\n"
            << "  b  Add box obstacle\n"
            << "  t  Toggle Gripper Open/Close\n"
            << "  q  Quit\n"
            << "  e  Emergency Stop\n"
            << "  r  Reset Emergency Stop\n"
            << ">> ";
  std::cout.flush();
}

// helper to call ROS Services
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

// *** ADDED: helper to move gripper to a named target (e.g. "open" / "close") ***
void commandGripperWidth(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose,
    double width_m)
{
  // Clamp to a safe range for RG2-style width control
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

  // Give controller time to respond
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
}

void moveGripperOpen(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose)
{
  commandGripperWidth(gripper_pub, visual_tools, text_pose, 0.085);
}

void moveGripperClose(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose)
{
  commandGripperWidth(gripper_pub, visual_tools, text_pose, 0.0);
}

//  s : save current pose (arm + gripper)
// *** CHANGED: now saves arm and gripper joint values separately ***
void handleSavePose(
    moveit::planning_interface::MoveGroupInterface& ur_move_group,
    moveit::planning_interface::MoveGroupInterface& gripper_move_group,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose,
    std::vector<geometry_msgs::msg::Pose>& saved_poses,
    std::vector<std::vector<double>>& ur_joint_values,
    std::vector<std::vector<double>>& gripper_joint_values)
{
  geometry_msgs::msg::Pose current_pose = ur_move_group.getCurrentPose("tool0").pose;
  saved_poses.push_back(current_pose);
  ur_joint_values.push_back(ur_move_group.getCurrentJointValues());
  gripper_joint_values.push_back(gripper_move_group.getCurrentJointValues());

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
  visual_tools.trigger();
}

//  g : execute stored pose sequence
// *** CHANGED: arm and gripper planned and executed separately per pose ***
void handleExecuteSequence(
    moveit::planning_interface::MoveGroupInterface& ur_move_group,
    moveit::planning_interface::MoveGroupInterface& gripper_move_group,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose,
    const std::vector<geometry_msgs::msg::Pose>& saved_poses,
    const std::vector<std::vector<double>>& ur_joint_values,
    const std::vector<std::vector<double>>& gripper_joint_values,
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

    ur_move_group.setStartStateToCurrentState();
    ur_move_group.setGoalPositionTolerance(0.1);
    ur_move_group.setGoalOrientationTolerance(0.1);
    ur_move_group.setPlanningTime(10.0);

    if (use_cartesian)
    {
      std::vector<geometry_msgs::msg::Pose> waypoints = { saved_poses[i] };
      moveit_msgs::msg::RobotTrajectory trajectory;
      double fraction = ur_move_group.computeCartesianPath(
          waypoints, 0.01, 0.0, trajectory);

      if (fraction < 0.9) {
        RCLCPP_WARN(LOGGER, "Cartesian path only %.1f%% – skipping.", fraction * 100.0);
        continue;
      }
      moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
      cart_plan.trajectory_ = trajectory;
      ur_move_group.execute(cart_plan);

      auto arm_result = ur_move_group.execute(cart_plan);
      if (arm_result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        // GRIPPER (same as joint-space branch)
                if (!gripper_joint_values[i].empty()) {
          double target_width = gripper_joint_values[i][0];
          commandGripperWidth(gripper_pub, visual_tools, text_pose, target_width);
          visual_tools.publishText(text_pose, "Motion Complete!", rvt::GREEN, rvt::XLARGE);
        } else {
          visual_tools.publishText(text_pose, "No saved gripper width!", rvt::RED, rvt::XLARGE);
        }
      } else {
        // RCLCPP_ERROR(LOGGER, "Cartesian arm execution failed for pose #%zu", i + 1);
        visual_tools.publishText(text_pose, "Arm Execution Failed!", rvt::RED, rvt::XLARGE);
      }
    }
    else
    {
      // --- ARM ---
      ur_move_group.setJointValueTarget(ur_joint_values[i]);

      moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
      bool arm_ok = (ur_move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (arm_ok) {
        visual_tools.publishText(text_pose, "Arm Planning Success!", rvt::GREEN, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(
            arm_plan.trajectory_,
            ur_move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP));
        visual_tools.trigger();

        auto arm_result = ur_move_group.execute(arm_plan);
        // Move ARM and check success
        if (arm_result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(LOGGER, "Arm execution successful for pose #%zu", i + 1);
        } else {
          RCLCPP_ERROR(LOGGER, "Arm execution failed for pose #%zu – skipping gripper.", i + 1);
          visual_tools.publishText(text_pose, "Arm Execution Failed!", rvt::RED, rvt::XLARGE);
          continue;  // skip gripper if arm failed
        }
      } else {
        RCLCPP_ERROR(LOGGER, "Arm planning failed for pose #%zu", i + 1);
        continue;  // skip gripper if arm planning failed
      }
      
           if (!gripper_joint_values[i].empty()) {
        double target_width = gripper_joint_values[i][0];
        commandGripperWidth(gripper_pub, visual_tools, text_pose, target_width);
        RCLCPP_INFO(LOGGER, "Gripper width command sent for pose #%zu", i + 1);
        visual_tools.publishText(text_pose, "Motion Complete!", rvt::GREEN, rvt::XLARGE);
      } else {
        RCLCPP_ERROR(LOGGER, "No saved gripper width for pose #%zu", i + 1);
        visual_tools.publishText(text_pose, "No saved gripper width!", rvt::RED, rvt::XLARGE);
      }
    }

    visual_tools.trigger();
  }

  visual_tools.publishText(text_pose, "Sequence complete!", rvt::GREEN, rvt::XLARGE);
  visual_tools.trigger();
}

//  c : toggle planning mode
void handleTogglePlanningMode(
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose,
    bool& use_cartesian)
{
  use_cartesian = !use_cartesian;
  std::string mode = use_cartesian ? "Cartesian" : "Joint-space";
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
  box.header.frame_id = move_group.getPlanningFrame();
  box.id = "box_" + std::to_string(box_count);

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type       = primitive.BOX;
  primitive.dimensions = { 0.1, 0.1, 0.1 };

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x    = 0.3;
  box_pose.position.y    = 0.0 + (box_count - 1) * 0.15;
  box_pose.position.z    = 0.2;

  box.primitives.push_back(primitive);
  box.primitive_poses.push_back(box_pose);
  box.operation = box.ADD;

  planning_scene_interface.addCollisionObjects({ box });

  RCLCPP_INFO(LOGGER, "Added obstacle '%s'", box.id.c_str());
  visual_tools.publishText(text_pose, "Added " + box.id, rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
}

// void toggleGripper(
//     moveit::planning_interface::MoveGroupInterface& gripper_move_group,
//     moveit_visual_tools::MoveItVisualTools& visual_tools,
//     bool& is_open){
//   std::string target = is_open ? "close" : "open";
//   moveGripper(gripper_move_group, visual_tools, target);
//   is_open = !is_open;
// }

// void moveGripperToDefault(
//     moveit::planning_interface::MoveGroupInterface& gripper_move_group,
//     moveit_visual_tools::MoveItVisualTools& visual_tools){

//   // Move gripper to default open position at end of demo, 88mm
//   gripper_move_group.setJointValueTarget(0.088);
//   moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
//   bool gripper_ok = (gripper_move_group.plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//   if (gripper_ok) {
//     auto gripper_result = gripper_move_group.execute(gripper_plan);
//     if (gripper_result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
//       RCLCPP_INFO(LOGGER, "Gripper execution successful for default position");
//     } else {
//       RCLCPP_ERROR(LOGGER, "Gripper execution failed for default position");
//     }
//   } else {
//     RCLCPP_ERROR(LOGGER, "Gripper planning failed for going to default position");
//   }
// }


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

void triggerEmergencyStop(
    rclcpp::Node::SharedPtr node,
    moveit::planning_interface::MoveGroupInterface& ur_move_group,
    moveit::planning_interface::MoveGroupInterface& gripper_move_group,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose)
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

  visual_tools.publishText(text_pose, "EMERGENCY STOP", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();
}


void resetEmergencyStop(
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const Eigen::Isometry3d& text_pose)
{
  estop_requested.store(false);
  estop_latched.store(false);

  RCLCPP_INFO(LOGGER, "Emergency stop reset");
  visual_tools.publishText(text_pose, "E-STOP RESET", rvt::GREEN, rvt::XLARGE);
  visual_tools.trigger();
  
}

void emergencyStopMonitor(
    rclcpp::Node::SharedPtr node,
    moveit::planning_interface::MoveGroupInterface* ur_move_group,
    moveit::planning_interface::MoveGroupInterface* gripper_move_group,
    moveit_visual_tools::MoveItVisualTools* visual_tools,
    Eigen::Isometry3d text_pose,
    std::atomic<bool>* monitor_running)
{
  rclcpp::WallRate rate(50.0);  // 50 Hz

  while (rclcpp::ok() && monitor_running->load()) {
    if (estop_requested.load()) {
      ur_move_group->stop();
      gripper_move_group->stop();

      visual_tools->publishText(text_pose, "EMERGENCY STOP", rvt::RED, rvt::XLARGE);
      visual_tools->trigger();

      estop_requested.store(false);  // one-shot trigger, latched state remains
    }
    rate.sleep();
  }
}








//  main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ur3e_demo_node");

  auto gripper_pub =
    node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/finger_width_controller/commands", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  

  // *** CHANGED: updated planning group name ***
  moveit::planning_interface::MoveGroupInterface ur_move_group(node, PLANNING_GROUP);

  // *** ADDED: gripper move group ***
  moveit::planning_interface::MoveGroupInterface gripper_move_group(node, GRIPPER_PLANNING_GROUP);

  // one planning scene covers both arm and gripper
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "UR3e + OnRobot Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // state variables
  std::vector<geometry_msgs::msg::Pose> saved_poses;
  std::vector<std::vector<double>> ur_joint_values; // seperate arm joint values
  std::vector<std::vector<double>> gripper_joint_values; // seperate gripper joint values
  bool use_cartesian = false;
  int box_count = 0;
  bool gripper_is_open = true;
  bool joint_state_trajectory = true;  // if false, will use end effector pose goals instead of joint space goals for the arm

  // get current opened or closed state of the gripper by checking the current joint values of the gripper move group
//  std::vector<double> current_gripper_joints = gripper_move_group.getCurrentJointValues();
//   /// likely a width value for the gripper, so we can check if it's closer to the open or closed position

//   // print 

//   for (size_t i = 0; i < current_gripper_joints.size(); i++) {
//    std::cout << "Gripper joint value: " << current_gripper_joints[i] << std::endl;
//   }


//   double finger_width = current_gripper_joints[0];
//   RCLCPP_INFO(LOGGER, "Current gripper joint value: %.3f", finger_width);
//   // you may need to adjust these thresholds based on your specific gripper and its joint limits
//   if (finger_width < 0.02) {  // assuming fully closed position is near 0.0
//     gripper_is_open = false;
//     RCLCPP_INFO(LOGGER, "Gripper is currently CLOSED");
//   } else if (finger_width > 0.08) {  // assuming fully open position is near 0.1
//     gripper_is_open = true;
//     RCLCPP_INFO(LOGGER, "Gripper is currently OPEN");
//   } else {
//     RCLCPP_WARN(LOGGER, "Gripper is in an UNKNOWN state (finger width: %.3f)", finger_width);
//   }



  std::ifstream tty("/dev/tty");
  if (!tty.is_open()) {
    RCLCPP_FATAL(LOGGER, "Could not open /dev/tty - cannot read keyboard input.");
    rclcpp::shutdown();
    return 1;
  }

  placeGround(ur_move_group, planning_scene_interface, visual_tools);

  bool running = true;
  while (running && rclcpp::ok())
  {
    printMenu();

    std::string line;
    while (line.empty()) {
      std::cout << ">> ";
      std::cout.flush();
      if (!std::getline(tty, line)) {
        RCLCPP_WARN(LOGGER, "Error reading Line");
        break;
      }
    }
    char cmd = line[0];

    switch (cmd)
    {
      case 'f':
        if (estop_latched.load()) {
            RCLCPP_WARN(LOGGER, "System is E-stopped. Reset before continuing.");
            break;
        }
        handleEnableFreedrive(node, visual_tools, text_pose);
        break;
      case 'd':
              if (estop_latched.load()) {
          RCLCPP_WARN(LOGGER, "System is E-stopped. Reset before continuing.");
          break;
        }
        handleDisableFreedrive(node, visual_tools, text_pose);
        break;
      case 's':
              if (estop_latched.load()) {
          RCLCPP_WARN(LOGGER, "System is E-stopped. Reset before continuing.");
          break;
        }
        // *** CHANGED FROM DEMO: pass both move groups and separate joint value vectors ***
        handleSavePose(ur_move_group, gripper_move_group, visual_tools, text_pose,
          saved_poses, ur_joint_values, gripper_joint_values);
        break;
      case 'g':
              if (estop_latched.load()) {
          RCLCPP_WARN(LOGGER, "System is E-stopped. Reset before continuing.");
          break;
        }
        // *** CHANGED: pass both move groups and separate joint value vectors ***
       handleExecuteSequence(ur_move_group, gripper_move_group, gripper_pub, visual_tools, text_pose,
          saved_poses, ur_joint_values, gripper_joint_values, use_cartesian);
        break;
      case 'c':
              if (estop_latched.load()) {
          RCLCPP_WARN(LOGGER, "System is E-stopped. Reset before continuing.");
          break;
        }
        handleTogglePlanningMode(visual_tools, text_pose, use_cartesian);
        break;
      case 'o':
              if (estop_latched.load()) {
          RCLCPP_WARN(LOGGER, "System is E-stopped. Reset before continuing.");
          break;
        }
        moveGripperOpen(gripper_pub, visual_tools, text_pose);
        break;
      case 'p':
              if (estop_latched.load()) {
          RCLCPP_WARN(LOGGER, "System is E-stopped. Reset before continuing.");
          break;
        }
        moveGripperClose(gripper_pub, visual_tools, text_pose);
        break;
      case 'b':
          if (estop_latched.load()) {
      RCLCPP_WARN(LOGGER, "System is E-stopped. Reset before continuing.");
      break;
      }
        handleAddBox(ur_move_group, planning_scene_interface, visual_tools, text_pose, box_count);
        break;
      case 'q':
            if (estop_latched.load()) {
        RCLCPP_WARN(LOGGER, "System is E-stopped. Reset before continuing.");
        break;
      }
        RCLCPP_INFO(LOGGER, "Quitting demo.");
        running = false;
        break;
      // case 't':
      //   toggleGripper(gripper_move_group, visual_tools, gripper_is_open);
      //   break;

      case 'e':
        triggerEmergencyStop(node, ur_move_group, gripper_move_group, visual_tools, text_pose);
        break;
      case 'r':
        resetEmergencyStop(visual_tools, text_pose);

        break;


      default:
        std::cout << "Unknown command '" << cmd << "' - try again.\n";
        break;
    }
  }

  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  std::atomic<bool> estop_monitor_running{true};

  std::thread estop_thread(
      emergencyStopMonitor,
      node,
      &ur_move_group,
      &gripper_move_group,
      &visual_tools,
      text_pose,
      &estop_monitor_running);

  estop_monitor_running.store(false);
  if (estop_thread.joinable()) {
    estop_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}

// TO DO:
// - research how to check for self collision so that goal state via end effector pose is viable
// - read into robot joint limits i.e. velocity, configuration limits, etc and enforce those in the planner
// - verify "open" and "close" named targets exist in ur_onrobot SRDF for the gripper group