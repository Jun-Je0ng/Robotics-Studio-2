










/////CL  1
///// Move to pose goal using MoveGroupInterface, with user prompts and visualization in RViz


#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur3e_demo_node");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ur3e_demo_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Planning group
  static const std::string PLANNING_GROUP = "ur_manipulator";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "UR3e MoveGroup Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();


  // Capture current state as start state
  RCLCPP_INFO(LOGGER, "Capturing current state...");

  // Set the start state to the current state of the robot (robot_state)
  move_group.setStartStateToCurrentState();

  // Generate target end effector pose relative to base_link
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.3;  // 30 cm forward
  target_pose.position.y = 0.1;  // 10 cm to the side
  target_pose.position.z = 0.3;  // 30 cm up

  // OR get random acheivable pose
  geometry_msgs::msg::Pose random_pose = move_group.getRandomPose("tool0")


  // generate a target pose as joint configuration
  std::vector<double> target_joint_group_positions = {0.0, -M_PI/4, 0.0, -M_PI/2, 0.0, -M_PI/4, 0.0};
  move_group.setJointValueTarget(target_joint_group_positions);


  // Visualize the target pose
  visual_tools.publishAxisLabeled(target_pose, "target_pose");
  visual_tools.trigger();

  // === PROMPT 1: Ask user to start planning ===
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start planning");
  RCLCPP_INFO(LOGGER, "Waiting for user to trigger planning...");


  // Set the target pose for the end-effector (tool0)
  move_group.setPoseTarget(target_pose, "tool0");

  // Plan
  visual_tools.publishText(text_pose, "Planning...", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  
  // 
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Check if planning succeeded
  if (success)
  {
      RCLCPP_INFO(LOGGER, "Planning succeeded!");
      
      // Visualize the planned path
      visual_tools.publishText(text_pose, "Plan found! Displaying trajectory", rvt::GREEN, rvt::XLARGE);
      visual_tools.publishTrajectoryLine(plan.trajectory_,
          move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP));
      visual_tools.trigger();

      // === PROMPT 2: Ask user to execute the plan ===
      visual_tools.prompt("Press 'next' to EXECUTE the planned motion");
      RCLCPP_INFO(LOGGER, "Waiting for user to trigger execution...");

      // Execute the plan
      visual_tools.publishText(text_pose, "Executing...", rvt::ORANGE, rvt::XLARGE);
      visual_tools.trigger();
      
      auto execute_result = move_group.execute(plan);
      
      if (execute_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
          RCLCPP_INFO(LOGGER, "Execution completed successfully!");
          visual_tools.publishText(text_pose, "Motion Complete!", rvt::GREEN, rvt::XLARGE);
      }
      else
      {
          RCLCPP_ERROR(LOGGER, "Execution failed!");
          visual_tools.publishText(text_pose, "Execution Failed!", rvt::RED, rvt::XLARGE);
      }
  }
  else
  {
      RCLCPP_ERROR(LOGGER, "Planning failed.");
      visual_tools.publishText(text_pose, "Planning Failed!", rvt::RED, rvt::XLARGE);
  }
  
  visual_tools.trigger();

  // === PROMPT 3: Final prompt before closing ===
  visual_tools.prompt("Press 'next' to end the demo");
  
  RCLCPP_INFO(LOGGER, "Demo finished.");
  
  // --- cleanup ---
  visual_tools.deleteAllMarkers();
  rclcpp::shutdown();
  return 0;
}










// Example code from MoveItCpp tutorial, adapted for UR3e robot. This code demonstrates how to use MoveItCpp to plan a motion to a target pose

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// namespace rvt = rviz_visual_tools;

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur3e_demo_node");

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("ur3e_demo_node");

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   std::thread([&executor]() { executor.spin(); }).detach();

//   // Planning group
//   static const std::string PLANNING_GROUP = "ur_manipulator";

//   moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
//   // moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
//   moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link");
//   visual_tools.deleteAllMarkers();
//   visual_tools.loadRemoteControl();

//   Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//   text_pose.translation().z() = 1.0;
//   visual_tools.publishText(text_pose, "UR3e MoveGroup Demo", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();


//   // 
//   RCLCPP_INFO(LOGGER, "Capturing current state...");
//   move_group.setStartStateToCurrentState();

//   // Generate target pose relative to base_link
//   geometry_msgs::msg::Pose target_pose;
//   target_pose.orientation.w = 1.0;
//   target_pose.position.x = 0.3;  // 30 cm forward
//   target_pose.position.y = 0.1;  // 10 cm to the side
//   target_pose.position.z = 0.3;  // 30 cm up

//   move_group.setPoseTarget(target_pose, "tool0");

//   // Plan
//   moveit::planning_interface::MoveGroupInterface::Plan plan;
//   bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//   if (success)
//   {
//       RCLCPP_INFO(LOGGER, "Planning succeeded! Executing...");
//       visual_tools.publishAxisLabeled(target_pose, "target_pose");
//       visual_tools.publishTrajectoryLine(plan.trajectory_,
//           move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP));
//       visual_tools.trigger();

//       move_group.execute(plan);
//   }
//   else
//   {
//       RCLCPP_ERROR(LOGGER, "Planning failed.");
//   }

//   RCLCPP_INFO(LOGGER, "Demo finished.");
//   // --- cleanup ---
//   rclcpp::shutdown();          // Then shutdown ROS
//   return 0;
// }


