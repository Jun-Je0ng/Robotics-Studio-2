/**
 * @file GUI.cpp
 * @brief ROS2 node for UR3e robot control with GUI interface
 *
 * This node provides a service-based interface for controlling a UR3e robot
 * with an OnRobot gripper. It supports pose saving/loading, sequence execution,
 * gripper control, obstacle management, and emergency stop functionality.
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nlohmann/json.hpp>

#include <fstream>
#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <vector>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <string>
#include <memory>
#include <functional>

using json = nlohmann::json;
namespace rvt = rviz_visual_tools;

// =============================================================================
// Constants
// =============================================================================

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur3e_demo");

// MoveIt groups
const std::string ARM_GROUP = "ur_onrobot_manipulator";
const std::string GRIPPER_GROUP = "ur_onrobot_gripper";

// ROS2 Topics
const std::string GRIPPER_COMMAND_TOPIC = "/finger_width_controller/commands";
const std::string GUI_COMMAND_TOPIC = "/gui_command";
const std::string SYSTEM_STATUS_TOPIC = "/system_status";
const std::string ROBOT_STATE_TOPIC = "/robot_state";
const std::string EVENT_LOG_TOPIC = "/event_log";
const std::string MOTION_COMMAND_TOPIC = "/motion_system/command";
const std::string MOTION_STATUS_TOPIC = "/motion_system/status";
const std::string MOTION_OBJECT_TOPIC = "/motion_system/current_object";

// Gripper limits
const double GRIPPER_OPEN_WIDTH = 0.085;
const double GRIPPER_CLOSED_WIDTH = 0.01;
const double GRIPPER_MAX_WIDTH = 0.11;
const double GRIPPER_MIN_WIDTH = 0.0;

// Planning parameters
const double PLANNING_TIME = 10.0;
const int NUM_PLANNING_ATTEMPTS = 5;
const double MAX_VELOCITY_SCALING = 0.1;
const double MAX_ACCELERATION_SCALING = 0.1;
const double GOAL_JOINT_TOLERANCE = 0.01;
const double GOAL_POSITION_TOLERANCE = 0.01;
const double GOAL_ORIENTATION_TOLERANCE = 0.05;
const double CARTESIAN_PATH_STEP_SIZE = 0.01;
const double MIN_CARTESIAN_FRACTION = 0.9;

// File paths
const std::string POSES_FILE = "poses.json";

// =============================================================================
// Global State (Thread-safe)
// =============================================================================

static std::atomic<bool> estop_requested{false};
static std::atomic<bool> estop_latched{false};
static std::atomic<bool> sequence_running{false};

// =============================================================================
// Data Structures
// =============================================================================

/**
 * @brief Stores pose data including position, arm joints, and gripper state
 */
struct PoseData
{
  geometry_msgs::msg::Pose pose;
  std::vector<double> arm_joints;
  std::vector<double> gripper_joints;
};

// =============================================================================
// UR3e Demo Node Class
// =============================================================================

/**
 * @brief Main ROS2 node for UR3e robot control and GUI interface
 *
 * This class provides a complete interface for controlling a UR3e robot with
 * OnRobot gripper, including pose management, sequence execution, and safety features.
 */
class UR3EDemoNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  UR3EDemoNode() : Node("ur3e_demo") {}

  /**
   * @brief Initialize the node and all interfaces
   */
  void init()
  {
    initializeMoveItInterfaces();
    initializePublishers();
    initializeSubscribers();
    initializeVisualTools();
    setupEnvironment();

    // Initialize status update timer (1 Hz)
    status_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&UR3EDemoNode::updateStatusDisplay, this));

    publishText(system_status_pub_, "Stopped");
    publishText(robot_state_pub_, "Idle");
    logEvent("System initialised");

    RCLCPP_INFO(LOGGER, "UR3e demo node ready");
    RCLCPP_INFO(LOGGER, "Listening for GUI commands on %s", GUI_COMMAND_TOPIC.c_str());
  }

private:
// =============================================================================
// Member Variables
// =============================================================================

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_log_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motion_cmd_pub_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gui_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasp_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_object_sub_;

  // MoveIt interfaces
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

  // State variables
  std::vector<PoseData> storage_;
  bool cartesian_mode_ = false;
  int box_count_ = 0;
  double latest_gripper_width_ = 0.0;
  bool latest_grasp_status_ = false;
  std::mutex action_mutex_;
  rclcpp::TimerBase::SharedPtr status_timer_;

// =============================================================================
// Initialization Methods
// =============================================================================

  /**
   * @brief Initialize MoveIt interfaces for arm and gripper
   */
  void initializeMoveItInterfaces()
  {
    arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), ARM_GROUP);

    gripper_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), GRIPPER_GROUP);
  }

  /**
   * @brief Initialize all ROS2 publishers
   */
  void initializePublishers()
  {
    gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      GRIPPER_COMMAND_TOPIC, 10);

    system_status_pub_ = this->create_publisher<std_msgs::msg::String>(
      SYSTEM_STATUS_TOPIC, 10);

    robot_state_pub_ = this->create_publisher<std_msgs::msg::String>(
      ROBOT_STATE_TOPIC, 10);

    event_log_pub_ = this->create_publisher<std_msgs::msg::String>(
      EVENT_LOG_TOPIC, 10);

    motion_cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
      MOTION_COMMAND_TOPIC, 10);
  }

  /**
   * @brief Initialize all ROS2 subscribers
   */
  void initializeSubscribers()
  {
    gui_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      GUI_COMMAND_TOPIC, 10,
      std::bind(&UR3EDemoNode::guiCommandCallback, this, std::placeholders::_1));

    gripper_feedback_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      GRIPPER_COMMAND_TOPIC, 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
      {
        if (!msg->data.empty()) {
          latest_gripper_width_ = msg->data[0];
        }
      });

    grasp_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/gripper_grasp_status", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        latest_grasp_status_ = msg->data;
      });

    motion_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      MOTION_STATUS_TOPIC, 10,
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        logEvent(msg->data);
      });

    motion_object_sub_ = this->create_subscription<std_msgs::msg::String>(
      MOTION_OBJECT_TOPIC, 10,
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        publishText(robot_state_pub_, msg->data);
      });
  }

  /**
   * @brief Initialize visual tools for RViz
   */
  void initializeVisualTools()
  {
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      shared_from_this(), "base_link");
    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();
  }

  /**
   * @brief Setup the robot environment (collision objects, etc.)
   */
  void setupEnvironment()
  {
    addTrolleyMesh();
  }

// =============================================================================
// Utility Methods
// =============================================================================

  /**
   * @brief Publish a text message to a string topic
   * @param publisher The publisher to use
   * @param text The text to publish
   */
  void publishText(
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& publisher,
    const std::string& text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    publisher->publish(msg);
  }

  /**
   * @brief Log an event to the event log topic
   * @param text The event message
   */
  void logEvent(const std::string& text)
  {
    publishText(event_log_pub_, text);
  }

  /**
   * @brief Update the status display with current gripper information
   */
  void updateStatusDisplay()
  {
    // Update robot state with grasp status
    std::string robot_state = "Idle";
    if (latest_grasp_status_) {
      robot_state += " (Gripping)";
    } else {
      robot_state += " (No Grip)";
    }
    publishText(robot_state_pub_, robot_state);
  }

  /**
   * @brief Check if emergency stop is active and handle accordingly
   * @param fail_message Message to log if E-stop is active
   * @return true if E-stop is active (operation should be blocked)
   */
  bool checkEstopBlock(const std::string& fail_message)
  {
    if (estop_latched.load()) {
      publishText(system_status_pub_, "Failed");
      publishText(robot_state_pub_, "Idle");
      logEvent(fail_message);
      return true;
    }
    return false;
  }

  /**
   * @brief Send gripper width command
   * @param width The desired gripper width (clamped to valid range)
   */
  void sendGripper(double width)
  {
    if (estop_latched.load()) return;

    width = std::clamp(width, GRIPPER_MIN_WIDTH, GRIPPER_MAX_WIDTH);

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {width};
    gripper_pub_->publish(msg);

    RCLCPP_INFO(LOGGER, "Gripper -> %.3f", width);

    // Wait for gripper movement to complete (with E-stop checking)
    for (int i = 0; i < 50; ++i) {
      if (estop_latched.load()) return;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

// =============================================================================
// Pose Management Methods
// =============================================================================

  /**
   * @brief Save the current robot pose (position, joints, gripper state)
   */
  void savePose()
  {
    PoseData pose_data;
    pose_data.pose = arm_->getCurrentPose("tool0").pose;
    pose_data.arm_joints = arm_->getCurrentJointValues();
    pose_data.gripper_joints = {latest_gripper_width_};
    storage_.push_back(pose_data);

    RCLCPP_INFO(LOGGER, "Saved pose #%zu", storage_.size());
  }

  /**
   * @brief Save all stored poses to JSON file
   */
  void savePosesToFile()
  {
    json json_data = json::array();

    for (const auto& pose_data : storage_) {
      json_data.push_back({
        {"pose", {
          {"position", {pose_data.pose.position.x, pose_data.pose.position.y, pose_data.pose.position.z}},
          {"orientation", {pose_data.pose.orientation.x, pose_data.pose.orientation.y,
                          pose_data.pose.orientation.z, pose_data.pose.orientation.w}}
        }},
        {"arm_joints", pose_data.arm_joints},
        {"gripper_joints", pose_data.gripper_joints}
      });
    }

    std::ofstream file(POSES_FILE);
    file << json_data.dump(2);
  }

  /**
   * @brief Load poses from JSON file
   */
  void loadPosesFromFile()
  {
    std::ifstream file(POSES_FILE);
    if (!file.is_open()) {
      logEvent("Failed: " + POSES_FILE + " not found");
      return;
    }

    json json_data;
    file >> json_data;
    storage_.clear();

    for (const auto& item : json_data) {
      PoseData pose_data;

      // Parse pose
      const auto& pose_json = item["pose"];
      const auto& pos = pose_json["position"];
      const auto& ori = pose_json["orientation"];

      pose_data.pose.position.x = pos[0];
      pose_data.pose.position.y = pos[1];
      pose_data.pose.position.z = pos[2];
      pose_data.pose.orientation.x = ori[0];
      pose_data.pose.orientation.y = ori[1];
      pose_data.pose.orientation.z = ori[2];
      pose_data.pose.orientation.w = ori[3];

      // Parse joints
      pose_data.arm_joints = item["arm_joints"].get<std::vector<double>>();
      pose_data.gripper_joints = item["gripper_joints"].get<std::vector<double>>();

      storage_.push_back(pose_data);
    }
  }

// =============================================================================
// Motion Planning Methods
// =============================================================================

  /**
   * @brief Execute a planned trajectory with E-stop interrupt capability
   * @param plan The MoveIt plan to execute
   * @return true if execution completed successfully
   */
  bool executeArmAsyncInterruptible(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    const auto execution_result = arm_->asyncExecute(plan);
    if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "Failed to start async execution");
      return false;
    }

    rclcpp::Rate rate(100);
    const auto start_time = std::chrono::steady_clock::now();

    // Calculate trajectory duration
    double trajectory_duration = 0.0;
    if (!plan.trajectory_.joint_trajectory.points.empty()) {
      const auto& last_point = plan.trajectory_.joint_trajectory.points.back();
      trajectory_duration = last_point.time_from_start.sec +
                           last_point.time_from_start.nanosec * 1e-9;
    }

    // Monitor execution with E-stop checking
    while (rclcpp::ok()) {
      if (estop_latched.load()) {
        RCLCPP_ERROR(LOGGER, "E-STOP -> stopping robot now");
        arm_->stop();
        return false;
      }

      const auto elapsed_time =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();

      if (elapsed_time >= trajectory_duration) {
        return true;  // Execution completed
      }

      rate.sleep();
    }

    return false;
  }

  /**
   * @brief Execute arm movement to specified joint positions
   * @param joints Target joint positions
   * @return true if planning and execution succeeded
   */
  bool executeArmJoint(const std::vector<double>& joints)
  {
    arm_->setJointValueTarget(joints);

    // Configure planning parameters
    arm_->setPlanningTime(PLANNING_TIME);
    arm_->setNumPlanningAttempts(NUM_PLANNING_ATTEMPTS);
    arm_->setMaxVelocityScalingFactor(MAX_VELOCITY_SCALING);
    arm_->setMaxAccelerationScalingFactor(MAX_ACCELERATION_SCALING);
    arm_->setGoalJointTolerance(GOAL_JOINT_TOLERANCE);
    arm_->setGoalPositionTolerance(GOAL_POSITION_TOLERANCE);
    arm_->setGoalOrientationTolerance(GOAL_ORIENTATION_TOLERANCE);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      return false;
    }

    return executeArmAsyncInterruptible(plan);
  }

  /**
   * @brief Execute arm movement to specified Cartesian pose
   * @param pose Target pose
   * @return true if planning and execution succeeded
   */
  bool executeArmCartesian(const geometry_msgs::msg::Pose& pose)
  {
    const std::vector<geometry_msgs::msg::Pose> waypoints = {pose};
    moveit_msgs::msg::RobotTrajectory trajectory;

    const double path_fraction = arm_->computeCartesianPath(
      waypoints, CARTESIAN_PATH_STEP_SIZE, 0.0, trajectory);

    if (path_fraction < MIN_CARTESIAN_FRACTION) {
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    return executeArmAsyncInterruptible(plan);
  }

  /**
   * @brief Execute the stored sequence of poses
   */
  void executeSequence()
  {
    if (sequence_running.exchange(true)) {
      logEvent("Failed: sequence already running");
      return;
    }

    // Automatic cleanup of sequence_running flag
    const auto sequence_guard = std::unique_ptr<void, std::function<void(void*)>>(
      nullptr, [](void*) { sequence_running.store(false); });

    if (storage_.empty()) {
      publishText(system_status_pub_, "Failed");
      publishText(robot_state_pub_, "Idle");
      logEvent("Failed: no saved poses");
      return;
    }

    publishText(system_status_pub_, "Running");
    logEvent("Execution started");

    for (size_t i = 0; i < storage_.size(); ++i) {
      if (estop_latched.load()) {
        publishText(system_status_pub_, "Failed");
        publishText(robot_state_pub_, "Idle");
        logEvent("Failed: execution interrupted by E-stop");
        return;
      }

      publishText(robot_state_pub_, "Picking");
      logEvent("Picking item");

      // Configure planning parameters for each pose
      arm_->setStartStateToCurrentState();
      arm_->setPlanningTime(PLANNING_TIME);
      arm_->setNumPlanningAttempts(NUM_PLANNING_ATTEMPTS);
      arm_->setMaxVelocityScalingFactor(MAX_VELOCITY_SCALING);
      arm_->setMaxAccelerationScalingFactor(MAX_ACCELERATION_SCALING);
      arm_->setGoalJointTolerance(GOAL_JOINT_TOLERANCE);
      arm_->setGoalPositionTolerance(GOAL_POSITION_TOLERANCE);
      arm_->setGoalOrientationTolerance(GOAL_ORIENTATION_TOLERANCE);

      const bool arm_success = cartesian_mode_ ?
        executeArmCartesian(storage_[i].pose) :
        executeArmJoint(storage_[i].arm_joints);

      if (!arm_success) {
        publishText(system_status_pub_, "Failed");
        publishText(robot_state_pub_, "Idle");
        logEvent("Failed: arm planning or execution failed");
        continue;
      }

      logEvent("Picked");
      publishText(robot_state_pub_, "Placing");
      logEvent("Placing item");

      if (!storage_[i].gripper_joints.empty()) {
        sendGripper(storage_[i].gripper_joints[0]);
      } else {
        publishText(system_status_pub_, "Failed");
        publishText(robot_state_pub_, "Idle");
        logEvent("Failed: no saved gripper width");
        continue;
      }

      logEvent("Placed");
      publishText(robot_state_pub_, "Idle");
    }

    publishText(system_status_pub_, "Stopped");
    publishText(robot_state_pub_, "Idle");
    logEvent("Sequence complete");
  }

// =============================================================================
// Environment Management Methods
// =============================================================================

  /**
   * @brief Add a collision box obstacle to the planning scene
   */
  void handleAddBox()
  {
    ++box_count_;
    moveit_msgs::msg::CollisionObject collision_box;
    collision_box.header.frame_id = arm_->getPlanningFrame();
    collision_box.id = "box_" + std::to_string(box_count_);

    // Define box dimensions
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.1, 0.1, 0.9};  // width, height, depth

    // Position the box
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4 + (box_count_ - 1) * -0.8;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.45;

    collision_box.primitives.push_back(primitive);
    collision_box.primitive_poses.push_back(box_pose);
    collision_box.operation = collision_box.ADD;

    planning_scene_interface_.addCollisionObjects({collision_box});
    if (visual_tools_) {
      visual_tools_->trigger();
    }
  }

  /**
   * @brief Move the robot to the home position
   */
  void returnHome()
  {
    arm_->setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      if (estop_latched.load()) {
        logEvent("Failed: E-stop active, cannot return home");
        return;
      }
      arm_->execute(plan);
    } else {
      logEvent("Failed: home plan failed");
      return;
    }

    sendGripper(GRIPPER_OPEN_WIDTH);
  }

  /**
   * @brief Jog the robot arm in Cartesian space
   * @param dx X-axis displacement in meters
   * @param dy Y-axis displacement in meters
   * @param dz Z-axis displacement in meters
   * @param drx Rotation around X-axis in radians
   * @param dry Rotation around Y-axis in radians
   * @param drz Rotation around Z-axis in radians
   */
  void jogCartesian(double dx, double dy, double dz, double drx, double dry, double drz)
  {
    // Get current pose
    geometry_msgs::msg::PoseStamped current_pose = arm_->getCurrentPose();

    // Apply jog increments
    current_pose.pose.position.x += dx;
    current_pose.pose.position.y += dy;
    current_pose.pose.position.z += dz;

    // Apply rotation increments (simplified - for small increments)
    tf2::Quaternion current_quat;
    tf2::fromMsg(current_pose.pose.orientation, current_quat);

    tf2::Quaternion jog_quat;
    jog_quat.setRPY(drx, dry, drz);

    tf2::Quaternion new_quat = current_quat * jog_quat;
    new_quat.normalize();

    current_pose.pose.orientation = tf2::toMsg(new_quat);

    // Set target pose
    arm_->setPoseTarget(current_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      if (estop_latched.load()) {
        logEvent("Failed: E-stop active, cannot jog");
        return;
      }
      arm_->execute(plan);
    } else {
      logEvent("Failed: jog plan failed");
    }
  }

// =============================================================================
// Safety Methods
// =============================================================================

  /**
   * @brief Trigger emergency stop
   */
  void triggerEmergencyStop()
  {
    estop_requested.store(true);
    estop_latched.store(true);

    arm_->stop();
    gripper_->stop();

    publishText(system_status_pub_, "Failed");
    publishText(robot_state_pub_, "Idle");
    logEvent("Emergency stop activated");
  }

  /**
   * @brief Reset emergency stop
   */
  void resetEmergencyStop()
  {
    estop_requested.store(false);
    estop_latched.store(false);

    publishText(system_status_pub_, "Stopped");
    publishText(robot_state_pub_, "Idle");
    logEvent("Emergency stop reset");
  }

// =============================================================================
// Command Handling Methods
// =============================================================================

  /**
   * @brief Handle incoming GUI commands
   * @param msg The command message
   */
  void guiCommandCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string command = msg->data;
    std::lock_guard<std::mutex> lock(action_mutex_);

    // Pose management commands
    if (command == "save_pose") {
      if (checkEstopBlock("Failed: cannot save pose during E-stop")) return;
      savePose();
      updateSystemState("Stopped", "Idle", "Pose saved");
    }
    else if (command == "save_poses_file") {
      savePosesToFile();
      updateSystemState("Stopped", "Idle", "Poses saved to file");
    }
    else if (command == "load_poses") {
      loadPosesFromFile();
      updateSystemState("Stopped", "Idle", "Poses loaded from file");
    }
    else if (command == "clear_poses") {
      storage_.clear();
      std::ofstream(POSES_FILE) << "[]";
      updateSystemState("Stopped", "Idle", "Saved poses cleared");
    }

    // Motion commands
    else if (command == "start") {
      if (checkEstopBlock("Failed: E-stop active, cannot execute")) return;
      publishText(motion_cmd_pub_, "START");
      updateSystemState("Running", "Idle", "Sequence started");
    }
    else if (command == "stop") {
      publishText(motion_cmd_pub_, "STOP");
      updateSystemState("Stopped", "Idle", "Sequence stopped");
    }
    else if (command == "home") {
      if (checkEstopBlock("Failed: cannot return home during E-stop")) return;
      updateSystemState("Running", "Idle", "Returning home");
      returnHome();
      updateSystemState("Stopped", "Idle", "Returned home");
    }
    else if (command == "toggle_cartesian") {
      if (checkEstopBlock("Failed: cannot toggle mode during E-stop")) return;
      cartesian_mode_ = !cartesian_mode_;
      const std::string mode_msg = cartesian_mode_ ? "Cartesian mode enabled" : "Joint mode enabled";
      updateSystemState("Stopped", "Idle", mode_msg);
    }

    // Gripper commands
    else if (command == "open_gripper") {
      if (checkEstopBlock("Failed: cannot open gripper during E-stop")) return;
      updateSystemState("Running", "Placing", "Gripper opened");
      sendGripper(GRIPPER_OPEN_WIDTH);
      updateSystemState("Stopped", "Idle", "");
    }
    else if (command == "close_gripper") {
      if (checkEstopBlock("Failed: cannot close gripper during E-stop")) return;
      updateSystemState("Running", "Picking", "Gripper closed");
      sendGripper(GRIPPER_CLOSED_WIDTH);
      updateSystemState("Stopped", "Idle", "");
    }

    // Environment commands
    else if (command == "add_obstacle") {
      if (checkEstopBlock("Failed: cannot add obstacle during E-stop")) return;
      handleAddBox();
      updateSystemState("Stopped", "Idle", "Obstacle added");
    }

    // Safety commands
    else if (command == "estop") {
      triggerEmergencyStop();
    }
    else if (command == "reset") {
      resetEmergencyStop();
    }

    // Jogging commands
    else if (command.substr(0, 13) == "jog_cartesian") {
      if (checkEstopBlock("Failed: cannot jog during E-stop")) return;

      std::istringstream iss(command);
      std::string cmd;
      double dx, dy, dz, drx, dry, drz;
      iss >> cmd >> dx >> dy >> dz >> drx >> dry >> drz;

      if (iss.fail()) {
        logEvent("Failed: invalid jog command format");
        return;
      }

      updateSystemState("Running", "Idle", "Jogging...");
      jogCartesian(dx, dy, dz, drx, dry, drz);
      updateSystemState("Stopped", "Idle", "Jog complete");
    }

    // Unknown command
    else {
      logEvent("Unknown command: " + command);
    }
  }

  /**
   * @brief Update system status, robot state, and log an event
   * @param status System status message
   * @param state Robot state message
   * @param event Event log message (empty string to skip logging)
   */
  void updateSystemState(const std::string& status, const std::string& state, const std::string& event)
  {
    publishText(system_status_pub_, status);
    publishText(robot_state_pub_, state);
    if (!event.empty()) {
      logEvent(event);
    }
  }

// =============================================================================
// Environment Setup Methods
// =============================================================================

  /**
   * @brief Add the UR3e trolley collision mesh to the planning scene
   */
  void addTrolleyMesh()
  {
    moveit_msgs::msg::CollisionObject trolley_collision;
    trolley_collision.header.frame_id = arm_->getPlanningFrame();
    trolley_collision.id = "ur3e_trolley";

    const std::string mesh_path = "package://ur_gripper_demo/meshes/UR3eTrolley_decimated.dae";

    shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path);
    if (!mesh) {
      RCLCPP_ERROR(LOGGER, "Failed to load mesh from %s", mesh_path.c_str());
      return;
    }

    // Convert mesh to ROS message format
    shape_msgs::msg::Mesh mesh_message;
    shapes::ShapeMsg mesh_message_tmp;
    shapes::constructMsgFromShape(mesh, mesh_message_tmp);
    mesh_message = boost::get<shape_msgs::msg::Mesh>(mesh_message_tmp);

    // Position the mesh at origin
    geometry_msgs::msg::Pose mesh_pose;
    mesh_pose.orientation.w = 1.0;
    mesh_pose.position.x = 0.0;
    mesh_pose.position.y = 0.0;
    mesh_pose.position.z = 0.0;

    trolley_collision.meshes.push_back(mesh_message);
    trolley_collision.mesh_poses.push_back(mesh_pose);
    trolley_collision.operation = trolley_collision.ADD;

    planning_scene_interface_.applyCollisionObject(trolley_collision);
    delete mesh;  // Clean up allocated mesh
  }
};

// =============================================================================
// Main Function
// =============================================================================

/**
 * @brief Main entry point for the UR3e demo node
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<UR3EDemoNode>();
  node->init();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
