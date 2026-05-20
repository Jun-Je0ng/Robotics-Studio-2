#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>

#include <object_msgs/msg/object_array.hpp>
#include <object_msgs/msg/object.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace rvt = rviz_visual_tools;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_controller");

using GripperCommandAction = control_msgs::action::GripperCommand;
using GripperActionClient  = rclcpp_action::Client<GripperCommandAction>;

const std::string ARM_GROUP     = "ur_onrobot_manipulator";
const std::string GRIPPER_GROUP = "ur_onrobot_gripper";
const double PREGRASP_HEIGHT    = 0.03;   // metres above object centre
const double GRIPPER_OPEN       = 0.110;
const double GRIPPER_CLOSED     = 0.001;
const double SAFE_Z_HEIGHT      = 0.15;   // reduced from 0.30 — UR3e workspace limit at tray XY coords is ~0.24m; 0.35m was unreachable

// RG2 physical limits
const double RG2_MAX_SPAN       = 0.110;  // metres
// const double RG2_FINGER_MARGIN  = 0.006;  // safety margin each side

const double RG2_FINGER_MARGIN  = 0.01;  // safety margin each side zero for now to test if touchlinks does really alllow touching

const double RG2_FINGER_EXTENSION_MAX = 0.0392;  // 39.2mm from datasheet — extension at fully closed

// If the graspable dimension exceeds this, switch to SIDE strategy
const double TOP_DOWN_MAX_SPAN  = RG2_MAX_SPAN - 0.02;  // 0.090m
// If object height exceeds this, top-down risks wrist collision with object top
// const double TOP_DOWN_MAX_HEIGHT = 0.120;  // metres
const double TOP_DOWN_MAX_HEIGHT = 0.080;  // metres
std::map<std::string, geometry_msgs::msg::Pose> bin_map;

// camera + tripod location
// can be done via primitaves

// ── Camera assembly collision geometry ────────────────────────────────────────
const double CAM_ROD_X          =  0.00;   // metres — position in robot frame
const double CAM_ROD_Y          =  0.80;
const double CAM_ROD_Z          =  0.40;   // centre of vertical rod
const double CAM_ROD_LENGTH     =  0.80;   // height of tripod rod
const double CAM_ROD_RADIUS     =  0.01;  // rod thickness

const double CAM_ARM_X          =  0.00;   // horizontal arm position
const double CAM_ARM_Y          =  0.80;
const double CAM_ARM_Z          =  0.80;   // top of vertical rod
const double CAM_ARM_LENGTH     =  0.40;   // horizontal reach
const double CAM_ARM_RADIUS     =  0.025;
const double CAM_ARM_YAW        =  0.00;   // radians — rotate arm in XY plane

const double CAM_HEAD_X         =  0.80;   // camera head position
const double CAM_HEAD_Y         =  0.20;
const double CAM_HEAD_Z         =  1.20;
const double CAM_HEAD_DX        =  0.10;
const double CAM_HEAD_DY        =  0.10;
const double CAM_HEAD_DZ        =  0.08;


// sldie under parameters
// ── Side grasp / tray wall ─────────────────────────────────────────────────
const double TRAY_WALL_Y          =  0.60;   // wall position in robot frame (+Y)
const double TRAY_X_EXTENT        =  0.50;   // full tray width in X (for reference)
const double SIDE_GRASP_PITCH     =  25.0 * M_PI / 180.0;  // ramp angle in radians
const double SIDE_GRASP_MARGIN    =  0.010;  // how far past object face to push
const double SIDE_FINGER_GROUND_CLEARANCE = 0.004;  // 4mm above ground
const double SIDE_PREGRASP_OFFSET =  0.10;   // how far in -Y to start approach
const double SIDE_RETREAT_Y       =  0.10;   // how far to retreat in -Y after grasp
const double SIDE_RETREAT_Z       =  0.15;   // how high to raise during retreat


const double GRIPPER_FORCE = 20.0; // the force in Newtons the gripper will apply when closing for grasping.

// Latest wrench from /force_torque_sensor_broadcaster/wrench
// Populated by a subscriber in main(); read by confirmGrasp().
geometry_msgs::msg::WrenchStamped::SharedPtr latest_wrench;
std::mutex wrench_mutex;

// Set from the "sim" ROS parameter at startup (see main()).
// When true, closeGripperForGrasp() skips stall detection and always returns grasped.
bool g_sim_mode = false;

// Resolved path to bin_poses.json — set once in main(), used everywhere.
std::string g_bin_poses_file;

// user interface states
bool sequence_requested = true;

// settings
int attempts = 2;


// incoming message
// Array of:
// std_msgs/Header header
// geometry_msgs/Pose pose
// string classification       # material type e.g. 'metal', 'plastic', 'fabric'
// float64[3] dimensions       # dimensions of the object aligned to the pose . i.e. this is what the bounding box definition would be

// plsu array of goal poses or positions

// ============================================================
// Structs
// ============================================================
const std::map<std::string, int> BIN_MAP = {
    {"metal",        0},
    {"plastic",      1},
    {"fabric",       2},
    {"hdpe_bottle",  1},
    {"pet_bottle",   1},
    {"bottle",   1},
};

enum class GraspStrategy {
    TOP_DOWN,
    SIDE_VERTICAL,   // current "slide under"
    SIDE_HORIZONTAL  // new bottle-style grasp
};

enum class PickResult {
    SUCCESS,      // object grasped and placed
    GRASP_FAILED, // arm reached object but fingers found nothing — retry with fresh perception
    SKIPPED,      // unknown classification, no bin defined
    DROPPED,      // grasped but object fell during transport
};

struct GraspGeometry
{
  GraspStrategy strategy;
  int           thin_axis;      // 0=x, 1=y, 2=z in object frame
  double        grip_width;     // metres — commanded finger separation
  double        object_yaw;     // radians — yaw extracted from object pose
  double        finger_angle;
};

struct ResolvedObject
{
  object_msgs::msg::Object obj;

  int bin_index;
  GraspGeometry grasp;

  std::string id;   // unique scene ID
};

// ============================================================
// Helpers
// ============================================================



// pose loading:
void loadBinPoses(const std::string& file)
{
    std::ifstream in(file);
    if (!in.good())
    {
        RCLCPP_WARN(LOGGER, "No bin pose file found at: %s", file.c_str());
        return;
    }

    nlohmann::json j;
    in >> j;

    bin_map.clear();

    for (auto& [label, val] : j.items())
    {
        geometry_msgs::msg::Pose pose;

        pose.position.x = val["position"][0];
        pose.position.y = val["position"][1];
        pose.position.z = val["position"][2];

        tf2::Quaternion q;
        q.setRPY(
            val["orientation"][0],
            val["orientation"][1],
            val["orientation"][2]);

        pose.orientation = tf2::toMsg(q);

        bin_map[label] = pose;
    }

    RCLCPP_INFO(LOGGER, "Loaded %zu bin poses", bin_map.size());
}

void saveBinPose(
    const std::string& label,
    const geometry_msgs::msg::Pose& pose,
    const std::string& file)
{
    // 1. update in-memory map immediately
    bin_map[label] = pose;

    // 2. persist to disk
    nlohmann::json j;

    for (const auto& [k, v] : bin_map)
    {
        tf2::Quaternion q;
        tf2::fromMsg(v.orientation, q);

        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);

        j[k] = {
            {"position", {v.position.x, v.position.y, v.position.z}},
            {"orientation", {r, p, y}}
        };
    }

    std::ofstream out(file);
    out << j.dump(4);

    RCLCPP_INFO(LOGGER, "Saved + updated: %s", label.c_str());
}

geometry_msgs::msg::Pose getDropOffPose(const std::string& class_name)
{
    auto it = bin_map.find(class_name);

    if (it == bin_map.end())
    {
        RCLCPP_ERROR(LOGGER, "No bin defined for class: %s", class_name.c_str());

        geometry_msgs::msg::Pose fallback;
        fallback.orientation.w = 1.0;
        return fallback;
    }

    return it->second;
}

inline const char* toString(GraspStrategy s)
{
    switch (s)
    {
        case GraspStrategy::TOP_DOWN: return "TOP_DOWN";
        case GraspStrategy::SIDE_VERTICAL: return "SIDE_VERTICAL";
        case GraspStrategy::SIDE_HORIZONTAL: return "SIDE_HORIZONTAL";
        default: return "UNKNOWN";
    }
}


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
    pose.position.z    = -0.055;

    ground.primitives.push_back(primitive);
    ground.primitive_poses.push_back(pose);
    ground.operation = ground.ADD;

    psi.addCollisionObjects({ground});

    moveit_msgs::msg::CollisionObject platform;
    platform.header.frame_id = arm.getPlanningFrame();
    platform.id = "platform";


    shape_msgs::msg::SolidPrimitive primitive_;
    primitive_.type       = primitive_.BOX;
    primitive_.dimensions = {0.50, 0.32, 0.01};

    geometry_msgs::msg::Pose platform_pose;
    platform_pose.orientation.w = 1.0;
    platform_pose.position.z    = 0.005;
    platform_pose.position.y    = 0.30;

    platform.primitives.push_back(primitive_);
    platform.primitive_poses.push_back(platform_pose);
    platform.operation = platform.ADD;

    psi.addCollisionObjects({platform});

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void spawnCameraAssembly(
    moveit::planning_interface::PlanningSceneInterface& psi,
    const std::string& frame_id = "base_link"){
    std::vector<moveit_msgs::msg::CollisionObject> objects;

    auto makeCylinder = [&](
        const std::string& id,
        double x, double y, double z,
        double length, double radius,
        double roll, double pitch, double yaw)
    {
        moveit_msgs::msg::CollisionObject co;
        co.id              = id;
        co.header.frame_id = frame_id;
        co.operation       = moveit_msgs::msg::CollisionObject::ADD;

        shape_msgs::msg::SolidPrimitive cyl;
        cyl.type       = shape_msgs::msg::SolidPrimitive::CYLINDER;
        cyl.dimensions = {length, radius};  // height, radius

        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        pose.orientation = tf2::toMsg(q);

        co.primitives.push_back(cyl);
        co.primitive_poses.push_back(pose);
        return co;
    };

    auto makeBox = [&](
        const std::string& id,
        double x, double y, double z,
        double dx, double dy, double dz,
        double yaw)
    {
        moveit_msgs::msg::CollisionObject co;
        co.id              = id;
        co.header.frame_id = frame_id;
        co.operation       = moveit_msgs::msg::CollisionObject::ADD;

        shape_msgs::msg::SolidPrimitive box;
        box.type       = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {dx, dy, dz};

        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pose.orientation = tf2::toMsg(q);

        co.primitives.push_back(box);
        co.primitive_poses.push_back(pose);
        return co;
    };

    // Vertical rod — cylinder standing upright
    objects.push_back(makeCylinder(
        "cam_rod",
        CAM_ROD_X, CAM_ROD_Y, CAM_ROD_Z,
        CAM_ROD_LENGTH, CAM_ROD_RADIUS,
        0.0, 0.0, 0.0));

    // Horizontal arm — cylinder rotated 90deg around Y
    objects.push_back(makeCylinder(
        "cam_arm",
        CAM_ARM_X, CAM_ARM_Y, CAM_ARM_Z,
        CAM_ARM_LENGTH, CAM_ARM_RADIUS,
        0.0, M_PI / 2.0, CAM_ARM_YAW));

    // Camera head — box
    // objects.push_back(makeBox(
    //     "cam_head",
    //     CAM_HEAD_X, CAM_HEAD_Y, CAM_HEAD_Z,
    //     CAM_HEAD_DX, CAM_HEAD_DY, CAM_HEAD_DZ,
    //     0.0));

    psi.addCollisionObjects(objects);

    RCLCPP_INFO(LOGGER, "Camera assembly spawned (%zu parts)", objects.size());
}

void addTrolleyMesh(
    moveit::planning_interface::MoveGroupInterface& arm,
    moveit::planning_interface::PlanningSceneInterface& psi)
{
    moveit_msgs::msg::CollisionObject trolley;
    trolley.header.frame_id = arm.getPlanningFrame();
    trolley.id = "ur3e_trolley";

    std::string mesh_path = "package://ur_gripper_demo/meshes/UR3eTrolley_decimated.dae";

    shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path);
    if (!mesh) {
        RCLCPP_ERROR(LOGGER, "Failed to load mesh from %s", mesh_path.c_str());
        return;
    }

    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(mesh, shape_msg);

    shape_msgs::msg::Mesh mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

    geometry_msgs::msg::Pose mesh_pose;
    mesh_pose.orientation.w = 1.0;
    mesh_pose.position.x = 0.0;
    mesh_pose.position.y = 0.0;
    mesh_pose.position.z = -2.0;

    trolley.meshes.push_back(mesh_msg);
    trolley.mesh_poses.push_back(mesh_pose);
    trolley.operation = trolley.ADD;

    psi.addCollisionObjects({trolley});

    delete mesh;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}


double extractYaw(const geometry_msgs::msg::Quaternion& q_msg){
  tf2::Quaternion q;
  tf2::fromMsg(q_msg, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

// ── Low-level gripper action helper ─────────────────────────────────────────
// Sends a GripperCommand goal and blocks until the action completes.
// The executor must already be spinning (it is — see spinner thread in main).
bool sendGripperAction(
    const GripperActionClient::SharedPtr& client,
    double position,
    double max_effort = 20.0)
{
    position = std::clamp(position, 0.0, RG2_MAX_SPAN);

    if (!client->wait_for_action_server(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(LOGGER, "Gripper action server not available");
        return false;
    }

    GripperCommandAction::Goal goal;
    goal.command.position   = position;
    goal.command.max_effort = max_effort;

    auto goal_future = client->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(LOGGER, "Gripper goal send timed out");
        return false;
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(LOGGER, "Gripper goal was rejected by server");
        return false;
    }

    auto result_future = client->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
        RCLCPP_ERROR(LOGGER, "Gripper action timed out waiting for result");
        return false;
    }

    auto result = result_future.get();
    RCLCPP_INFO(LOGGER,
        "Gripper result: pos=%.3fm effort=%.1fN stalled=%s reached_goal=%s",
        result.result->position, result.result->effort,
        result.result->stalled      ? "true" : "false",
        result.result->reached_goal ? "true" : "false");

    return true;
}

// ── Open gripper ─────────────────────────────────────────────────────────────
void openGripper(const GripperActionClient::SharedPtr& client, double width = GRIPPER_OPEN)
{
    RCLCPP_INFO(LOGGER, "Gripper → OPEN (%.3fm)", width);
    sendGripperAction(client, GRIPPER_OPEN, 20.0);
}

// ── Close gripper for grasping ────────────────────────────────────────────────
// Sends position = 0.005 m (nearly fully closed).
// GripperActionController stops and sets stalled=true when the fingers hit an
// object — that IS the grasp detection signal.
//
// Returns true  → stalled = object grasped.
// Returns false → reached target without stall = nothing in the gripper.
bool closeGripperForGrasp(const GripperActionClient::SharedPtr& client, double target_width = GRIPPER_CLOSED, double grasp_effort = GRIPPER_FORCE)
{
    // In simulation the gripper never stalls (no contact physics), so skip the
    // stall check and return true immediately. We still close visually so RViz looks right.
    if (g_sim_mode)
    {
        RCLCPP_INFO(LOGGER, "[SIM] Closing gripper visually (no stall detection) → grasped=true");
        // sendGripperAction(client, 0.05, grasp_effort);   // ~50mm looks plausible in RViz
        return true;
    }

    RCLCPP_INFO(LOGGER,
        "Closing gripper for grasp → target %.1fmm (stall = grasped)",
        target_width * 1000.0);

    if (!client->wait_for_action_server(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(LOGGER, "Gripper action server not available");
        return false;
    }

    GripperCommandAction::Goal goal;
    goal.command.position   = target_width;
    goal.command.max_effort = grasp_effort;

    auto goal_future = client->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(LOGGER, "Gripper grasp goal send timed out");
        return false;
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(LOGGER, "Gripper grasp goal was rejected");
        return false;
    }

    auto result_future = client->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
        RCLCPP_ERROR(LOGGER, "Gripper grasp action timed out");
        return false;
    }

    auto result  = result_future.get();
    bool grasped = result.result->stalled;

    RCLCPP_INFO(LOGGER,
        "Grasp close: pos=%.3fm stalled=%s reached_goal=%s → %s",
        result.result->position,
        result.result->stalled      ? "true" : "false",
        result.result->reached_goal ? "true" : "false",
        grasped ? "GRASPED" : "EMPTY (no object detected)");

    return grasped;
}

void returnHome(
    moveit::planning_interface::MoveGroupInterface& arm,
    const GripperActionClient::SharedPtr& gripper_client)
{
    arm.setNamedTarget("up");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        arm.setMaxVelocityScalingFactor(0.03);
        arm.setMaxAccelerationScalingFactor(0.03);
        arm.execute(plan);
    }
    openGripper(gripper_client, 0.088);
}

void spawnCollisionObject(
    moveit::planning_interface::PlanningSceneInterface & psi,
    const object_msgs::msg::Object & obj,
    const std::string & id,
    const std::string & frame_id = "base"){
    moveit_msgs::msg::CollisionObject co;
    co.id              = id;
    co.header.frame_id = frame_id;
    co.operation       = moveit_msgs::msg::CollisionObject::ADD;

    // Cap each dimension so a long/thick object collision box cannot
    // intersect the robot arm during raise.  Objects are confirmed flat
    // (≤5 cm height) so clamp z; x/y capped at 0.15 m so the box
    // doesn't reach the forearm when the object is attached horizontally.
    const double MAX_DIM   = 0.15;   // m — max footprint dimension
    const double MAX_Z_DIM = 0.05;   // m — max height (flat objects only)

    double dx = std::min(obj.dimensions[0] - RG2_FINGER_MARGIN, MAX_DIM);
    double dy = std::min(obj.dimensions[1] - RG2_FINGER_MARGIN, MAX_DIM);
    double dz = std::min(obj.dimensions[2],                      MAX_Z_DIM);

    shape_msgs::msg::SolidPrimitive box;
    box.type       = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {dx, dy, dz};

    co.primitives.push_back(box);
    co.primitive_poses.push_back(obj.pose);

    psi.addCollisionObjects({co});

    RCLCPP_INFO(LOGGER, "Spawned '%s'  [%.3f x %.3f x %.3f]  class=%s",
        id.c_str(), dx, dy, dz, obj.classification.c_str());
}

// void spawnCollisionObject(
//     moveit::planning_interface::PlanningSceneInterface & psi,
//     const object_msgs::msg::Object & obj,
//     const std::string & id,
//     const std::string & frame_id = "base")
// {
//     moveit_msgs::msg::CollisionObject co;
//     co.id              = id;
//     co.header.frame_id = frame_id;
//     co.operation       = moveit_msgs::msg::CollisionObject::ADD;

//     shape_msgs::msg::SolidPrimitive cylinder;
//     cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;

//     double x = obj.dimensions[0];
//     double y = obj.dimensions[1];
//     double z = obj.dimensions[2];

//     double radius = 0.5 * std::min(x, y) - RG2_FINGER_MARGIN;
//     double height = z - RG2_FINGER_MARGIN;

//     cylinder.dimensions.resize(2);
//     cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = height;
//     cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = radius;

//     // --- FIX: use corrected pose ---
//     geometry_msgs::msg::Pose cyl_pose = obj.pose;

//     // OPTIONAL: lay cylinder on its side (X-axis alignment)
//     tf2::Quaternion q;
//     q.setRPY(M_PI / 2.0, 0.0, 0.0);   // rotate cylinder from Z → X
//     cyl_pose.orientation = tf2::toMsg(q);

//     co.primitives.push_back(cylinder);
//     co.primitive_poses.push_back(cyl_pose);  // ✅ use corrected pose

//     psi.addCollisionObjects({co});

//     RCLCPP_INFO(LOGGER,
//         "Spawned CYLINDER '%s' [h=%.3f, r=%.3f] class=%s",
//         id.c_str(),
//         height,
//         radius,
//         obj.classification.c_str());
// }


// ============================================================
// Grasp geometry — full strategy selection based on object dimensions.
//
// Strategy rules:
//   TOP_DOWN      — narrowest XY span fits gripper AND object is short
//                   (most flat/side-lying objects)
//   SIDE_HORIZONTAL — object is tall (upright bottle) and not thin
//                   (approach horizontally, fingers vertical)
//   SIDE_VERTICAL  — wide flat object that cannot be gripped top-down
//                   (slide fingers underneath from the side)
// ============================================================
GraspGeometry computeGraspGeometry(const object_msgs::msg::Object& obj)
{
    GraspGeometry g;
    const auto& d = obj.dimensions;  // [dx, dy, dz]

    // Find thin axis
    g.thin_axis = 0;
    if (d[1] < d[g.thin_axis]) g.thin_axis = 1;
    if (d[2] < d[g.thin_axis]) g.thin_axis = 2;

    g.object_yaw = extractYaw(obj.pose.orientation);

    double xy_min = std::min(d[0], d[1]);
    double xy_max = std::max(d[0], d[1]);
    double height = d[2];

    // Strategy selection based on object dimensions
    bool fits_topdown = (xy_min < TOP_DOWN_MAX_SPAN) && (height <= TOP_DOWN_MAX_HEIGHT);
    bool is_tall      = height > TOP_DOWN_MAX_HEIGHT;   // likely upright bottle
    bool is_thin      = xy_min < 0.03;                  // thin profile → slide-under more reliable

    if (fits_topdown)
        g.strategy = GraspStrategy::TOP_DOWN;
    else if (is_tall && !is_thin)
        g.strategy = GraspStrategy::SIDE_HORIZONTAL;    // upright bottle, grip from sides
    else
        g.strategy = GraspStrategy::SIDE_VERTICAL;      // wide flat object, slide under

    if (!g_sim_mode)
    {
        RCLCPP_INFO(LOGGER, "[Non_simulation] Grasp geometry computed (strategy selection disabled) → using TOP_DOWN for all objects");
        g.strategy = GraspStrategy::TOP_DOWN;
    }

    // Grip width — this is the MINIMUM the gripper will close to (safety stop).
    // Force detection stops closing before this in normal operation.
    // TOP_DOWN: set 10mm wider than object so force detection fires first.
    if (g.strategy == GraspStrategy::TOP_DOWN) {
        g.grip_width = std::clamp(xy_min + 0.010, 0.0, RG2_MAX_SPAN);
    }
    else if (g.strategy == GraspStrategy::SIDE_HORIZONTAL) {
        g.grip_width = std::clamp(xy_min + 0.005, 0.0, RG2_MAX_SPAN);
    } else {  // SIDE_VERTICAL
        std::vector<double> cross_section;
        for (int i = 0; i < 3; ++i)
            if (i != g.thin_axis)
                cross_section.push_back(d[i]);

        g.grip_width = std::clamp(
            *std::min_element(cross_section.begin(), cross_section.end()) + 0.005,
            0.0,
            RG2_MAX_SPAN);
    }

    // Finger alignment (only matters for TOP_DOWN)
    if (g.strategy == GraspStrategy::TOP_DOWN) {
        if (d[0] >= d[1])
            g.finger_angle = g.object_yaw;
        else
            g.finger_angle = g.object_yaw + M_PI / 2.0;
    } else {
        g.finger_angle = g.object_yaw;
    }

    RCLCPP_INFO(LOGGER,
        "Grasp geometry: strategy=%s thin_axis=%d dims=[%.3f %.3f %.3f] grip_width=%.3f yaw=%.2f",
        toString(g.strategy),
        g.thin_axis, d[0], d[1], d[2], g.grip_width, g.object_yaw);
    return g;
}


double normalizeGraspYaw(double yaw)
{
    // Fold yaw into [-PI/2, PI/2] — approach from nearest side
    while (yaw >  M_PI / 2.0) yaw -= M_PI;
    while (yaw < -M_PI / 2.0) yaw += M_PI;
    return yaw;
}

geometry_msgs::msg::Quaternion computeGraspOrientation(
    const GraspGeometry& g,
    GraspStrategy strategy_override = GraspStrategy::TOP_DOWN,
    bool use_override = false){
    GraspStrategy strat = use_override ? strategy_override : g.strategy;

    tf2::Quaternion q;

    if (strat == GraspStrategy::TOP_DOWN)
    {
        q.setRPY(M_PI, 0.0, g.finger_angle);
    }
    else if (strat == GraspStrategy::SIDE_HORIZONTAL)
    {
        // approach_angle is passed in via g.finger_angle for SIDE_HORIZONTAL
        // (robot-base → object direction, set in executeSideHorizontalGrasp).
        // setRPY(π/2, 0, φ + π/2) puts the gripper Z axis along
        // (cos φ, sin φ, 0) — pointing horizontally toward the object —
        // with the gripper body upright and fingers in the horizontal plane.
        q.setRPY(M_PI / 2.0, 0.0, g.finger_angle + M_PI / 2.0);
    }
    else if (strat == GraspStrategy::SIDE_VERTICAL)
    {
        double yaw = 0.0;  // push toward +Y wall → fixed orientation

        // Roll = 90° → vertical gripper
        // Pitch = ramp angle → scoop
        // Yaw = toward wall
        q.setRPY(M_PI / 2.0, SIDE_GRASP_PITCH, yaw);
    }

    q.normalize();
    return tf2::toMsg(q);
}

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

double fingerExtension(double grip_width) {
    return RG2_FINGER_EXTENSION_MAX * (grip_width / RG2_MAX_SPAN);
}

bool moveToPregrasp(
    moveit::planning_interface::MoveGroupInterface& arm,
    const ResolvedObject& r){
    geometry_msgs::msg::Pose target;

    target.position.x = r.obj.pose.position.x;
    target.position.y = r.obj.pose.position.y;
    target.position.z = r.obj.pose.position.z
                      + (r.obj.dimensions[2] / 2.0)
                      + PREGRASP_HEIGHT
                      - fingerExtension(r.grasp.grip_width);

    target.orientation = computeGraspOrientation(r.grasp);

    arm.setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (ok)
    {
        arm.setMaxVelocityScalingFactor(0.03);
        arm.setMaxAccelerationScalingFactor(0.03);
        arm.execute(plan);
    }
    else
        RCLCPP_WARN(LOGGER, "Pregrasp plan failed for '%s'", r.id.c_str());

    return ok;
}

void attachObject(
    moveit::planning_interface::MoveGroupInterface& arm,
    const std::string& object_id){

    std::vector<std::string> touch_links = {
        "wrist_3_link",
        "wrist_2_link",
        "wrist_1_link",
        "tool0",
        // RG2 gripper
        "onrobot_base_link",
        "cable_connector_0",
        "cable_connector_1",
        "left_inner_knuckle",
        "left_outer_knuckle",
        "left_inner_finger",
        "left_finger_tip",
        "right_inner_knuckle",
        "right_outer_knuckle",
        "right_inner_finger",
        "right_finger_tip",
        "finger_width_mock_link",
        "gripper_tcp"
    };

    arm.attachObject(object_id, arm.getEndEffectorLink(), touch_links);

    RCLCPP_INFO(LOGGER, "Attached '%s' to '%s'",
        object_id.c_str(), arm.getEndEffectorLink().c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void detachObject(
    moveit::planning_interface::MoveGroupInterface& arm,
    const std::string& object_id){

    arm.detachObject(object_id);
    RCLCPP_INFO(LOGGER, "Detached '%s'", object_id.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void removeObject(
    moveit::planning_interface::PlanningSceneInterface& psi,
    const std::string& object_id){
    psi.removeCollisionObjects({object_id});
    RCLCPP_INFO(LOGGER, "Removed '%s' from planning scene", object_id.c_str());
}


bool moveCartesian(
    moveit::planning_interface::MoveGroupInterface& arm,
    const geometry_msgs::msg::Pose& target,
    double max_step = 0.002,
    double min_fraction = 0.9){
    std::vector<geometry_msgs::msg::Pose> waypoints{target};
    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = arm.computeCartesianPath(waypoints, max_step, 0.0, traj);

    if (fraction < min_fraction)
    {
        RCLCPP_ERROR(LOGGER, "Cartesian path only %.0f%% complete", fraction * 100.0);
        return false;
    }

    // computeCartesianPath does not inherit setMaxVelocityScalingFactor —
    // manually apply time parametrization so joint velocities respect the 0.03 limit.
    robot_trajectory::RobotTrajectory rt(arm.getRobotModel(), arm.getName());
    rt.setRobotTrajectoryMsg(*arm.getCurrentState(), traj);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, 0.03, 0.03);
    rt.getRobotTrajectoryMsg(traj);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = traj;
    arm.setMaxVelocityScalingFactor(0.03);
    arm.setMaxAccelerationScalingFactor(0.03);
    return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

// Sum of absolute joint angle changes across a trajectory.
// Lower = fewer/smaller joint movements = more efficient motion.
double computePlanCost(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    const auto& pts = plan.trajectory_.joint_trajectory.points;
    if (pts.size() < 2) return 0.0;

    double total = 0.0;
    for (size_t i = 1; i < pts.size(); ++i)
        for (size_t j = 0; j < pts[i].positions.size(); ++j)
            total += std::abs(pts[i].positions[j] - pts[i - 1].positions[j]);

    return total;
}

// Plan to a Cartesian pose, but try multiple times and execute the plan with
// the lowest total joint movement.  This prevents the robot from choosing a
// 270° shoulder rotation when a 90° one reaches the same Cartesian target.
bool moveToPose(
    moveit::planning_interface::MoveGroupInterface& arm,
    const geometry_msgs::msg::Pose& pose,
    int num_attempts = 8)
{
    arm.setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan best_plan;
    double best_cost = std::numeric_limits<double>::infinity();

    for (int i = 0; i < num_attempts; ++i)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            double cost = computePlanCost(plan);
            RCLCPP_DEBUG(LOGGER, "Plan attempt %d: cost=%.3f", i + 1, cost);
            if (cost < best_cost)
            {
                best_cost = cost;
                best_plan = plan;
            }
        }
    }

    if (best_cost == std::numeric_limits<double>::infinity())
    {
        RCLCPP_ERROR(LOGGER, "All %d planning attempts failed", num_attempts);
        return false;
    }

    RCLCPP_INFO(LOGGER, "Executing lowest-cost plan (cost=%.3f) out of %d attempts",
                best_cost, num_attempts);
    arm.setMaxVelocityScalingFactor(0.03);
    arm.setMaxAccelerationScalingFactor(0.03);
    return arm.execute(best_plan) == moveit::core::MoveItErrorCode::SUCCESS;
}


bool executeTopDownGrasp(
    moveit::planning_interface::MoveGroupInterface& arm,
    const GripperActionClient::SharedPtr& gripper_client,
    moveit::planning_interface::PlanningSceneInterface& psi,
    const ResolvedObject& r,
    rclcpp::Node::SharedPtr node,
    int max_attempts = 3)
{
    for (int attempt = 1; attempt <= max_attempts; ++attempt)
    {
        RCLCPP_INFO(LOGGER, "Grasp attempt %d/%d for '%s'", attempt, max_attempts, r.id.c_str());

        if (!moveToPregrasp(arm, r))
        {
            RCLCPP_WARN(LOGGER, "Pregrasp failed on attempt %d", attempt);
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Descend
        geometry_msgs::msg::Pose grasp_pose;
        grasp_pose.position.x = r.obj.pose.position.x;
        grasp_pose.position.y = r.obj.pose.position.y;
        grasp_pose.position.z = r.obj.pose.position.z
                              + (r.obj.dimensions[2] / 2.0)
                              - fingerExtension(r.grasp.grip_width);
        grasp_pose.orientation = computeGraspOrientation(r.grasp);

        if (!moveCartesian(arm, grasp_pose))
        {
            RCLCPP_WARN(LOGGER, "Descend failed on attempt %d", attempt);
            continue;
        }

        // Attach + close
        attachObject(arm, r.id);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        bool grasped = closeGripperForGrasp(gripper_client);
        if (!grasped)
        {
            RCLCPP_WARN(LOGGER, "No object detected (gripper not stalled) on attempt %d — releasing", attempt);
            openGripper(gripper_client);
            detachObject(arm, r.id);
            continue;
        }

        // Remove collision geometry before raising — the attached box causes
        // "Invalid goal state" self-collision in the raise configuration.
        detachObject(arm, r.id);
        removeObject(psi, r.id);

        geometry_msgs::msg::Pose raise_pose = grasp_pose;
        raise_pose.position.z = r.obj.pose.position.z + SAFE_Z_HEIGHT;

                              
        if (!moveCartesian(arm, raise_pose))
        {
            RCLCPP_WARN(LOGGER, "Raise failed on attempt %d — detaching", attempt);
            if (!moveToPose(arm, raise_pose))
                {
                    RCLCPP_WARN(LOGGER, "Raise failed on attempt %d — detaching", attempt);
                    openGripper(gripper_client);
                    detachObject(arm, r.id);
                    continue;
                }
        }

        // if (!moveToPose(arm, raise_pose))
        // {
        //     RCLCPP_WARN(LOGGER, "Raise failed on attempt %d — detaching", attempt);
        //     openGripper(gripper_client);
        //     detachObject(arm, r.id);
        //     continue;
        // }

        RCLCPP_INFO(LOGGER, "Grasp succeeded on attempt %d", attempt);
        return true;
    }

    RCLCPP_ERROR(LOGGER, "Failed to grasp '%s' after %d attempts", r.id.c_str(), max_attempts);
    return false;
}

bool executeSideHorizontalGrasp(
    moveit::planning_interface::MoveGroupInterface& arm,
    const GripperActionClient::SharedPtr& gripper_client,
    const ResolvedObject& r,
    rclcpp::Node::SharedPtr node,
    int max_attempts = 3){
    for (int attempt = 1; attempt <= max_attempts; ++attempt)
    {
        RCLCPP_INFO(LOGGER, "SIDE_HORIZONTAL attempt %d/%d for '%s'",
                    attempt, max_attempts, r.id.c_str());

        // Direction from robot base origin to object in XY.
        const double obj_x = r.obj.pose.position.x;
        const double obj_y = r.obj.pose.position.y;
        const double dist_to_obj = std::sqrt(obj_x * obj_x + obj_y * obj_y);
        double approach_angle = std::atan2(obj_y, obj_x);

        // Minimum clearance the pregrasp must keep from the robot base.
        // UR3e can't reach poses closer than ~150 mm from base_link in XY.
        const double MIN_BASE_CLEARANCE = 0.15;
        const double desired_approach   = 0.10;

        // Clamp so that pregrasp stays at least MIN_BASE_CLEARANCE from base.
        // If object is too close to even allow that, skip this attempt.
        double approach_dist = std::min(desired_approach,
                                        dist_to_obj - MIN_BASE_CLEARANCE);
        if (approach_dist <= 0.0)
        {
            RCLCPP_WARN(LOGGER,
                "SIDE_HORIZONTAL: object too close to base (dist=%.3fm) — skipping",
                dist_to_obj);
            continue;
        }

        // Temporarily store approach_angle in a mutable copy of grasp so that
        // computeGraspOrientation can read it via finger_angle.
        GraspGeometry grasp_copy   = r.grasp;
        grasp_copy.finger_angle    = approach_angle;

        // -----------------------
        // 1. Pregrasp (offset in XY)
        // -----------------------
        // Stand off from the object along the approach vector (robot→object),
        // at the object's centroid height.
        geometry_msgs::msg::Pose pregrasp;

        pregrasp.position.x = r.obj.pose.position.x - approach_dist * std::cos(approach_angle);
        pregrasp.position.y = r.obj.pose.position.y - approach_dist * std::sin(approach_angle);
        pregrasp.position.z = r.obj.pose.position.z;  // mid-height of object

        pregrasp.orientation = computeGraspOrientation(
            grasp_copy, GraspStrategy::SIDE_HORIZONTAL, true);

        RCLCPP_INFO(LOGGER,
            "Pregrasp target: x=%.3f y=%.3f z=%.3f  approach_angle=%.2f rad"
            "  qx=%.3f qy=%.3f qz=%.3f qw=%.3f",
            pregrasp.position.x, pregrasp.position.y, pregrasp.position.z,
            approach_angle,
            pregrasp.orientation.x, pregrasp.orientation.y,
            pregrasp.orientation.z, pregrasp.orientation.w);

        if (!moveToPose(arm, pregrasp))
        {
            RCLCPP_WARN(LOGGER, "Pregrasp failed");
            continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        // -----------------------
        // 2. Approach (XY Cartesian)
        // -----------------------
        geometry_msgs::msg::Pose grasp_pose = pregrasp;
        grasp_pose.position.x = r.obj.pose.position.x;
        grasp_pose.position.y = r.obj.pose.position.y;

        if (!moveCartesian(arm, grasp_pose))
        {
            RCLCPP_WARN(LOGGER, "Approach failed");
            continue;
        }

        // -----------------------
        // 3. Close + attach
        // -----------------------
        attachObject(arm, r.id);

        bool grasped = closeGripperForGrasp(gripper_client);
        if (!grasped)
        {
            RCLCPP_WARN(LOGGER, "No object detected (gripper not stalled) on attempt %d — releasing", attempt);
            detachObject(arm, r.id);
            openGripper(gripper_client);
            continue;
        }

        // -----------------------
        // 4. Lift slightly (avoid drag)
        // -----------------------
        geometry_msgs::msg::Pose lift_pose = grasp_pose;
        lift_pose.position.z += 0.05;

        if (!moveCartesian(arm, lift_pose))
        {
            RCLCPP_WARN(LOGGER, "Lift failed — detaching");
            detachObject(arm, r.id);
            continue;
        }

        // -----------------------
        // 5. Retreat back (XY)
        // -----------------------
        geometry_msgs::msg::Pose retreat_pose = lift_pose;
        retreat_pose.position.x -= approach_dist * std::cos(approach_angle);
        retreat_pose.position.y -= approach_dist * std::sin(approach_angle);

        moveCartesian(arm, retreat_pose);

        RCLCPP_INFO(LOGGER, "SIDE_HORIZONTAL grasp succeeded");
        return true;
    }

    RCLCPP_ERROR(LOGGER, "SIDE_HORIZONTAL failed for '%s'", r.id.c_str());
    return false;
}

bool executeSideVerticalGrasp(
    moveit::planning_interface::MoveGroupInterface& arm,
    const GripperActionClient::SharedPtr& gripper_client,
    moveit::planning_interface::PlanningSceneInterface& psi,
    const ResolvedObject& r,
    rclcpp::Node::SharedPtr node,
    int max_attempts = 3){

    for (int attempt = 1; attempt <= max_attempts; ++attempt)
    {
        RCLCPP_INFO(LOGGER, "SIDE_VERTICAL attempt %d/%d for '%s'",
                    attempt, max_attempts, r.id.c_str());

        const auto& obj = r.obj;
        const auto& d   = obj.dimensions;

        double obj_thickness = d[1];

        // -----------------------
        // 1. Pregrasp
        // -----------------------
        geometry_msgs::msg::Pose pregrasp;

        pregrasp.position.x = obj.pose.position.x;
        pregrasp.position.y = obj.pose.position.y - SIDE_PREGRASP_OFFSET;

        pregrasp.position.z = obj.pose.position.z - (d[2] / 2.0)
                            + SIDE_FINGER_GROUND_CLEARANCE
                            + 0.01;

        pregrasp.orientation = computeGraspOrientation(
            r.grasp, GraspStrategy::SIDE_VERTICAL, true);

        if (!moveToPose(arm, pregrasp))
        {
            RCLCPP_WARN(LOGGER, "Pregrasp failed");
            continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        // -----------------------
        // 2. Allow collision ONLY with this object (gripper links only)
        // -----------------------
        moveit_msgs::msg::PlanningScene ps;
        ps.is_diff = true;

        auto links = arm.getLinkNames();

        ps.allowed_collision_matrix.entry_names.push_back(r.id);
        ps.allowed_collision_matrix.entry_values.emplace_back();

        std::vector<std::string> gripper_links = {
            "wrist_3_link",
            "tool0",
            "onrobot_base_link",
            "cable_connector_0",
            "cable_connector_1",
            "left_inner_knuckle",
            "left_outer_knuckle",
            "left_inner_finger",
            "left_finger_tip",
            "right_inner_knuckle",
            "right_outer_knuckle",
            "right_inner_finger",
            "right_finger_tip",
            "finger_width_mock_link",
            "gripper_tcp"
        };

        ps.allowed_collision_matrix.entry_values.back().enabled.resize(links.size(), false);

        for (const auto& link : gripper_links)
        {
            auto it = std::find(links.begin(), links.end(), link);
            if (it != links.end())
            {
                size_t idx = std::distance(links.begin(), it);
                ps.allowed_collision_matrix.entry_values.back().enabled[idx] = true;
            }
        }

        psi.applyPlanningScene(ps);

        // -----------------------
        // 3. Push target
        // -----------------------
        double push_target_y =
            TRAY_WALL_Y - obj_thickness + SIDE_GRASP_MARGIN;

        geometry_msgs::msg::Pose push_pose = pregrasp;
        push_pose.position.y = push_target_y;

        if (!moveCartesian(arm, push_pose))
        {
            RCLCPP_WARN(LOGGER, "Push failed");
            continue;
        }

        // -----------------------
        // 4. Close gripper
        // -----------------------

        bool grasped = closeGripperForGrasp(gripper_client);
        if (!grasped)
        {
            RCLCPP_WARN(LOGGER, "No object detected (gripper not stalled) on attempt %d — releasing", attempt);
            openGripper(gripper_client);
            continue;
        }

        // -----------------------
        // 5. Retreat + lift
        // -----------------------
        geometry_msgs::msg::Pose retreat_pose = push_pose;

        retreat_pose.position.y -= SIDE_RETREAT_Y;
        retreat_pose.position.z += SIDE_RETREAT_Z;

        if (!moveCartesian(arm, retreat_pose))
        {
            RCLCPP_WARN(LOGGER, "Retreat failed");
            continue;
        }

        // -----------------------
        // 6. Attach object
        // -----------------------
        attachObject(arm, r.id);

        RCLCPP_INFO(LOGGER, "SIDE_VERTICAL grasp succeeded");
        return true;
    }

    RCLCPP_ERROR(LOGGER, "SIDE_VERTICAL failed for '%s'", r.id.c_str());
    return false;
}

// ============================================================
// DropMonitor — background thread that polls the gripper every
// 500 ms while the robot is transporting an object.
//
// Re-sends a "close to 0.005 m" goal. If the gripper reaches
// the target without stalling (reached_goal=true, stalled=false)
// the object is no longer in the fingers.
//
// Usage:
//   DropMonitor dm;
//   dm.start(gripper_client);   // before moveToPose
//   moveToPose(...);
//   dm.stop();                  // after moveToPose
//   if (dm.object_dropped) { ... }
// ============================================================
struct DropMonitor
{
    std::atomic<bool> running{false};
    std::atomic<bool> object_dropped{false};

    // RAII: stop the thread on destruction so no dangling threads.
    ~DropMonitor() { stop(); }

    void start(const GripperActionClient::SharedPtr& client)
    {
        running        = true;
        object_dropped = false;

        thread_ = std::thread([this, client]()
        {
            while (running.load())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                if (!running.load()) break;

                if (!client->wait_for_action_server(std::chrono::milliseconds(300)))
                    continue;

                GripperCommandAction::Goal goal;
                goal.command.position   = 0.005;
                goal.command.max_effort = 20.0;

                auto gf = client->async_send_goal(goal);
                if (gf.wait_for(std::chrono::seconds(3)) != std::future_status::ready)
                    continue;
                auto gh = gf.get();
                if (!gh) continue;

                auto rf = client->async_get_result(gh);
                if (rf.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
                    continue;
                auto result = rf.get();

                // Fingers fully closed → object is gone
                if (result.result->reached_goal && !result.result->stalled)
                {
                    RCLCPP_WARN(LOGGER,
                        "[DropMonitor] Object dropped — gripper reached %.1fmm without stall",
                        result.result->position * 1000.0);
                    object_dropped = true;
                    break;
                }
            }
        });
    }

    void stop()
    {
        running = false;
        if (thread_.joinable()) thread_.join();
    }

private:
    std::thread thread_;
};

// ============================================================
// processOneObject — picks ONE object and drops it in its bin.
//
// The caller (reactive main loop) is responsible for:
//   • selecting which object to process from the latest perception frame
//   • returning the arm home between calls so the camera has a clear view
//   • retrying or skipping based on the returned PickResult
//
// object_counter is a monotonically increasing ID so collision-object
// names never collide across iterations of the outer loop.
// ============================================================
PickResult processOneObject(
    moveit::planning_interface::MoveGroupInterface& arm,
    const GripperActionClient::SharedPtr& gripper_client,
    moveit::planning_interface::PlanningSceneInterface& psi,
    const object_msgs::msg::Object& obj,
    int object_counter,
    rclcpp::Node::SharedPtr node)
{
    if (BIN_MAP.find(obj.classification) == BIN_MAP.end())
    {
        RCLCPP_WARN(LOGGER, "No bin for class '%s' — skipping", obj.classification.c_str());
        return PickResult::SKIPPED;
    }

    ResolvedObject r;
    r.obj       = obj;
    r.bin_index = BIN_MAP.at(obj.classification);
    r.grasp     = computeGraspGeometry(obj);
    r.id        = "object_" + std::to_string(object_counter);

    RCLCPP_INFO(LOGGER, "Processing '%s' (class=%s, strategy=%s)",
        r.id.c_str(), obj.classification.c_str(), toString(r.grasp.strategy));

    openGripper(gripper_client);

    // Spawn collision object immediately before grasping — pose is the freshest possible.
    spawnCollisionObject(psi, obj, r.id);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    bool grasped = false;
    if (r.grasp.strategy == GraspStrategy::TOP_DOWN)
        grasped = executeTopDownGrasp(arm, gripper_client, psi, r, node);
    // else if (r.grasp.strategy == GraspStrategy::SIDE_HORIZONTAL)
    //     grasped = executeSideHorizontalGrasp(arm, gripper_client, r, node);
    // else if (r.grasp.strategy == GraspStrategy::SIDE_VERTICAL)
    //     grasped = executeSideVerticalGrasp(arm, gripper_client, psi, r, node);
    else
    {
        RCLCPP_WARN(LOGGER, "Unknown strategy — skipping '%s'", r.id.c_str());
        removeObject(psi, r.id);
        return PickResult::SKIPPED;
    }

    if (!grasped)
    {
        RCLCPP_WARN(LOGGER, "Grasp failed for '%s'", r.id.c_str());
        removeObject(psi, r.id);
        return PickResult::GRASP_FAILED;
    }

    // ── Transport to bin with drop monitoring ─────────────────────────────────
    geometry_msgs::msg::Pose bin_pose = getDropOffPose(obj.classification);

    DropMonitor drop_monitor;
    if (!g_sim_mode) drop_monitor.start(gripper_client);

    moveToPose(arm, bin_pose);

    drop_monitor.stop();

    if (drop_monitor.object_dropped.load())
    {
        RCLCPP_WARN(LOGGER, "Object '%s' dropped during transport", r.id.c_str());
        detachObject(arm, r.id);
        removeObject(psi, r.id);
        // The dropped object will appear in the next perception frame
        // and be picked up naturally in the next loop iteration.
        return PickResult::DROPPED;
    }

    // ── Normal release ────────────────────────────────────────────────────────
    openGripper(gripper_client);
    detachObject(arm, r.id);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    removeObject(psi, r.id);

    RCLCPP_INFO(LOGGER, "Object '%s' placed successfully", r.id.c_str());
    return PickResult::SUCCESS;
}

void handleCommand(
    const std::string& cmd,
    moveit::planning_interface::MoveGroupInterface& arm,
    const GripperActionClient::SharedPtr& gripper_client,
    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
{
    if (cmd == "HOME")
    {
        returnHome(arm, gripper_client);
        RCLCPP_INFO(LOGGER, "CMD: HOME");
    }
    else if (cmd == "LOAD_BINS")
    {
        loadBinPoses(g_bin_poses_file);
        RCLCPP_INFO(LOGGER, "CMD: LOAD_BINS");
    }
    else if (cmd.rfind("SAVE_BIN:", 0) == 0)
    {
        std::string label = cmd.substr(9);

        // arm.getCurrentPose() returns in the model frame (base_link), NOT in the
        // planning reference frame ("base").  In the UR URDF, base and base_link are
        // related by a 180° rotation around Z, so using getCurrentPose() directly
        // produces a sign flip on X and Y when the saved pose is later used as a
        // planning target.
        //
        // Fix: look up the EEF transform in "base" via TF2 — the same frame that
        // setPoseReferenceFrame("base") uses — so saved and planned poses are consistent.
        geometry_msgs::msg::Pose p;
        try
        {
            auto tf = tf_buffer->lookupTransform(
                "base",                    // target frame — matches setPoseReferenceFrame
                arm.getEndEffectorLink(),  // source frame (e.g. gripper_tcp)
                tf2::TimePointZero,        // latest available transform
                tf2::durationFromSec(1.0));

            p.position.x  = tf.transform.translation.x;
            p.position.y  = tf.transform.translation.y;
            p.position.z  = tf.transform.translation.z;
            p.orientation = tf.transform.rotation;

            RCLCPP_INFO(LOGGER,
                "EEF in 'base' frame: x=%.4f y=%.4f z=%.4f",
                p.position.x, p.position.y, p.position.z);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_ERROR(LOGGER,
                "Could not look up EEF in 'base' frame: %s — bin NOT saved", ex.what());
            return;
        }

        saveBinPose(label, p, g_bin_poses_file);
        RCLCPP_INFO(LOGGER, "CMD: SAVED %s → %s", label.c_str(), g_bin_poses_file.c_str());
    }
    else if (cmd == "START")
    {
        sequence_requested = true;
        RCLCPP_INFO(LOGGER, "CMD: START");
    }
    else if (cmd == "STOP")
    {
        sequence_requested = false;
        RCLCPP_WARN(LOGGER, "CMD: STOP");
    }
    else
    {
        RCLCPP_WARN(LOGGER, "Unknown command: %s", cmd.c_str());
    }
}


// ============================================================
// Main
// ============================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("motion_controller");

    // Read the "sim" parameter (set via launch argument sim:=true).
    // Controls whether closeGripperForGrasp() uses real stall detection or skips it.
    node->declare_parameter<bool>("sim", false);
    g_sim_mode = node->get_parameter("sim").as_bool();
    RCLCPP_INFO(rclcpp::get_logger("motion_controller"),
        "Simulation mode: %s", g_sim_mode ? "ON (stall detection disabled)" : "OFF");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    std::mutex data_mutex;
    object_msgs::msg::ObjectArray::SharedPtr latest_objects;
    geometry_msgs::msg::PoseArray::SharedPtr latest_goals;
    std::atomic<bool> sequence_requested{false};
    std::atomic<bool> objects_fresh{false};
    std::atomic<std::chrono::steady_clock::time_point> last_perception_time{
        std::chrono::steady_clock::now()};


    // Resolve the installed config path — works after colcon build + sourcing install/setup.bash.
    g_bin_poses_file =
        ament_index_cpp::get_package_share_directory("ur_gripper_demo") +
        "/config/bin_poses.json";

    auto object_sub = node->create_subscription<object_msgs::msg::ObjectArray>(
    "perception/objects", 10,
    [&](const object_msgs::msg::ObjectArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex);
        latest_objects = msg;
        last_perception_time = std::chrono::steady_clock::now();
        if (!objects_fresh) {
            objects_fresh = true;
            RCLCPP_INFO(LOGGER, "First perception message received (%zu objects)",
                        msg->objects.size());
        }
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
        if (!latest_objects) {
            res->success = false;
            res->message = "No perception data yet.";
            return;
        }
        sequence_requested = true;
        res->success = true;
        res->message = "Sequence started.";
        });

    // Wrench subscriber — used by confirmGrasp() on real hardware.
    // In sim this just keeps latest_wrench populated (values are not meaningful).
    auto wrench_sub = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/force_torque_sensor_broadcaster/wrench", 10,
        [](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(wrench_mutex);
            latest_wrench = msg;
        });

    auto gripper_client = rclcpp_action::create_client<GripperCommandAction>(
        node, "/gripper_action_controller/gripper_cmd");

    // TF2 buffer + listener — used by SAVE_BIN to look up the EEF pose in "base"
    // frame, matching the planning reference frame set below.
    auto tf_buffer   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node);



    auto status_pub = node->create_publisher<std_msgs::msg::String>(
        "motion_system/status", 10);


    moveit::planning_interface::MoveGroupInterface arm(node, ARM_GROUP);
    moveit::planning_interface::PlanningSceneInterface psi;

    // RRTConnect produces cleaner trajectory waypoints than BiTRRT.
    // BiTRRT occasionally places waypoints so close together that TOTG
    // time-parameterisation allocates near-zero time intervals, causing
    // the UR driver to report "velocity NNN required in 0.002 seconds".
    arm.setPlannerId("RRTConnectkConfigDefault");
    arm.setMaxVelocityScalingFactor(0.03);
    arm.setMaxAccelerationScalingFactor(0.03);
    arm.setPlanningTime(10.0);
    arm.setGoalJointTolerance(0.01);
    arm.setGoalOrientationTolerance(0.01);
    arm.setGoalPositionTolerance(0.005);

    arm.setPoseReferenceFrame("base"); 

    std::this_thread::sleep_for(std::chrono::seconds(5));

    arm.setStartStateToCurrentState();
    RCLCPP_INFO(LOGGER, "Current EEF: %s", arm.getEndEffectorLink().c_str());

    moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    RCLCPP_INFO(LOGGER, "Loading bin poses from: %s", g_bin_poses_file.c_str());
    loadBinPoses(g_bin_poses_file);

    if (bin_map.empty())
    {
        RCLCPP_WARN(LOGGER, "No bin map loaded — using defaults or blocking execution");
    } else {
        RCLCPP_INFO(LOGGER, "Bin poses loaded at startup");
    }

    auto cmd_sub = node->create_subscription<std_msgs::msg::String>(
        "/motion_system/command", 10,
        [&](const std_msgs::msg::String::SharedPtr msg){
        handleCommand(msg->data, arm, gripper_client, tf_buffer);
    });

    placeGround(arm, psi);
    spawnCameraAssembly(psi);
    returnHome(arm, gripper_client);

    std::ifstream tty("/dev/tty");
    RCLCPP_INFO(LOGGER, "Ready. Press ENTER to start, 'q' to quit.");

    // ── Tuning constants for the reactive loop ────────────────────────────────
    // How many consecutive empty perception frames before declaring the platform clear.
    // Requires the translator to publish empty arrays (not suppress them).
    const int    EMPTY_THRESHOLD        = 3;
    // How long (seconds) to wait for ANY perception message before giving up.
    // This catches "perception node died" — NOT "platform is empty".
    const int    PERCEPTION_TIMEOUT_SEC = 5;
    // How close (metres) two failed-grasp positions must be to count as the same object.
    const double REPEAT_FAIL_RADIUS     = 0.03;
    // How many times to attempt the same location before skipping it for this run.
    const int    MAX_REPEAT_FAILS       = 3;
    // ─────────────────────────────────────────────────────────────────────────

    while (rclcpp::ok())
    {
        // ── Wait for user input ───────────────────────────────────────────────
        std::cout << "\nPress ENTER to start, 'q' to quit and h to return robot to home.\n>> ";
        std::string line;
        std::getline(tty, line);
        char cmd_char = line.empty() ? '\n' : line[0];
        if (cmd_char == 'q') break;
        if (cmd_char == 'h') {
            returnHome(arm, gripper_client);
            std::cout << "\nReturning to Home\n>> ";
            continue;
        }
        if (cmd_char == 'b') {
            std::cout << "\nRemoving object data\n>> ";
            continue;
        }

        // Inline pose-save command: "pose <label>"
        if (line.rfind("pose ", 0) == 0)
        {
            std::string name = line.substr(5);
            geometry_msgs::msg::Pose current_pose;
            try
            {
                auto tf = tf_buffer->lookupTransform(
                    "base", arm.getEndEffectorLink(),
                    tf2::TimePointZero, tf2::durationFromSec(1.0));
                current_pose.position.x  = tf.transform.translation.x;
                current_pose.position.y  = tf.transform.translation.y;
                current_pose.position.z  = tf.transform.translation.z;
                current_pose.orientation = tf.transform.rotation;
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_ERROR(LOGGER, "TF lookup failed: %s", ex.what());
                continue;
            }
            saveBinPose(name, current_pose, g_bin_poses_file);
            RCLCPP_INFO(LOGGER, "Saved pose for '%s'", name.c_str());
            continue;
        }

        bool go = (cmd_char == '\n') || sequence_requested.exchange(false);
        if (!go) continue;

        // ── Move home first so camera has a clear view ────────────────────────
        RCLCPP_INFO(LOGGER, "Moving home to clear camera view...");
        returnHome(arm, gripper_client);

        // ── Reactive per-object pick loop ─────────────────────────────────────
        // Each iteration:
        //   1. Invalidate — force a wait for the next fresh perception message.
        //   2. Wait — block until a new message arrives (or timeout).
        //   3. If empty N times in a row → platform clear, exit loop.
        //   4. Pick the closest object from that message.
        //   5. Execute grasp + transport.
        //   6. Return home so camera view is restored for the next iteration.
        int empty_count    = 0;
        int object_counter = 0;

        // Repeat-fail tracking — detect being stuck on the same unmovable object.
        bool                      has_last_fail     = false;
        geometry_msgs::msg::Point last_fail_pos;
        int                       repeat_fail_count = 0;

        RCLCPP_INFO(LOGGER, "Reactive pick loop started.");

        while (rclcpp::ok())
        {
            // Step 1 — invalidate so we always wait for the next fresh message
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                objects_fresh = false;
            }

            // Step 2 — wait for next perception message
            auto deadline = std::chrono::steady_clock::now()
                          + std::chrono::seconds(PERCEPTION_TIMEOUT_SEC);
            while (!objects_fresh.load() && rclcpp::ok())
            {
                if (std::chrono::steady_clock::now() > deadline)
                {
                    RCLCPP_WARN(LOGGER,
                        "No perception message in %ds — perception node down? Stopping.",
                        PERCEPTION_TIMEOUT_SEC);
                    goto pick_loop_done;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            // Step 3 — snapshot the latest message
            object_msgs::msg::ObjectArray snap;
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                snap = *latest_objects;
            }

            // Step 4 — check for empty scene.
            // Requires translator to publish empty arrays (not suppress them):
            // remove the "if not obj_array.objects: return" guard in
            // plastic_detections_translator.py so empty frames reach us here.
            if (snap.objects.empty())
            {
                if (++empty_count >= EMPTY_THRESHOLD)
                {
                    RCLCPP_INFO(LOGGER,
                        "Platform clear (%d consecutive empty frames) — done.",
                        EMPTY_THRESHOLD);
                    break;
                }
                RCLCPP_INFO(LOGGER, "Empty frame %d/%d", empty_count, EMPTY_THRESHOLD);
                continue;
            }
            empty_count = 0;   // reset on any non-empty frame

            // Step 5 — pick the closest object to the robot base (XY distance)
            auto best = std::min_element(
                snap.objects.begin(), snap.objects.end(),
                [](const auto& a, const auto& b) {
                    return std::hypot(a.pose.position.x, a.pose.position.y)
                         < std::hypot(b.pose.position.x, b.pose.position.y);
                });

            // Step 6 — repeat-fail guard: avoid looping forever on a stuck object
            if (has_last_fail)
            {
                double dx = best->pose.position.x - last_fail_pos.x;
                double dy = best->pose.position.y - last_fail_pos.y;
                if (std::hypot(dx, dy) < REPEAT_FAIL_RADIUS)
                {
                    if (++repeat_fail_count >= MAX_REPEAT_FAILS)
                    {
                        RCLCPP_ERROR(LOGGER,
                            "Object at (%.3f, %.3f) failed %d times — skipping location",
                            best->pose.position.x, best->pose.position.y,
                            repeat_fail_count);
                        has_last_fail     = false;
                        repeat_fail_count = 0;
                        continue;   // next iteration will see the same or a different object
                    }
                    RCLCPP_WARN(LOGGER, "Same location failed %d/%d — retrying",
                        repeat_fail_count, MAX_REPEAT_FAILS);
                }
                else
                {
                    repeat_fail_count = 0;
                    has_last_fail     = false;
                }
            }

            // Step 7 — process this object
            PickResult res = processOneObject(
                arm, gripper_client, psi, *best, object_counter++, node);

            // Step 8 — handle result; always return home to restore camera view
            switch (res)
            {
                case PickResult::SUCCESS:
                    RCLCPP_INFO(LOGGER, "Pick %d succeeded — returning home", object_counter);
                    has_last_fail     = false;
                    repeat_fail_count = 0;
                    returnHome(arm, gripper_client);
                    break;

                case PickResult::DROPPED:
                    // Arm already finished moving to bin (empty-handed).
                    // Dropped object will reappear in next perception frame.
                    RCLCPP_WARN(LOGGER,
                        "Object dropped during transport — returning home; "
                        "dropped object will be picked up next iteration");
                    returnHome(arm, gripper_client);
                    break;

                case PickResult::GRASP_FAILED:
                    last_fail_pos = best->pose.position;
                    has_last_fail = true;
                    RCLCPP_WARN(LOGGER,
                        "Grasp failed — returning home and retrying with fresh perception");
                    returnHome(arm, gripper_client);
                    break;

                case PickResult::SKIPPED:
                    // Unknown class — don't count as a failure
                    break;
            }
        }

        pick_loop_done:
        RCLCPP_INFO(LOGGER,
            "Pick sequence complete. %d objects processed.", object_counter);
        returnHome(arm, gripper_client);
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
