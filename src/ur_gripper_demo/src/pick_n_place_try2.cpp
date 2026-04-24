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
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

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
// const double RG2_FINGER_MARGIN  = 0.006;  // safety margin each side

const double RG2_FINGER_MARGIN  = 0.00;  // safety margin each side zero for now to test if touchlinks does really alllow touching

const double RG2_FINGER_EXTENSION_MAX = 0.0392;  // 39.2mm from datasheet — extension at fully closed

// If the graspable dimension exceeds this, switch to SIDE strategy
const double TOP_DOWN_MAX_SPAN  = RG2_MAX_SPAN - 0.02;  // 0.090m
// If object height exceeds this, top-down risks wrist collision with object top
const double TOP_DOWN_MAX_HEIGHT = 0.120;  // metres

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
    {"metal",   0},
    {"plastic", 1},
    {"fabric",  2},
};

// Drop-off poses for each bin — position only, orientation is always top-down
struct DropOff {
    double x, y, z;
};

const std::array<DropOff, 3> DROP_OFFS = {{
    {0.37,  0.00, 0.15},   // bin 0 — metal
    {0.37,  0.25, 0.15},   // bin 1 — plastic
    {0.37,  0.50, 0.15},   // bin 2 — fabric
}};

enum class GraspStrategy { TOP_DOWN, SIDE };

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

// geometry_msgs::msg::Pose getDropOffPose(int bin_index){
//     const auto& d = DROP_OFFS[bin_index];
//     geometry_msgs::msg::Pose pose;
//     pose.position.x = d.x;
//     pose.position.y = d.y;
//     pose.position.z = d.z;

//     // Always drop top-down
//     tf2::Quaternion q;
//     q.setRPY(M_PI, 0.0, 0.0);
//     pose.orientation = tf2::toMsg(q);

//     return pose;
// }
geometry_msgs::msg::Pose getDropOffPoseAdaptive(
    moveit::planning_interface::MoveGroupInterface& arm,
    int bin_index)
{
    const auto& d = DROP_OFFS[bin_index];

    geometry_msgs::msg::Pose pose;
    pose.position.x = d.x;
    pose.position.y = d.y;
    pose.position.z = d.z;

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // -------------------------------
    // 1) Try TOP-DOWN orientation
    // -------------------------------
    {
        tf2::Quaternion q;
        q.setRPY(M_PI, 0.0, 0.0);
        pose.orientation = tf2::toMsg(q);

        arm.setPoseTarget(pose);
        if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(LOGGER, "Drop-off %d reachable top-down", bin_index);
            return pose;
        }
    }

    // -------------------------------
    // 2) Try yaw-aligned FORWARD
    // -------------------------------
    {
        double yaw = std::atan2(d.y, d.x);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pose.orientation = tf2::toMsg(q);

        arm.setPoseTarget(pose);
        if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_WARN(LOGGER, "Drop-off %d NOT reachable top-down. Using yaw-aligned fallback.", bin_index);
            return pose;
        }
    }

    // -------------------------------
    // 3) FINAL FALLBACK: free orientation
    // -------------------------------
    {
        // Keep position, but orientation unconstrained
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = arm.getPlanningFrame();
        ps.pose = pose;

        arm.setPositionTarget(d.x, d.y, d.z);

        if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(LOGGER, "Drop-off %d unreachable with any fixed orientation. Using free-orientation fallback.", bin_index);
            return pose; // orientation ignored by IK
        }
    }

    // If even free orientation fails, bin is physically unreachable
    RCLCPP_FATAL(LOGGER, "Drop-off %d is completely unreachable!", bin_index);
    return pose;
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

    //   std::this_thread::sleep_for(std::chrono::milliseconds(500));


    // will also add little slotted raised level 

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
    objects.push_back(makeBox(
        "cam_head",
        CAM_HEAD_X, CAM_HEAD_Y, CAM_HEAD_Z,
        CAM_HEAD_DX, CAM_HEAD_DY, CAM_HEAD_DZ,
        0.0));

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

    // Load mesh
    shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path);
    if (!mesh) {
        RCLCPP_ERROR(LOGGER, "Failed to load mesh from %s", mesh_path.c_str());
        return;
    }

    // Convert to ROS message
    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(mesh, shape_msg);

    shape_msgs::msg::Mesh mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

    // Pose of the trolley
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

void sendGripper(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& pub,
    double width){
    width = std::clamp(width, 0.0, RG2_MAX_SPAN);
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {width};
    pub->publish(msg);
    RCLCPP_INFO(LOGGER, "Gripper → %.3fm", width);
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
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

void spawnCollisionObject(
    moveit::planning_interface::PlanningSceneInterface & psi,
    const object_msgs::msg::Object & obj,
    const std::string & id,
    const std::string & frame_id = "base_link"){
    moveit_msgs::msg::CollisionObject co;
    co.id              = id;
    co.header.frame_id = frame_id;
    co.operation       = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive box;
    box.type       = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {obj.dimensions[0] - RG2_FINGER_MARGIN, obj.dimensions[1] - RG2_FINGER_MARGIN, obj.dimensions[2] - RG2_FINGER_MARGIN};

    co.primitives.push_back(box);
    co.primitive_poses.push_back(obj.pose);

    psi.addCollisionObjects({co});

    RCLCPP_INFO(LOGGER, "Spawned '%s'  [%.3f x %.3f x %.3f]  class=%s",
        id.c_str(),
        obj.dimensions[0], obj.dimensions[1], obj.dimensions[2],
        obj.classification.c_str());
}


GraspGeometry computeGraspGeometry(const object_msgs::msg::Object& obj)
{
    GraspGeometry g;
    const auto& d = obj.dimensions;  // [dx, dy, dz]

    // Find thin axis
    g.thin_axis = 0;
    if (d[1] < d[g.thin_axis]) g.thin_axis = 1;
    if (d[2] < d[g.thin_axis]) g.thin_axis = 2;

    g.object_yaw = extractYaw(obj.pose.orientation);

    // ----------------------------------------
    // Determine grasp strategy
    // ----------------------------------------
    double xy_min = std::min(d[0], d[1]);
    double xy_max = std::max(d[0], d[1]);

    if (xy_min <= TOP_DOWN_MAX_SPAN && d[2] <= TOP_DOWN_MAX_HEIGHT)
        g.strategy = GraspStrategy::TOP_DOWN;
    else
        g.strategy = GraspStrategy::SIDE;

    // ----------------------------------------
    // Compute grip width correctly
    // ----------------------------------------
    if (g.strategy == GraspStrategy::TOP_DOWN)
    {
        // use XY footprint ONLY
        g.grip_width = std::clamp(xy_min - 0.005, 0.0, RG2_MAX_SPAN);
    }
    else
    {
        // SIDE grasp: use best cross-section perpendicular to thin axis
        std::vector<double> cross_section;

        for (int i = 0; i < 3; ++i)
            if (i != g.thin_axis)
                cross_section.push_back(d[i]);

        g.grip_width = std::clamp(*std::min_element(cross_section.begin(),
                                                     cross_section.end()) + 0.005,
                                   0.0,
                                   RG2_MAX_SPAN);
    }

    // Which horizontal axis is longer — fingers should align with it
    if (g.strategy == GraspStrategy::TOP_DOWN)
    {
        if (d[0] >= d[1])
            // X is long axis — fingers align with X, add pose yaw
            g.finger_angle = g.object_yaw;
        else
            // Y is long axis — fingers align with Y, rotate 90deg
            g.finger_angle = g.object_yaw + M_PI / 2.0;
    }

    RCLCPP_INFO(LOGGER,
        "Grasp geometry: strategy=%s thin_axis=%d dims=[%.3f %.3f %.3f] grip_width=%.3f yaw=%.2f",
        g.strategy == GraspStrategy::TOP_DOWN ? "TOP_DOWN" : "SIDE",
        g.thin_axis, d[0], d[1], d[2], g.grip_width, g.object_yaw);

    return g;
}

geometry_msgs::msg::Quaternion computeGraspOrientation(
    const GraspGeometry& g,
    GraspStrategy strategy_override = GraspStrategy::TOP_DOWN,
    bool use_override = false){
    GraspStrategy strat = use_override ? strategy_override : g.strategy;

    tf2::Quaternion q;

    if (strat == GraspStrategy::TOP_DOWN)
    {
        // Base: gripper Z points down (wrist flipped)
        // Rotate about Z by object_yaw + thin_axis offset so fingers align with thin face
        double finger_angle = g.object_yaw;
        if (g.thin_axis == 1) finger_angle += M_PI / 2.0;  // Y-thin needs 90deg offset
        // thin_axis==2 (flat object) or 0: no additional offset

        // q.setRPY(M_PI, 0.0, finger_angle);
        q.setRPY(M_PI, 0.0, g.finger_angle);
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
        arm.execute(plan);
    else
        RCLCPP_WARN(LOGGER, "Pregrasp plan failed for '%s'", r.id.c_str());

    return ok;
}

void attachObject(
    moveit::planning_interface::MoveGroupInterface& arm,
    const std::string& object_id){


    // std::vector<std::string> touch_links = {
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
    //     "gripper_tcp",
    //     "wrist_3_link"
    // };

    // trial, try allowing all links to see if touchlinks actually works:
    std::vector<std::string> touch_links = {
        // UR3e arm
        // "base_link",
        // "base_link_inertia",
        // "shoulder_link",
        // "upper_arm_link",
        // "forearm_link",
        // "wrist_1_link",
        // "wrist_2_link",
        "wrist_3_link",
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
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = traj;
    return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool moveToPose(
    moveit::planning_interface::MoveGroupInterface& arm,
    const geometry_msgs::msg::Pose& pose){
    arm.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS){
        RCLCPP_ERROR(LOGGER, "Joint-space planning failed");
        return false;
    }
    return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}


bool executeTopDownGrasp(
    moveit::planning_interface::MoveGroupInterface& arm,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    const ResolvedObject& r,
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
        sendGripper(gripper_pub, r.grasp.grip_width);
        std::this_thread::sleep_for(std::chrono::milliseconds(800));

        // Raise
        geometry_msgs::msg::Pose raise_pose = grasp_pose;
        raise_pose.position.z = r.obj.pose.position.z
                              + (r.obj.dimensions[2] / 2.0)
                              + PREGRASP_HEIGHT
                              - fingerExtension(r.grasp.grip_width);

        if (!moveCartesian(arm, raise_pose))
        {
            RCLCPP_WARN(LOGGER, "Raise failed on attempt %d — detaching", attempt);
            detachObject(arm, r.id);
            continue;
        }

        RCLCPP_INFO(LOGGER, "Grasp succeeded on attempt %d", attempt);
        return true;
    }

    RCLCPP_ERROR(LOGGER, "Failed to grasp '%s' after %d attempts", r.id.c_str(), max_attempts);
    return false;
}

void MainLoop(
    moveit::planning_interface::MoveGroupInterface& arm,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    moveit::planning_interface::PlanningSceneInterface& psi,
    const object_msgs::msg::ObjectArray& object_array,
    const geometry_msgs::msg::PoseArray& goal_poses){
    // check for empty
    if (object_array.objects.empty())  { RCLCPP_WARN(LOGGER, "No objects."); return; }
    if (goal_poses.poses.empty())      { RCLCPP_WARN(LOGGER, "No goal poses."); return; }

    auto resolved = resolveObjects(object_array);

    // sort by bin
    std::sort(resolved.begin(), resolved.end(),
        [](const ResolvedObject& a, const ResolvedObject& b)
    {
        return a.bin_index < b.bin_index;
    });

    // go to pregrasp:
    for (const auto& r : resolved){

        if (r.grasp.strategy == GraspStrategy::SIDE){
            RCLCPP_INFO(LOGGER, "Skipping '%s' (SIDE strategy not implemented yet)", r.id.c_str());
            // side grasp here 
            continue;
        }
        
        // top down grasp here
        if (!executeTopDownGrasp(arm, gripper_pub, r))
        {
            continue;
        }

        // go to bin
        geometry_msgs::msg::Pose bin_pose = getDropOffPoseAdaptive(arm, r.bin_index);

        moveToPose(arm, bin_pose);

        // Detach and leave it back in the scene
        sendGripper(gripper_pub, 0.110);
        detachObject(arm, r.id);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        removeObject(psi, r.id);


        // go to home or request updated objects data
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

    // arm.setPlannerId("RRTstarkConfigDefault");
    arm.setMaxVelocityScalingFactor(0.3);
    arm.setMaxAccelerationScalingFactor(0.3);
    arm.setPlanningTime(15.0);
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
    spawnCameraAssembly(psi);
    // addTrolleyMesh(arm,psi);
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

        // SPAWN objects:
        // for (std::size_t i = 0; i < objects_copy.objects.size(); ++i) {
        //     spawnCollisionObject(objects_copy.objects[i]);
        // }
        for (std::size_t i = 0; i < objects_copy.objects.size(); ++i) {
            spawnCollisionObject(psi, objects_copy.objects[i], "object_" + std::to_string(i));
        }

        // executePickPlace(arm, gripper_pub, psi, objects_copy, goals_copy);
        MainLoop(arm, gripper_pub, psi, objects_copy, goals_copy);
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}





// possible outputs to GUI
// status messages
// which object is current target

// inputs:
// sort priority. e.g. uniform(fill each bins equally, ) non uniform (bin is sorted). random

// start message  already in
