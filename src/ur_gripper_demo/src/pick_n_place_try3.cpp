// compared to try 2 this file is tailored to the new messages that will be sent from the preception and ML:
// /plastic_detections message format
// [{
//     "pose": {
//       "position": {"x": 22.1, "y": -306.7, "z": -133.1},
//       "orientation": {"qx": 0.0, "qy": 0.0, "qz": 0.71, "qw": 0.71}
//     },
//     "dimensions": {"dx_mm": 65.2, "dy_mm": 210.5, "dz_mm": 63.0},
//     "classification": {"class": "pet_bottle", "confidence": 0.94},
//     "debug": {
//       "z_table_mm": -183.1,
//       "z_approach_mm": -33.1,
//       "angle_deg": 45.0,
//       "angle_rad": 0.7854,
//       "depth_m": 0.312,
//       "dz_source": "depth"
//     }
// }]


// /grip_pose message format
// [{
//     "class": "pet_bottle",
//     "confidence": 0.94,
//     "grip_position": {"x": 22.1, "y": -306.7, "z": -133.1},
//     "grip_orientation": {"qx": 0.0, "qy": 0.0, "qz": 0.71, "qw": 0.71},
//     "jaw_opening_mm": 75.0,
//     "approach": "side_down",
//     "debug": {
//       "angle_deg": 45.0,
//       "is_upright": false,
//       "long_axis_mm": 210.5,
//       "short_axis_mm": 65.2,
//       "jaw_dir": [0.7071, -0.7071, 0.0],
//       "approach_vec": [0.0, 0.0, -1.0]
//     }
// }]

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

// JSON parsing — nlohmann/json (header-only, add to your CMakeLists)
#include <nlohmann/json.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <fstream>

namespace rvt = rviz_visual_tools;
using json = nlohmann::json;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_demo");

// ─── Robot / gripper constants ─────────────────────────────────────────────
const std::string ARM_GROUP     = "ur_onrobot_manipulator";
const std::string GRIPPER_GROUP = "ur_onrobot_gripper";
const double PREGRASP_HEIGHT    = 0.03;    // metres above object centre
const double GRIPPER_OPEN       = 0.110;
const double GRIPPER_CLOSED     = 0.01;
const double SAFE_Z_HEIGHT      = 0.20;

const double RG2_MAX_SPAN              = 0.110;
const double RG2_FINGER_MARGIN         = 0.00;
const double RG2_FINGER_EXTENSION_MAX  = 0.0392;

// ─── Camera assembly collision geometry ────────────────────────────────────
const double CAM_ROD_X      =  0.00;
const double CAM_ROD_Y      =  0.80;
const double CAM_ROD_Z      =  0.40;
const double CAM_ROD_LENGTH =  0.80;
const double CAM_ROD_RADIUS =  0.01;

const double CAM_ARM_X      =  0.00;
const double CAM_ARM_Y      =  0.80;
const double CAM_ARM_Z      =  0.80;
const double CAM_ARM_LENGTH =  0.40;
const double CAM_ARM_RADIUS =  0.025;
const double CAM_ARM_YAW    =  0.00;

const double CAM_HEAD_X     =  0.80;
const double CAM_HEAD_Y     =  0.20;
const double CAM_HEAD_Z     =  1.20;
const double CAM_HEAD_DX    =  0.10;
const double CAM_HEAD_DY    =  0.10;
const double CAM_HEAD_DZ    =  0.08;

// ─── Bin routing ───────────────────────────────────────────────────────────
// Map ML classification strings → bin indices
const std::map<std::string, int> BIN_MAP = {
    {"pet_bottle",  0},
    {"hdpe_bottle", 1},
    // extend as needed
};

struct DropOff { double x, y, z; };

const std::array<DropOff, 2> DROP_OFFS = {{
    {0.37,  0.00, 0.15},   // bin 0 — PET
    {0.37,  0.25, 0.15},   // bin 1 — HDPE
}};

// ─── Unit conversion ────────────────────────────────────────────────────────
inline double mm_to_m(double mm) { return mm * 1e-3; }

// ─── Internal structs ───────────────────────────────────────────────────────

// Populated directly from /grip_pose JSON
struct GraspGeometry
{
    // "top_down", "side_down", "side" — from grip_pose.approach
    std::string approach;

    // Gripper jaw opening in metres (converted from jaw_opening_mm)
    double grip_width;

    // Gripper orientation, ready to use (from grip_pose.grip_orientation)
    geometry_msgs::msg::Quaternion orientation;

    // Jaw direction unit vector (for diagnostics / future use)
    std::array<double, 3> jaw_dir;
};

struct DetectedObject
{
    // Position in metres (converted from mm)
    geometry_msgs::msg::Pose pose;

    // Bounding box in metres [dx, dy, dz]
    // dz falls back to dy (long axis) if depth measurement failed
    std::array<double, 3> dimensions;

    // Classification
    std::string classification;   // "pet_bottle" / "hdpe_bottle"
    double      confidence;
};

struct ResolvedObject
{
    DetectedObject obj;
    GraspGeometry  grasp;
    int            bin_index;
    std::string    id;    // unique scene ID, e.g. "object_0"
};


// ============================================================
// JSON parsing helpers
// ============================================================

/**
 * Parse /plastic_detections JSON string into DetectedObject list.
 * Positions are published in mm → converted to metres here.
 * dz_mm is optional; if absent, falls back to dy_mm (the long axis).
 */
std::vector<DetectedObject> parsePlasticDetections(const std::string& json_str)
{
    std::vector<DetectedObject> out;
    try
    {
        auto arr = json::parse(json_str);
        for (const auto& item : arr)
        {
            DetectedObject d;

            // --- pose (mm → m) ---
            const auto& pos = item["pose"]["position"];
            d.pose.position.x = mm_to_m(pos["x"].get<double>());
            d.pose.position.y = mm_to_m(pos["y"].get<double>());
            d.pose.position.z = mm_to_m(pos["z"].get<double>());

            const auto& ori = item["pose"]["orientation"];
            d.pose.orientation.x = ori["qx"].get<double>();
            d.pose.orientation.y = ori["qy"].get<double>();
            d.pose.orientation.z = ori["qz"].get<double>();
            d.pose.orientation.w = ori["qw"].get<double>();

            // --- dimensions (mm → m) ---
            const auto& dims = item["dimensions"];
            double dx = mm_to_m(dims["dx_mm"].get<double>());
            double dy = mm_to_m(dims["dy_mm"].get<double>());
            // dz_mm is optional; fall back to dy (long axis) if absent
            double dz = dims.contains("dz_mm")
                        ? mm_to_m(dims["dz_mm"].get<double>())
                        : dy;
            d.dimensions = {dx, dy, dz};

            // --- classification ---
            d.classification = item["classification"]["class"].get<std::string>();
            d.confidence     = item["classification"]["confidence"].get<double>();

            out.push_back(d);
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(LOGGER, "parsePlasticDetections: %s", e.what());
    }
    return out;
}

/**
 * Parse /grip_pose JSON string into GraspGeometry list.
 * jaw_opening_mm → metres. Orientation quaternion used directly.
 *
 * The ordering of grip_pose entries is assumed to match plastic_detections
 * (same detection index). If your ML node guarantees this, great. If not,
 * match on class + position proximity instead.
 */
std::vector<GraspGeometry> parseGripPoses(const std::string& json_str)
{
    std::vector<GraspGeometry> out;
    try
    {
        auto arr = json::parse(json_str);
        for (const auto& item : arr)
        {
            GraspGeometry g;

            g.approach    = item["approach"].get<std::string>();
            g.grip_width  = mm_to_m(item["jaw_opening_mm"].get<double>());
            g.grip_width  = std::clamp(g.grip_width, 0.0, RG2_MAX_SPAN);

            const auto& ori = item["grip_orientation"];
            g.orientation.x = ori["qx"].get<double>();
            g.orientation.y = ori["qy"].get<double>();
            g.orientation.z = ori["qz"].get<double>();
            g.orientation.w = ori["qw"].get<double>();

            if (item["debug"].contains("jaw_dir"))
            {
                const auto& jd = item["debug"]["jaw_dir"];
                g.jaw_dir = {jd[0].get<double>(), jd[1].get<double>(), jd[2].get<double>()};
            }

            out.push_back(g);
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(LOGGER, "parseGripPoses: %s", e.what());
    }
    return out;
}


// ============================================================
// Resolve objects: pair detections with grip poses → ResolvedObject
// ============================================================
std::vector<ResolvedObject> resolveObjects(
    const std::vector<DetectedObject>& detections,
    const std::vector<GraspGeometry>&  grip_poses)
{
    std::vector<ResolvedObject> out;

    if (detections.size() != grip_poses.size())
    {
        RCLCPP_WARN(LOGGER,
            "Detection count (%zu) != grip pose count (%zu) — pairing by index up to min",
            detections.size(), grip_poses.size());
    }

    size_t n = std::min(detections.size(), grip_poses.size());

    for (size_t i = 0; i < n; ++i)
    {
        const auto& d = detections[i];
        const auto& g = grip_poses[i];

        auto it = BIN_MAP.find(d.classification);
        if (it == BIN_MAP.end())
        {
            RCLCPP_WARN(LOGGER, "Unknown class '%s' — skipping", d.classification.c_str());
            continue;
        }

        ResolvedObject r;
        r.obj       = d;
        r.grasp     = g;
        r.bin_index = it->second;
        r.id        = "object_" + std::to_string(i);

        RCLCPP_INFO(LOGGER,
            "[%s] class=%s conf=%.2f approach=%s jaw=%.3fm bin=%d",
            r.id.c_str(), d.classification.c_str(), d.confidence,
            g.approach.c_str(), g.grip_width, r.bin_index);

        out.push_back(r);
    }

    return out;
}


// ============================================================
// Planning scene helpers
// ============================================================

void placeGround(
    moveit::planning_interface::MoveGroupInterface& arm,
    moveit::planning_interface::PlanningSceneInterface& psi)
{
    // Ground plane
    {
        moveit_msgs::msg::CollisionObject ground;
        ground.header.frame_id = arm.getPlanningFrame();
        ground.id = "ground";

        shape_msgs::msg::SolidPrimitive prim;
        prim.type       = prim.BOX;
        prim.dimensions = {2.0, 2.0, 0.10};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.z    = -0.055;

        ground.primitives.push_back(prim);
        ground.primitive_poses.push_back(pose);
        ground.operation = ground.ADD;
        psi.addCollisionObjects({ground});
    }

    // Raised platform
    {
        moveit_msgs::msg::CollisionObject platform;
        platform.header.frame_id = arm.getPlanningFrame();
        platform.id = "platform";

        shape_msgs::msg::SolidPrimitive prim;
        prim.type       = prim.BOX;
        prim.dimensions = {0.50, 0.32, 0.01};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.z    = 0.005;
        pose.position.y    = 0.30;

        platform.primitives.push_back(prim);
        platform.primitive_poses.push_back(pose);
        platform.operation = platform.ADD;
        psi.addCollisionObjects({platform});
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void spawnCameraAssembly(
    moveit::planning_interface::PlanningSceneInterface& psi,
    const std::string& frame_id = "base_link")
{
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
        cyl.dimensions = {length, radius};

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

    objects.push_back(makeCylinder(
        "cam_rod",
        CAM_ROD_X, CAM_ROD_Y, CAM_ROD_Z,
        CAM_ROD_LENGTH, CAM_ROD_RADIUS,
        0.0, 0.0, 0.0));

    objects.push_back(makeCylinder(
        "cam_arm",
        CAM_ARM_X, CAM_ARM_Y, CAM_ARM_Z,
        CAM_ARM_LENGTH, CAM_ARM_RADIUS,
        0.0, M_PI / 2.0, CAM_ARM_YAW));

    objects.push_back(makeBox(
        "cam_head",
        CAM_HEAD_X, CAM_HEAD_Y, CAM_HEAD_Z,
        CAM_HEAD_DX, CAM_HEAD_DY, CAM_HEAD_DZ,
        0.0));

    psi.addCollisionObjects(objects);
    RCLCPP_INFO(LOGGER, "Camera assembly spawned (%zu parts)", objects.size());
}

void spawnCollisionObject(
    moveit::planning_interface::PlanningSceneInterface& psi,
    const DetectedObject& obj,
    const std::string& id,
    const std::string& frame_id = "base_link")
{
    moveit_msgs::msg::CollisionObject co;
    co.id              = id;
    co.header.frame_id = frame_id;
    co.operation       = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive box;
    box.type       = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {
        obj.dimensions[0] - RG2_FINGER_MARGIN,
        obj.dimensions[1] - RG2_FINGER_MARGIN,
        obj.dimensions[2] - RG2_FINGER_MARGIN
    };

    co.primitives.push_back(box);
    co.primitive_poses.push_back(obj.pose);

    psi.addCollisionObjects({co});

    RCLCPP_INFO(LOGGER, "Spawned '%s'  [%.3f x %.3f x %.3f]  class=%s",
        id.c_str(),
        obj.dimensions[0], obj.dimensions[1], obj.dimensions[2],
        obj.classification.c_str());
}


// ============================================================
// Motion helpers
// ============================================================

double fingerExtension(double grip_width)
{
    return RG2_FINGER_EXTENSION_MAX * (grip_width / RG2_MAX_SPAN);
}

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

bool moveCartesian(
    moveit::planning_interface::MoveGroupInterface& arm,
    const geometry_msgs::msg::Pose& target,
    double max_step    = 0.002,
    double min_fraction = 0.9)
{
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

    // 1) Top-down
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

    // 2) Yaw-aligned forward
    {
        double yaw = std::atan2(d.y, d.x);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pose.orientation = tf2::toMsg(q);
        arm.setPoseTarget(pose);
        if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_WARN(LOGGER, "Drop-off %d using yaw-aligned fallback", bin_index);
            return pose;
        }
    }

    // 3) Free orientation
    arm.setPositionTarget(d.x, d.y, d.z);
    if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(LOGGER, "Drop-off %d using free-orientation fallback", bin_index);
        return pose;
    }

    RCLCPP_FATAL(LOGGER, "Drop-off %d is completely unreachable!", bin_index);
    return pose;
}

void attachObject(
    moveit::planning_interface::MoveGroupInterface& arm,
    const std::string& object_id)
{
    std::vector<std::string> touch_links = {
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

    arm.attachObject(object_id, arm.getEndEffectorLink(), touch_links);
    RCLCPP_INFO(LOGGER, "Attached '%s' to '%s'",
        object_id.c_str(), arm.getEndEffectorLink().c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void detachObject(
    moveit::planning_interface::MoveGroupInterface& arm,
    const std::string& object_id)
{
    arm.detachObject(object_id);
    RCLCPP_INFO(LOGGER, "Detached '%s'", object_id.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void removeObject(
    moveit::planning_interface::PlanningSceneInterface& psi,
    const std::string& object_id)
{
    psi.removeCollisionObjects({object_id});
    RCLCPP_INFO(LOGGER, "Removed '%s' from planning scene", object_id.c_str());
}


// ============================================================
// Grasp execution
// ============================================================

/**
 * Build the pre-grasp pose above an object.
 * Uses the ML-provided grip orientation directly.
 */
geometry_msgs::msg::Pose buildPregraspPose(const ResolvedObject& r)
{
    geometry_msgs::msg::Pose p;
    p.position.x   = r.obj.pose.position.x;
    p.position.y   = r.obj.pose.position.y;
    p.position.z   = r.obj.pose.position.z
                   + (r.obj.dimensions[2] / 2.0)
                   + PREGRASP_HEIGHT
                   - fingerExtension(r.grasp.grip_width);
    p.orientation  = r.grasp.orientation;
    return p;
}

/**
 * Build the grasp pose (at object centre top surface).
 */
geometry_msgs::msg::Pose buildGraspPose(const ResolvedObject& r)
{
    geometry_msgs::msg::Pose p;
    p.position.x  = r.obj.pose.position.x;
    p.position.y  = r.obj.pose.position.y;
    p.position.z  = r.obj.pose.position.z
                  + (r.obj.dimensions[2] / 2.0)
                  - fingerExtension(r.grasp.grip_width);
    p.orientation = r.grasp.orientation;
    return p;
}

bool executeGrasp(
    moveit::planning_interface::MoveGroupInterface& arm,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    const ResolvedObject& r,
    int max_attempts = 3)
{
    for (int attempt = 1; attempt <= max_attempts; ++attempt)
    {
        RCLCPP_INFO(LOGGER, "Grasp attempt %d/%d for '%s' (approach=%s)",
            attempt, max_attempts, r.id.c_str(), r.grasp.approach.c_str());

        // ── Pre-grasp ──────────────────────────────────────────
        geometry_msgs::msg::Pose pregrasp = buildPregraspPose(r);
        arm.setPoseTarget(pregrasp);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_WARN(LOGGER, "Pre-grasp plan failed on attempt %d", attempt);
            continue;
        }
        arm.execute(plan);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // ── Descend ────────────────────────────────────────────
        geometry_msgs::msg::Pose grasp_pose = buildGraspPose(r);
        if (!moveCartesian(arm, grasp_pose))
        {
            RCLCPP_WARN(LOGGER, "Descend failed on attempt %d", attempt);
            continue;
        }

        // ── Close gripper + attach ─────────────────────────────
        attachObject(arm, r.id);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        sendGripper(gripper_pub, r.grasp.grip_width);
        std::this_thread::sleep_for(std::chrono::milliseconds(800));

        // ── Raise ──────────────────────────────────────────────
        geometry_msgs::msg::Pose raise_pose  = pregrasp;   // same XY + orientation, pregrasp Z
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


// ============================================================
// Main loop
// ============================================================

void MainLoop(
    moveit::planning_interface::MoveGroupInterface& arm,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& gripper_pub,
    moveit::planning_interface::PlanningSceneInterface& psi,
    const std::vector<DetectedObject>& detections,
    const std::vector<GraspGeometry>&  grip_poses)
{
    if (detections.empty()) { RCLCPP_WARN(LOGGER, "No detections."); return; }
    if (grip_poses.empty()) { RCLCPP_WARN(LOGGER, "No grip poses."); return; }

    auto resolved = resolveObjects(detections, grip_poses);

    // Sort by bin index (optional — keeps bins filled in order)
    std::sort(resolved.begin(), resolved.end(),
        [](const ResolvedObject& a, const ResolvedObject& b)
        { return a.bin_index < b.bin_index; });

    for (const auto& r : resolved)
    {
        // Spawn collision object in scene
        spawnCollisionObject(psi, r.obj, r.id);

        // Execute grasp (approach strategy already decided by ML)
        if (!executeGrasp(arm, gripper_pub, r))
        {
            RCLCPP_ERROR(LOGGER, "Skipping '%s' — grasp failed", r.id.c_str());
            removeObject(psi, r.id);
            continue;
        }

        // Move to bin
        geometry_msgs::msg::Pose bin_pose = getDropOffPoseAdaptive(arm, r.bin_index);
        moveToPose(arm, bin_pose);

        // Release
        sendGripper(gripper_pub, GRIPPER_OPEN);
        detachObject(arm, r.id);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        removeObject(psi, r.id);
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

    // Latest parsed data — updated by subscribers
    std::vector<DetectedObject> latest_detections;
    std::vector<GraspGeometry>  latest_grip_poses;
    std::atomic<bool> sequence_requested{false};

    // ── Subscribers ─────────────────────────────────────────────────────────

    // /plastic_detections — JSON string published by OBB Detector node
    auto detections_sub = node->create_subscription<std_msgs::msg::String>(
        "plastic_detections", 10,
        [&](const std_msgs::msg::String::SharedPtr msg) {
            auto parsed = parsePlasticDetections(msg->data);
            std::lock_guard<std::mutex> lock(data_mutex);
            latest_detections = std::move(parsed);
            RCLCPP_INFO(LOGGER, "Received %zu detections", latest_detections.size());
        });

    // /grip_pose — JSON string published by grip planning node
    auto grip_sub = node->create_subscription<std_msgs::msg::String>(
        "grip_pose", 10,
        [&](const std_msgs::msg::String::SharedPtr msg) {
            auto parsed = parseGripPoses(msg->data);
            std::lock_guard<std::mutex> lock(data_mutex);
            latest_grip_poses = std::move(parsed);
            RCLCPP_INFO(LOGGER, "Received %zu grip poses", latest_grip_poses.size());
        });

    // ── Trigger service ─────────────────────────────────────────────────────
    auto trigger_srv = node->create_service<std_srvs::srv::Trigger>(
        "start_pick_place",
        [&](const std_srvs::srv::Trigger::Request::SharedPtr,
            std_srvs::srv::Trigger::Response::SharedPtr res) {
            std::lock_guard<std::mutex> lock(data_mutex);
            if (latest_detections.empty() || latest_grip_poses.empty()) {
                res->success = false;
                res->message = "No perception data yet.";
                return;
            }
            sequence_requested = true;
            res->success = true;
            res->message = "Sequence started.";
        });

    // ── Gripper publisher ────────────────────────────────────────────────────
    auto gripper_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/finger_width_controller/commands", 10);

    // ── MoveIt setup ─────────────────────────────────────────────────────────
    moveit::planning_interface::MoveGroupInterface arm(node, ARM_GROUP);
    moveit::planning_interface::PlanningSceneInterface psi;

    arm.setMaxVelocityScalingFactor(0.3);
    arm.setMaxAccelerationScalingFactor(0.3);
    arm.setPlanningTime(15.0);
    arm.setGoalJointTolerance(0.01);
    arm.setGoalOrientationTolerance(0.01);
    arm.setGoalPositionTolerance(0.005);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    arm.setStartStateToCurrentState();
    RCLCPP_INFO(LOGGER, "Current EEF: %s", arm.getEndEffectorLink().c_str());

    moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // ── Scene setup ──────────────────────────────────────────────────────────
    placeGround(arm, psi);
    spawnCameraAssembly(psi);
    returnHome(arm, gripper_pub);

    // ── Main event loop ──────────────────────────────────────────────────────
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

        std::vector<DetectedObject> detections_copy;
        std::vector<GraspGeometry>  grip_poses_copy;
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            if (latest_detections.empty() || latest_grip_poses.empty()) {
                RCLCPP_WARN(LOGGER, "No perception data yet.");
                continue;
            }
            detections_copy = latest_detections;
            grip_poses_copy = latest_grip_poses;
        }

        MainLoop(arm, gripper_pub, psi, detections_copy, grip_poses_copy);
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
