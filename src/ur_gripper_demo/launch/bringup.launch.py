#!/usr/bin/env python3
"""
bringup.launch.py — Single command to launch the full plastic-sorting system.

Launches four subsystems in order:
  1. UR3e + OnRobot RG2 driver       (ur_onrobot_control / start_robot.launch.py)
  2. MoveIt + motion controller      (ur_gripper_demo / ur_moveit.launch.py)
  3. OBB perception detector         (plastic_detection / obb_detector_node)
  4. Status GUI                      (ur_gripper_demo / status_gui)

Usage (real robot):
    ros2 launch ur_gripper_demo bringup.launch.py robot_ip:=192.168.0.197

Usage (simulation / fake hardware):
    ros2 launch ur_gripper_demo bringup.launch.py sim:=true

All arguments have sensible defaults — the only one you typically change is robot_ip.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ══════════════════════════════════════════════════════════════════════════
    # Arguments
    # ══════════════════════════════════════════════════════════════════════════
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.0.197",
            description="IP address of the UR robot.",
        ),
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="Set true for fake hardware + no stall detection.",
        ),
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3e",
            description="UR robot type.",
        ),
        DeclareLaunchArgument(
            "onrobot_type",
            default_value="rg2",
            description="OnRobot gripper type.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch RViz alongside MoveIt.",
        ),
        DeclareLaunchArgument(
            "launch_perception",
            default_value="true",
            description="Launch the OBB detector node (disable if running separately).",
        ),
        DeclareLaunchArgument(
            "launch_gui",
            default_value="true",
            description="Launch the status GUI (disable for headless operation).",
        ),
    ]

    robot_ip          = LaunchConfiguration("robot_ip")
    sim               = LaunchConfiguration("sim")
    ur_type           = LaunchConfiguration("ur_type")
    onrobot_type      = LaunchConfiguration("onrobot_type")
    launch_rviz       = LaunchConfiguration("launch_rviz")
    launch_perception = LaunchConfiguration("launch_perception")
    launch_gui        = LaunchConfiguration("launch_gui")

    # use_fake_hardware mirrors sim — when sim:=true we use fake hardware
    use_fake_hardware = PythonExpression(["'true' if '", sim, "' == 'true' else 'false'"])

    # ══════════════════════════════════════════════════════════════════════════
    # 1. UR3e + OnRobot RG2 driver
    # ══════════════════════════════════════════════════════════════════════════
    robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_onrobot_control"),
                "launch",
                "start_robot.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type":           ur_type,
            "onrobot_type":      onrobot_type,
            "robot_ip":          robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "launch_rviz":       "false",  # RViz handled by MoveIt launch below
        }.items(),
    )

    # ══════════════════════════════════════════════════════════════════════════
    # 2. MoveIt + motion controller (delayed 3s for driver to initialise)
    # ══════════════════════════════════════════════════════════════════════════
    moveit_and_controller = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("ur_gripper_demo"),
                        "launch",
                        "ur_moveit.launch.py",
                    ])
                ),
                launch_arguments={
                    "ur_type":      ur_type,
                    "onrobot_type": onrobot_type,
                    "robot_ip":     robot_ip,
                    "demo_type":    "reactive",
                    "sim":          sim,
                    "launch_rviz":  launch_rviz,
                    "launch_servo": "true",
                }.items(),
            ),
        ],
    )

    # ══════════════════════════════════════════════════════════════════════════
    # 3. OBB perception detector (delayed 2s — needs camera only, not MoveIt)
    # ══════════════════════════════════════════════════════════════════════════
    obb_detector = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="plastic_detection",
                executable="obb_detector_node",
                name="obb_detector_node",
                output="screen",
                condition=IfCondition(launch_perception),
            ),
        ],
    )

    # ══════════════════════════════════════════════════════════════════════════
    # 4. Status GUI (delayed 4s — waits for topics to be available)
    # ══════════════════════════════════════════════════════════════════════════
    status_gui = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="ur_gripper_demo",
                executable="status_gui",
                name="status_gui",
                output="screen",
                condition=IfCondition(launch_gui),
            ),
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_driver,
            moveit_and_controller,
            obb_detector,
            status_gui,
        ]
    )
