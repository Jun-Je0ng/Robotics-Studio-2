# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# my launch file

import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# *** CHANGED: import load_yaml from ur_onrobot_moveit_config instead of ur_moveit_config ***
from ur_onrobot_moveit_config.launch_common import load_yaml
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.substitutions import PythonExpression

def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    # *** ADDED: onrobot_type argument ***
    onrobot_type = LaunchConfiguration("onrobot_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")

    # General arguments
    # *** CHANGED: description_package -> ur_description_package to match onrobot launch convention ***
    ur_description_package = LaunchConfiguration("ur_description_package")
    description_file = LaunchConfiguration("description_file")
    _publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")


    # *** Changed: Load which node to run
    demo_type = LaunchConfiguration("demo_type")
    sim = LaunchConfiguration("sim")

    # *** CHANGED: use ur_description_package for UR-specific config params ***
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            # *** CHANGED: URDF now comes from ur_onrobot_description package ***
            PathJoinSubstitution([FindPackageShare("ur_onrobot_description"), "urdf", description_file]),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            # *** CHANGED: name is now ur_onrobot to match the combined URDF ***
            "name:=",
            "ur_onrobot",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            # *** ADDED: onrobot_type passed to xacro ***
            "onrobot_type:=",
            onrobot_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
            " ",
            "name:=",
            # *** CHANGED: name matches the combined robot name ***
            "ur_onrobot",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    publish_robot_description_semantic = {
        "publish_robot_description_semantic": _publish_robot_description_semantic
    }

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    # from ur_onrobot_moveit_config.launch_common import load_yaml

    # robot_description_kinematics = {
    #     "robot_description_kinematics": load_yaml(
    #         str(moveit_config_package.perform(context)),
    #         "config/kinematics.yaml"
    #     )
    # }

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", str(moveit_joint_limits_file.perform(context))),
        )
    }

    # Planning Configuration
    # planning_pipeline_config = {
    #     "move_group": {
    #         "planning_plugin": "ompl_interface/OMPLPlanner",
    #         "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
    #         "start_state_max_bounds_error": 0.1,
    #     }
    # }
    # # *** CHANGED: load ompl config from ur_onrobot_moveit_config ***
    # ompl_planning_yaml = load_yaml("ur_onrobot_moveit_config", "config/ompl_planning.yaml")
    # planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Double CHanged to use newer format
    planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_yaml = load_yaml("ur_onrobot_moveit_config", "config/ompl_planning.yaml")
    if ompl_yaml:
        planning_pipeline_config["ompl"].update(ompl_yaml)


    # tryign to add stomp
    # planning_pipeline_config = {
    #     "planning_pipelines": ["ompl", "stomp"],
    #     "default_planning_pipeline": "stomp",  # was ompl before
    #     "ompl": {
    #         "planning_plugin": "ompl_interface/OMPLPlanner",
    #         "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
    #         "start_state_max_bounds_error": 0.1,
    #     },
    #     "stomp": load_yaml("ur_onrobot_moveit_config", "config/stomp_planning.yaml"),
    # }
    # ompl_yaml = load_yaml("ur_onrobot_moveit_config", "config/ompl_planning.yaml")
    # if ompl_yaml:
    #     planning_pipeline_config["ompl"].update(ompl_yaml)

    # Trajectory Execution Configuration
    # *** CHANGED: load controllers from ur_onrobot_moveit_config ***
    controllers_yaml = load_yaml("ur_onrobot_moveit_config", "config/controllers.yaml")
    change_controllers = context.perform_substitution(use_sim_time)
    if change_controllers == "true":
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],  # silence
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "error"],  # silence
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Servo node for realtime control
    # *** CHANGED: servo yaml loaded from ur_onrobot_moveit_config ***
    servo_yaml = load_yaml("ur_onrobot_moveit_config", "config/ur_onrobot_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        arguments=["--ros-args", "--log-level", "warn"], # silence
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics
        ],
        output="screen",
    )

    # Your custom demo node (kept from your original launch file)
    demo_node = Node(
        
        package="ur_gripper_demo",
        executable="gripper_demo_node",
        name="ur3e_moveit_cpp",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", demo_type, "' == 'demo'"])
        ),
        # arguments=["--ros-args", "--log-level", "info"], # silence
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    pick_place_node = Node(
        package="ur_gripper_demo",
        executable="gripper_pick_place",
        name="ur_pick_place",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", demo_type, "' == 'pick_place'"])
        ),
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    unit_test_node = Node(
        package="ur_gripper_demo",
        executable="motion_controller",
        name="motion_controller_test",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", demo_type, "' == 'motion_controller_test'"])
        ),
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            {"sim": sim},
        ],
    )

    reactive_controller_node = Node(
        package="ur_gripper_demo",
        executable="motion_controller_reactive",
        name="motion_controller_reactive",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", demo_type, "' == 'reactive'"])
        ),
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            {"sim": sim},
        ],
    )

    Perception_info_translator = Node(
        package='ur_gripper_demo',
        # executable='plastic_detections_translator',
        executable='plastic_detections_translator_calibrated',
        name='plastic_detections_translator',
        output='screen',
    )

    # Gripper bridge — real robot only.
    # On the real robot the gripper_action_controller is not configured by the
    # OnRobot RG driver, so this bridge presents the standard GripperCommand
    # action interface and detects stall/grip via the effort field of the
    # finger_width joint in /joint_states (set by the hardware interface from
    # the RG status register grip-detected bit).
    # Not started in simulation because the gripper_action_controller works
    # there and two servers on the same action topic would conflict.
    gripper_bridge_node = Node(
        package='ur_gripper_demo',
        executable='gripper_bridge',
        name='gripper_bridge',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", sim, "' == 'false'"])
        ),
    )

    # nodes_to_start = [move_group_node, rviz_node, servo_node, demo_node]
    nodes_to_start = [
        move_group_node,
        # MTC_executor_node,
        rviz_node,
        servo_node,
        demo_node,
        pick_place_node,
        unit_test_node,
        reactive_controller_node,
        Perception_info_translator,
        gripper_bridge_node,
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            # *** KEPT: your broader choices list from Doc 2 ***
            choices=[
                "ur3", "ur5", "ur10", "ur3e", "ur5e", "ur7e",
                "ur10e", "ur12e", "ur16e", "ur8long", "ur15", "ur18", "ur20", "ur30",
            ],
        )
    )
    # *** ADDED: onrobot_type argument ***
    declared_arguments.append(
        DeclareLaunchArgument(
            "onrobot_type",
            description="Type of the OnRobot gripper.",
            choices=["rg2", "rg6"],
            default_value="rg2",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # *** CHANGED: renamed to ur_description_package for clarity ***
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files.",
        )
    )
    # *** CHANGED: default description_file now points to combined ur_onrobot xacro ***
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur_onrobot.urdf.xacro",
            description="URDF/XACRO description file with the robot + gripper.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="True",
            description="Whether to publish the SRDF description on topic /robot_description_semantic.",
        )
    )
    # *** CHANGED: default moveit_config_package now points to ur_onrobot_moveit_config ***
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_onrobot_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files.",
        )
    )
    # *** CHANGED: default moveit_config_file now points to combined SRDF ***
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur_onrobot.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot + gripper.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits that augment or override the values from the URDF robot_description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Servo?")
    )



    declared_arguments.append(
        DeclareLaunchArgument(
            "demo_type",
            default_value="demo",
            description="Which demo node to run",
            choices=["demo", "pick_place", "none", "motion_controller_test", "reactive"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="Set to true when running in simulation: disables gripper stall detection "
                        "and always reports a successful grasp.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])