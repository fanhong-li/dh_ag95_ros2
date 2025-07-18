#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def load_kinematics_config():
    """Load kinematics configuration from YAML file."""
    kinematics_config_path = os.path.join(
        get_package_share_directory("dh_ag95_gripper"),
        "config",
        "kinematics.yaml"
    )
    
    try:
        with open(kinematics_config_path, 'r') as file:
            config = yaml.safe_load(file)
            # Return empty dict if config is None or empty
            return config if config is not None else {}
    except Exception:
        # Return empty dict if file cannot be loaded
        return {}


def generate_moveit_config():
    """Generate MoveIt configuration for the gripper."""
    moveit_config = (
        MoveItConfigsBuilder("dh_ag95_gripper_standalone", package_name="dh_ag95_gripper")
        .robot_description(file_path="urdf/dh_ag95_gripper_standalone.urdf.xacro")
        .robot_description_semantic(file_path="config/dh_ag95_gripper.srdf")
        .trajectory_execution(file_path="config/ros2_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            pipelines=["ompl"], 
            default_planning_pipeline="ompl",
            pipeline_configs={"ompl": "config/ompl_planning.yaml"}
        )
        .planning_scene_monitor(
            publish_robot_description=True, 
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )
    
    return moveit_config


def launch_setup(context, *args, **kwargs):
    """Launch setup function."""
    moveit_config = generate_moveit_config()
    kinematics_config = load_kinematics_config()
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.joint_limits,
        ],
    )

    # MoveIt Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
            moveit_config.joint_limits,
            kinematics_config,
            {"use_sim_time": False},
        ],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dh_ag95_gripper"), "rviz", "moveit.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            {"use_sim_time": False},
        ],
    )

    return [
        robot_state_publisher,
        joint_state_publisher,
        move_group_node,
        rviz_node,
    ]


def generate_launch_description():
    """Generate launch description."""
    
    # Declare launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz configuration file",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    ) 