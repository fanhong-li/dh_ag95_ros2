#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate launch description for simple robot display."""
    
    # Get URDF via xacro
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("dh_ag95_gripper"),
            "urdf",
            "dh_ag95_gripper_standalone.urdf.xacro"
        ])
    ])
    
    robot_description = {"robot_description": robot_description_content}
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui"))
    )

    # Joint State Publisher (non-GUI)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_jsp"))
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dh_ag95_gripper"), "rviz", "debug.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description],
        condition=IfCondition(LaunchConfiguration("rviz"))
    )

    # Declare launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start joint_state_publisher_gui",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_jsp",
            default_value="false",
            description="Start joint_state_publisher",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz",
        )
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher,
            joint_state_publisher_gui,
            joint_state_publisher,
            rviz_node,
        ]
    ) 