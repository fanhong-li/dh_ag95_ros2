#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    """
    启动夹爪状态显示的launch文件
    """
    
    # 声明launch参数
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "device_port",
            default_value="/dev/ttyUSB0",
            description="Serial device port for gripper communication",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz for visualization",
        )
    )
    
    # 获取URDF
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
    
    # 夹爪驱动节点 (已经发布 joint_states 和 gripper/state)
    gripper_driver = Node(
        package="dh_ag95_gripper",
        executable="dh_ag95_driver",
        name="dh_ag95_gripper_driver",
        output="screen",
        parameters=[{
            "device_port": LaunchConfiguration("device_port"),
            "auto_init": True,
        }],
    )
    
    # RViz配置文件
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("dh_ag95_gripper"), 
        "rviz", 
        "gripper_basic.rviz"
    ])
    
    # RViz节点
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description],
        condition=IfCondition(LaunchConfiguration("rviz"))
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher,
            gripper_driver,
            rviz_node,
        ]
    )
