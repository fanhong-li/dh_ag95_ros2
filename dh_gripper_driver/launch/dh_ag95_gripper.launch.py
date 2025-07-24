#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    
    # 声明启动参数
    device_port_arg = DeclareLaunchArgument(
        'device_port',
        default_value='/dev/ttyUSB0',
        description='Serial device port for DH AG95 gripper'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate for serial communication'
    )
    
    gripper_id_arg = DeclareLaunchArgument(
        'gripper_id',
        default_value='1',
        description='Gripper ID for communication'
    )
    
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='false',
        description='Enable test mode (simulation without hardware)'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz for visualization'
    )
    
    # 获取URDF
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("dh_ag95_description"),
            "urdf",
            "dh_ag95.urdf.xacro"
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
        remappings=[
            ('/joint_states', '/gripper/joint_states')  # Listen to gripper joint states
        ],
    )
    
    # DH AG95夹爪驱动节点
    dh_ag95_driver_node = Node(
        package='dh_gripper_driver',
        executable='dh_ag95_driver',
        name='dh_ag95_gripper_driver',
        parameters=[{
            'device_port': LaunchConfiguration('device_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'gripper_id': LaunchConfiguration('gripper_id'),
            'max_position': 100.0,  # AG95最大开口100mm
            'max_force': 100.0,     # AG95最大夹持力100N
        }],
        remappings=[
            ('/joint_states', '/gripper/joint_states')  # Remap to avoid conflict with static publisher
        ],
        output='screen'
    )
    
    # RViz配置文件
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("dh_ag95_description"), 
        "rviz", 
        "view_robot.rviz"
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
    
    return LaunchDescription([
        device_port_arg,
        baudrate_arg,
        gripper_id_arg,
        test_mode_arg,
        rviz_arg,
        robot_state_publisher,
        dh_ag95_driver_node,
        rviz_node,
    ]) 