#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directories
    pkg_dh_ag95_gripper = FindPackageShare('dh_ag95_gripper')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_driver = LaunchConfiguration('start_driver')
    start_display = LaunchConfiguration('start_display')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_start_driver = DeclareLaunchArgument(
        'start_driver',
        default_value='true',
        description='Start gripper driver'
    )
    
    declare_start_display = DeclareLaunchArgument(
        'start_display',
        default_value='false',
        description='Start display (robot state publisher and RViz)'
    )
    
    # Gripper driver node
    gripper_driver = Node(
        package='dh_ag95_gripper',
        executable='dh_ag95_driver',
        name='dh_ag95_driver',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'port': '/dev/ttyUSB0'},
            {'baud_rate': 115200}
        ],
        condition=IfCondition(start_driver)
    )
    
    # Gripper Action server
    gripper_action_server = Node(
        package='dh_ag95_gripper',
        executable='gripper_action_server',
        name='gripper_action_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Include display launch if requested
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_dh_ag95_gripper,
                'launch',
                'display.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(start_display)
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_start_driver,
        declare_start_display,
        
        # Nodes
        gripper_driver,
        gripper_action_server,
        
        # Display
        display_launch,
    ]) 