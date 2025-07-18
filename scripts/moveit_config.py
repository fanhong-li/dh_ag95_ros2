#!/usr/bin/env python3

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_moveit_config():
    moveit_config = MoveItConfigsBuilder("dh_ag95_gripper_standalone", package_name="dh_ag95_gripper").to_moveit_configs()
    return moveit_config


def generate_launch_description():
    moveit_config = generate_moveit_config()
    return generate_demo_launch(moveit_config)


if __name__ == "__main__":
    moveit_config = generate_moveit_config()
    print("MoveIt config generated successfully!")
    print(f"URDF: {moveit_config.robot_description}")
    print(f"SRDF: {moveit_config.robot_description_semantic}") 