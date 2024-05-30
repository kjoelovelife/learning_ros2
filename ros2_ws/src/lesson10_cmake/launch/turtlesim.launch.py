#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

THIS_PACKAGE = "lesson10_cmake"

def generate_launch_description() -> LaunchDescription:


    turtlesim_yaml = os.path.join(
        get_package_share_directory(THIS_PACKAGE),
        "config",
        "turtlesim.yaml"
    )
    turtlesim_node = Node(
        package="turtlesim",
        name="turtlesim",
        executable="turtlesim_node",
        parameters=[turtlesim_yaml]
    )

    launch_description = LaunchDescription()
    action_list = [turtlesim_node]
    for action in action_list:
        launch_description.add_action(action)
    
    return launch_description