#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        namespace="autorideBig",
    )

    turtlesim_teleop = Node(
        package="turtlesim",
        executable="turtle_teleop_key",
        namespace="autorideBig",
        output="screen",
        prefix="gnome-terminal --"
    )

    launch_description = LaunchDescription()
    action_list = [
        turtlesim_node, turtlesim_teleop
    ]

    for action in action_list:
        launch_description.add_action(action)

    return launch_description
    
