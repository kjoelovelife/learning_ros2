#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:

    turtlesim_node = Node(
        package="turtlesim",
        name="turtlesim",
        executable="turtlesim_node",
        output="screen"
    )

    teleop_twist_keyboard_node = Node(
        package="teleop_twist_keyboard",
        namespace="turtle1",
        name="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        output="screen",
        prefix="gnome-terminal --",
    )

    launch_description = LaunchDescription()
    action_list = [turtlesim_node, teleop_twist_keyboard_node]
    for action in action_list:
        launch_description.add_action(action)
    return launch_description