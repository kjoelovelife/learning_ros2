#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:

    THIS_PACKAGE = "lesson7_cmake"

    turtlesim_node = Node(
        package="turtlesim",
        name="turtlesim",
        executable="turtlesim_node",
        output="screen"
    )

    teleop_twist_keyboard_node = Node(
        package=THIS_PACKAGE,
        executable="teleop_in_terminal_exe",
        output="screen",
        prefix="gnome-terminal --",
    )

    launch_description = LaunchDescription()
    action_list = [turtlesim_node, teleop_twist_keyboard_node]
    for action in action_list:
        launch_description.add_action(action)
    return launch_description