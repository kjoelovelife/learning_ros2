#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

THIS_PACKAGE = "lesson6_cmake"

def generate_launch_description() -> LaunchDescription:

    turtlesim_node = Node(
        package="turtlesim",
        name="turtlesim1",
        executable="turtlesim_node",
        output="screen"
    )

    move_turtlesim_server_node = Node(
        package=THIS_PACKAGE,
        name="move_turtlesim_server_node",
        executable="move_turtlesim_server_exe",
        output="screen"
    )

    move_turtlesim_client_node = Node(
        package=THIS_PACKAGE,
        name="move_turtlesin_client_node",
        executable="move_turtlesim_client_exe",
        output="screen"
    )

    launch_description = LaunchDescription()
    action_list = [
        turtlesim_node, 
        move_turtlesim_server_node,
        move_turtlesim_client_node
    ]

    for action in action_list:
        launch_description.add_action(action)

    return launch_description