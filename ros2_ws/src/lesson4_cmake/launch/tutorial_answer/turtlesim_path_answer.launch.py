#!/usr/bin/env python3


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:


    turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim",
        output="screen"
    )

    path = Node(
        package="lesson4_python",
        executable="turtle_draw_circle_answer",
        name="turtle_draw_circle",
        output="screen",
    )


    launch_description = LaunchDescription()
    action_list = [
        turtlesim, path
    ]

    for action in action_list:
        launch_description.add_action(action)


    return launch_description
