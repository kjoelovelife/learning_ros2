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

    # === Please modify the code below ===
    path = Node(

    )


    launch_description = LaunchDescription()
    action_list = [

    ]

    # === end  ===

    for action in action_list:
        launch_description.add_action(action)


    return launch_description