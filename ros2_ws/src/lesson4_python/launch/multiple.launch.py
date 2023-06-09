#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    
    publisher = Node(
        package="lesson4_python",
        executable="publisher",
        namespace="",
        name="publisher_node"
    )

    subscriber = Node(
        package="lesson4_python",
        executable="subscriber",
        namespace="",
        name="subscriber_node"
    )

    launch_description = LaunchDescription()
    action_list = [
        publisher, subscriber
    ]

    for action in action_list:
        launch_description.add_action(action)

    return launch_description
    
