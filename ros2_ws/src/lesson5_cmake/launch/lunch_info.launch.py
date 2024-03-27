#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

THIS_PACKAGE = "lesson5_cmake"

def generate_launch_description() -> LaunchDescription:


    publisher = Node(
        package=THIS_PACKAGE,
        executable="publisher_exe",
        name="lunch_info_publisher",
        output="screen"
    )

    subscriber = Node(
        package=THIS_PACKAGE,
        executable="subscriber_exe",
        name="lunch_info_subscriber",
        output="screen"
    )

    launch_description = LaunchDescription()
    action_list = [
        publisher, subscriber
    ]

    for action in action_list:
        launch_description.add_action(action)

    return launch_description