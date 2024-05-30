#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

THIS_PACKAGE = "lesson10_cmake"

def generate_launch_description() -> LaunchDescription:

    retrieve_laser_scan_yaml = os.path.join(
        get_package_share_directory(THIS_PACKAGE),
        "config",
        "retrieve_laser_scan.yaml"
    )
    retrieve_laser_scan_instance = Node(
        package=THIS_PACKAGE,
        name="retrieve_laser_scan_node",
        executable="retrieve_laser_scan_exe",
        output="screen",
        parameters=[retrieve_laser_scan_yaml]
    )

    teleop_twist_keyboard = Node(
        package="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        output="screen",
        prefix="gnome-terminal --",
    )

    launch_description = LaunchDescription()
    action_list = [retrieve_laser_scan_instance, teleop_twist_keyboard]
    for action in action_list:
        launch_description.add_action(action)

    return launch_description