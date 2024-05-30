#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

THIS_PACKAGE = "lesson9_cmake"

def generate_launch_description() -> LaunchDescription:

    turtlesim_node_name = "turtlesim"
    turtlesim_instance = Node(
        package="turtlesim",
        name=turtlesim_node_name,
        executable="turtlesim_node",
        output="screen"
    )

    remap_services = [("get_parameters", f"/{turtlesim_node_name}/get_parameters"),]
    get_turtlesim_background_instance = Node(
        package=THIS_PACKAGE,
        name="get_turtlesim_background_node",
        executable="get_turtlesim_background_exe",
        output="screen",
        remappings=remap_services
    )

    rqt_reconfigure_node = Node(
        package="rqt_reconfigure",
        name="rqt_reconfigure",
        executable="rqt_reconfigure",
        output="screen"
    )


    launch_description = LaunchDescription()
    action_list = [
        turtlesim_instance, get_turtlesim_background_instance,
        rqt_reconfigure_node
    ]
    for action in action_list:
        launch_description.add_action(action)

    return launch_description