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

    remap_topic = [("cmd_vel", "/turtle1/cmd_vel")]
    turtle_draw_circle_node_name = "turtle_draw_circle_with_parameter_node"
    turtle_draw_circle_with_parameter_instance = Node(
        package=THIS_PACKAGE,
        name=turtle_draw_circle_node_name,
        executable="turtle_draw_circle_with_parameter_exe",
        output="screen",
        remappings=remap_topic
    )

    launch_description = LaunchDescription()
    action_list = [
        turtlesim_instance, turtle_draw_circle_with_parameter_instance,
    ]
    for action in action_list:
        launch_description.add_action(action)

    return launch_description