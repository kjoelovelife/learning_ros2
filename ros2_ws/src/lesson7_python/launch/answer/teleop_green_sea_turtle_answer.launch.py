#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:

    turtlesim_node = Node(
        package="turtlesim",
        namespace="green_sea_turtle",
        name="turtlesim",
        executable="turtlesim_node",
        output="screen",
    )
    
    remap_topic = [("cmd_vel", "turtle1/cmd_vel")]
    teleop_twist_keyboard = Node(
        package="teleop_twist_keyboard",
        namespace="green_sea_turtle",
        name="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        output="screen",
        prefix="gnome-terminal --",
        remappings=remap_topic   
    )


    launch_description = LaunchDescription()

    action_list = [turtlesim_node, teleop_twist_keyboard]
    for action in action_list:
        launch_description.add_action(action)


    return launch_description