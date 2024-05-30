#!/usr/bin/env Python3
from launch import LaunchDescription
from launch_ros.actions import Node

THIS_PACKAGE = "lesson8_cmake"


def generate_launch_description() -> LaunchDescription:

    turtlesim_node = Node(
        package="turtlesim",
        name="turtlesim",
        executable="turtlesim_node",
        output="screen"
    )

    turtle_teleop_key_node = Node(
        package="turtlesim",
        name="turtle_teleop_key",
        executable="turtle_teleop_key",
        output="screen",
        prefix="gnome-terminal --"
    )

    follow_the_turtle_node = Node(
        package=THIS_PACKAGE,
        name="followe_the_turtle_node",
        executable="follow_the_turtle_exe",
        output="screen"
    )


    launch_description = LaunchDescription()
    action_list = [turtlesim_node, turtle_teleop_key_node, follow_the_turtle_node]
    for action in action_list:
        launch_description.add_action(action)

    return launch_description