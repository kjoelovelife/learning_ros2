#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


THIS_PACKAGE = "lesson8_cmake" 

def generate_launch_description() -> LaunchDescription:


    # Node
    turtlesim_node = Node(
        package="turtlesim",
        name="turtlesim",
        executable="turtlesim_node",
        output="screen"
    )

    trutle_spawner_node = Node(
        package=THIS_PACKAGE,
        name="turtle_spawner_node",
        executable="turtle_spawner_exe",
        output="screen"
    )

    catch_the_turtle_node = Node(
        package=THIS_PACKAGE,
        name="catch_the_turtle_node",
        executable="catch_the_turtle_exe",
        output="screen"
    )

    launch_description = LaunchDescription()
    action_list = [turtlesim_node, trutle_spawner_node, catch_the_turtle_node]
    for action in action_list:
        launch_description.add_action(action)

    return launch_description