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

    remap_topic_service = [
        ("describe_parameters", f"/{turtle_draw_circle_node_name}/describe_parameters"),
        ("set_parameters_atomically", f"/{turtle_draw_circle_node_name}/set_parameters_atomically")
    ]
    set_turtle_velocity_instance = Node(
        package=THIS_PACKAGE,
        name="set_turtle_velocity_node",
        executable="set_turtle_velocity_exe",
        output="screen",
        remappings=remap_topic_service,
    )

    rqt_reconfigure_node = Node(
        package="rqt_reconfigure",
        name="rqt_reconfigure",
        executable="rqt_reconfigure",
        output="screen"
    )

    launch_description = LaunchDescription()
    action_list = [
        turtlesim_instance, turtle_draw_circle_with_parameter_instance,
        set_turtle_velocity_instance, rqt_reconfigure_node
    ]
    for action in action_list:
        launch_description.add_action(action)

    return launch_description