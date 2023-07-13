#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import TextSubstitution, LaunchConfiguration



def generate_launch_description() -> LaunchDescription:

    # Declare Launch Arguments
    use_turtlesim_arg = DeclareLaunchArgument(
        name="use_turtlesim",
        default_value=TextSubstitution(text="False"),
        description="excutable turtlesim_node or not"
    )

    cmd_vel_arg = DeclareLaunchArgument(
        name="cmd_vel",
        default_value=TextSubstitution(text="/autorideTB3/cmd_vel"),
        description="Topic name for cmd_vel"
    )



    # Configure Launch Arguments
    use_turtlesim_config = LaunchConfiguration(use_turtlesim_arg.name)
    cmd_vel_config       = LaunchConfiguration(cmd_vel_arg.name)

    # Node
    turtlesim_node_name = "turtle1"
    remap_topic = [(f"/{turtlesim_node_name}/cmd_vel", cmd_vel_config)]
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name=turtlesim_node_name,
        output="screen",
        remappings=remap_topic,
        condition=IfCondition(use_turtlesim_config),
    )

    remap_topic = [("cmd_vel", cmd_vel_config)]
    teleop_twist_keyboard_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="gnome-terminal --",
        remappings=remap_topic
    )

    launch_description = LaunchDescription()
    action_list = [
        use_turtlesim_arg, cmd_vel_arg,
        turtlesim_node, teleop_twist_keyboard_node
    ]
    
    for action in action_list:
        launch_description.add_action(action)

    return launch_description