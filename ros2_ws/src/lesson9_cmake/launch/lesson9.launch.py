#!/usr/bin/env python3
##########################################################################
# Copyright 2024 BROGENT TECHNOLOGIES INC.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Tsung-Meng Guo(brady_guo@brogent.com)
##########################################################################
import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from typing import Dict
from typing import List
from typing import Tuple


THIS_PACKAGE = "lesson9_cmake"

class LaunchArguments:
    ValueIndex = 0
    DescriptionIndex = 1
    FinalExamYAML = "fianl_exam_yaml"
                                                                               
    
    def __init__(self):
        self.__dict = {}
        self.__dict[self.FinalExamYAML] = (
            self.__get_configuration_file_path("lesson9.yaml"), 
            "Configuration file for the final_exam_node"
        )


    def __get_configuration_file_path(self, filename: str):
        return os.path.join(get_package_share_directory(THIS_PACKAGE), "config", filename)

    def get_default_dict(self) -> Dict[str, Tuple[str, str]]:
        return self.__dict

def generate_launch_argument_dict() -> Dict[str, DeclareLaunchArgument]:
    launch_argument_dict = LaunchArguments().get_default_dict()

    declare_name: str
    item: Tuple[str, str]
    for declare_name, item in launch_argument_dict.items():
        launch_argument_dict[declare_name] = DeclareLaunchArgument(
            name=declare_name,
            default_value=TextSubstitution(text=item[LaunchArguments.ValueIndex]),
            description=item[LaunchArguments.DescriptionIndex]
        )

    return launch_argument_dict



def generate_launch_configuration_dict(launch_argument_dict: Dict[str, DeclareLaunchArgument]) -> Dict[str, LaunchConfiguration]:
    launch_configuration_dict = {}
    declare_name: str
    for declare_name in launch_argument_dict.keys():
        launch_configuration_dict[declare_name] = LaunchConfiguration(declare_name)

    return launch_configuration_dict



def generate_node_list(launch_configuration_dict: Dict[str, LaunchConfiguration]) -> List[Node]:
    
    turtlesim_node_name = "turtlesim"
    turtlesim_instance = Node(
        package="turtlesim",
        name="turtlesim",
        executable="turtlesim_node",
        output="screen"
    )

    turtle_draw_circle_remap_topic = [("cmd_vel", "/turtle1/cmd_vel")]
    turtle_draw_circle_node_name = "turtle_draw_circle_with_parameter_node"
    turtle_draw_circle_with_parameter_instance = Node(
        package=THIS_PACKAGE,
        name=turtle_draw_circle_node_name,
        executable="turtle_draw_circle_with_parameter_exe",
        output="screen",
        remappings=turtle_draw_circle_remap_topic
    )

    final_exam_remap_topic_service = [
        ("describe_parameters", f"/{turtle_draw_circle_node_name}/describe_parameters"),
        ("get_parameters", f"/{turtle_draw_circle_node_name}/get_parameters"),
        ("set_parameters_atomically", f"/{turtlesim_node_name}/set_parameters_atomically")
    ]
    final_exam_instance = Node(
        package=THIS_PACKAGE,
        name="final_exam_node",
        executable="final_exam_exe",
        output="screen",
        remappings=final_exam_remap_topic_service,
        parameters=[launch_configuration_dict[LaunchArguments.FinalExamYAML]]
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


    set_turtle_velocity_remap_topic_service = [
        ("describe_parameters", f"/{turtle_draw_circle_node_name}/describe_parameters"),
        ("set_parameters_atomically", f"/{turtle_draw_circle_node_name}/set_parameters_atomically")
    ]
    set_turtle_velocity_instance = Node(
        package=THIS_PACKAGE,
        name="set_turtle_velocity_node",
        executable="set_turtle_velocity_exe",
        output="screen",
        remappings=set_turtle_velocity_remap_topic_service
    )
    node_list = [
        turtlesim_instance,
        turtle_draw_circle_with_parameter_instance,
        final_exam_instance,
        set_turtle_velocity_instance,
        get_turtlesim_background_instance,
        rqt_reconfigure_node
    ]

    return node_list   


def generate_launch_description() -> LaunchDescription:

    launch_argument_dict = generate_launch_argument_dict()
    launch_configuration_dict = generate_launch_configuration_dict(launch_argument_dict)
    node_list = generate_node_list(launch_configuration_dict)

    launch_description = LaunchDescription()
    action_list = [launch_argument for launch_argument in launch_argument_dict.values()]

    for node in node_list:
        action_list.append(node)


    for action in action_list:
        launch_description.add_action(action)

    return launch_description