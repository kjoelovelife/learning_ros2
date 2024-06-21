#!/usr/bin/env python3
#!/usr/bin/env pytohn3
##########################################################################
# Copyright 2023 BROGENT TECHNOLOGIES INC.
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

from launch import LaunchDescription
from launch_ros.actions import Node



THIS_PACKAGE = "lesson12_cmake"


def generate_launch_description() -> LaunchDescription:


    publisher_node= Node(
        package=THIS_PACKAGE,
        name="publisher_node",
        executable="dds_example_publisher_exe",
        output="screen",
        
    )

    subscriber_node= Node(
    package=THIS_PACKAGE,
    name="subscriber_node",
    executable="dds_example_subscriber_exe",
    output="screen",
    prefix="gnome-terminal --",    
    )


    launch_description = LaunchDescription()
    action_list = [
        publisher_node, subscriber_node,
    ]
    for action in action_list:
        launch_description.add_action(action)

    return launch_description