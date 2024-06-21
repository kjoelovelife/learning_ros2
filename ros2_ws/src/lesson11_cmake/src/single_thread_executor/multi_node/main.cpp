/*******************************************************************************
* Copyright 2024 BROGENT TECHNOLOGIES INC.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
* Author    : Brady Guo
* Maintainer: Brady Guo
*******************************************************************************/

#include "lesson11_cmake/single_thread_executor.hpp"
#include "lesson11_cmake/lesson11_node2.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);    
    rclcpp::Node::SharedPtr node1 = std::make_shared<SingleThreadExecutor>("node1");
    rclcpp::Node::SharedPtr node2 = std::make_shared<Lesson11Node2>("node2");
    rclcpp::executors::SingleThreadedExecutor executor {};
    executor.add_node(node1);
    executor.add_node(node2);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}