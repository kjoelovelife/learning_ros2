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
* Author    : Joe Lin
* Maintainer: Brady guo
* Reference : https://google.github.io/styleguide/cppguide.html#Class_Format
*******************************************************************************/

#include "lesson6_cmake/move_turtlesim_server.hpp"

MoveTurtlesimServer::MoveTurtlesimServer(std::string node_name)
        : Node(node_name) {

    RCLCPP_INFO_STREAM(this->get_logger(), "Initializing...");
    this->service_ = this->create_service<lesson_interfaces::srv::MoveTurtlesim> (
        this->service_name_,
        std::bind(&MoveTurtlesimServer::callback_service_, this, std::placeholders::_1, std::placeholders::_2)
    );
    this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized!!");
}