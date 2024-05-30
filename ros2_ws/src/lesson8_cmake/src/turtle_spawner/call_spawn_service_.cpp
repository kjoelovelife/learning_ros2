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
* Maintainer: Brady Guo (brady_guo@brogent.com)
*******************************************************************************/

#include "lesson8_cmake/turtle_spawner.hpp"

void TurtleSpawner::call_spawn_service_(std::string turtle_name, float x, float y, float theta){
    while (!turtle_spawn_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for \"/spawn\" service...");
        }

    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->x = x;
    request->y = y;
    request->theta = theta;
    request->name = turtle_name;

    auto future = turtle_spawn_client_->async_send_request(request);

    try {
        auto response = future.get();
        if (response->name != "") {
            auto new_turtle = lesson_interfaces::msg::Turtle();
            new_turtle.name = response->name;
            new_turtle.pose.x = x;
            new_turtle.pose.y = y;
            new_turtle.pose.theta = theta;
            this->alive_turtles_.turtle_array.push_back(new_turtle);
            this->publish_alive_turtles_();
            RCLCPP_INFO(this->get_logger(), "Turtle %s is now alive.", response->name.c_str());
        }
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed.");
    }



}