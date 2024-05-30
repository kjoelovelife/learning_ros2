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

void TurtleSpawner::call_kill_turtle_service_(std::string turtle_name){
    while (!this->kill_turtle_service_client_->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_WARN(this->get_logger(), "Waiting for \"/kill\" service...");
    }

    auto request = std::make_shared<turtlesim::srv::Kill::Request>();
    request->name = turtle_name;

    auto future = this->kill_turtle_service_client_->async_send_request(request);

    try {
        future.get();
        for (int i = 0; i < (int)alive_turtles_.turtle_array.size(); i++)
        {
            if (alive_turtles_.turtle_array.at(i).name == turtle_name)
            {
                alive_turtles_.turtle_array.erase(alive_turtles_.turtle_array.begin() + i);
                this->publish_alive_turtles_();
                break;
            }
        }
    }
    catch (const std::exception &e){
        RCLCPP_ERROR(this->get_logger(), "Kill service call failed.");
    }

}