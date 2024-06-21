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

#include "lesson8_cmake/catch_the_turtle.hpp"

void CatchTheTurtle::call_catch_turtle_service_(std::string turtle_name){
    while (!call_catch_turtle_client_->wait_for_service(std::chrono::seconds(1))){   
        RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
    }
    auto request = std::make_shared<lesson_interfaces::srv::CatchTurtle::Request>();
    request->name = turtle_name;

    auto future = call_catch_turtle_client_->async_send_request(request);

    try{
        auto response = future.get();
        if (!response->success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to catch Turtle %s ", turtle_name.c_str());
        }
    }
    catch (const std::exception &e){
        RCLCPP_ERROR(this->get_logger(), "Service call failed.");
    }

}