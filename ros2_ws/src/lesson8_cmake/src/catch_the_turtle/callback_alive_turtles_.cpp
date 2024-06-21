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

void CatchTheTurtle::callback_alive_turtles_(const lesson_interfaces::msg::TurtleArray msg) {
    if (!msg.turtle_array.empty()){
        this->turtle_to_catch_ = msg.turtle_array.at(0);
        this->get_turtle_to_catch_ = true;
        RCLCPP_INFO(this->get_logger(),"Have a turtle to catch!");
    } 
    else {
        RCLCPP_INFO(this->get_logger(),"NOOOOOOOOOOOOOO turtle to catch!");
        this->get_turtle_to_catch_ = false;

    }
}