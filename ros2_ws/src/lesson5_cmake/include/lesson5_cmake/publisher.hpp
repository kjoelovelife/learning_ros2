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
* Maintainer: Joe Lin
*******************************************************************************/
#ifndef PUBLISHER__HPP_
#define PUBLISHER__HPP_

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "color.hpp"
#include "lesson_interfaces/msg/lunch.hpp"

class Publisher: public rclcpp::Node 
{
private:
    ColorStorer __color_storer {};
    lesson_interfaces::msg::Lunch __launch;
    rclcpp::Publisher<lesson_interfaces::msg::Lunch>::SharedPtr __publisher;
    rclcpp::TimerBase::SharedPtr __timer;
    std::vector<double> __purple_vector {};

    void __callback_wall_timer();

public:

    Publisher(const std::string node_name="publisher_node"): Node(node_name){
        this->__publisher = this->create_publisher<lesson_interfaces::msg::Lunch>("lunch_info", 10);
        this->__timer     = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&Publisher::__callback_wall_timer, this)
        );

        this->__launch.bowls_of_rice = 5;
        this->__purple_vector = this->__color_storer.get_purple_vector();
        this->__launch.color_of_bowls.r = this->__purple_vector.at(int(ColorIndex::RED));
        this->__launch.color_of_bowls.g = this->__purple_vector.at(int(ColorIndex::GREEN));
        this->__launch.color_of_bowls.b = this->__purple_vector.at(int(ColorIndex::BLUE));
        this->__launch.color_of_bowls.a = this->__purple_vector.at(int(ColorIndex::ALPHA));

        std::vector<std::string> meats {"fish", "pork"};
        this->__launch.meats = meats;

        std::vector<std::string> vegetables {"spinach", "tomato"};
        this->__launch.vegetables = vegetables;
    };

};


#endif // PUBLISHER__HPP_