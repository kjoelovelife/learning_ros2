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
* Maintainer: Brady Guo
*******************************************************************************/
#ifndef PUBLISHER__HPP_
#define PUBLISHER__HPP_

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "color.hpp"
#include "lesson_interfaces/msg/lunch.hpp"

/**
 * @class Publisher
 * @brief publish std_msgs/String message with timer for testing
*/

class Publisher: public rclcpp::Node
{

public:

    /**
     * @brief constructor
    */
    Publisher(const std::string node_name="publisher_node"): Node(node_name)
    {
        
        this->lunch_.bowls_of_rice = 5;
        this->purple_vector_ = this->color_storer_.get_purple_vector();
        this->lunch_.color_of_bowls.r = this->purple_vector_.at(int(ColorIndex::RED));
        this->lunch_.color_of_bowls.g = this->purple_vector_.at(int(ColorIndex::GREEN));
        this->lunch_.color_of_bowls.b = this->purple_vector_.at(int(ColorIndex::BLUE));
        this->lunch_.color_of_bowls.a = this->purple_vector_.at(int(ColorIndex::ALPHA));

        std::vector<std::string> meats {"fish", "pork"};
        this->lunch_.meats = meats;

        std::vector<std::string> vegetables {"spinach", "tomato"};
        this->lunch_.vegetables = vegetables;

        this->publisher_ptr_ = this->create_publisher<lesson_interfaces::msg::Lunch>("lunch_info", 10);
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&Publisher::callback_wall_timer_, this)
        );
    }

private:

    ColorStorer color_storer_ {};
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<lesson_interfaces::msg::Lunch>::SharedPtr publisher_ptr_;
    lesson_interfaces::msg::Lunch lunch_;
    std::vector<double> purple_vector_ {};
        
    /**
     * @brief publish topic
    */
    void callback_wall_timer_();

};

#endif // PUBLISHER__HPP_