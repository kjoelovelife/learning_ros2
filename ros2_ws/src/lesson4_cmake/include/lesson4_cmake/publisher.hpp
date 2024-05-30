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
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

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
        this->publisher_ptr_ = this->create_publisher<std_msgs::msg::String>("publish_test", 10);
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&Publisher::callback_wall_timer_, this)
        );
    }

private:
    int counter_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ptr_;

    /**
     * @brief publish topic
    */
    void callback_wall_timer_();

};

#endif // PUBLISHER__HPP_