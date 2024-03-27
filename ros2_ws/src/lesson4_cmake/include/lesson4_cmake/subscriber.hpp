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
#ifndef SUBSCRIBER__HPP_
#define SUBSCRIBER__HPP_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @class Subscriber
 * @brief Subscribe "publishe_test" topic
*/
class Subscriber: public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr __subscription_ptr;

    /**
     * @brief publish topic
    */
    void __callback_publish_test(const std_msgs::msg::String::SharedPtr msg);

public:

    /**
     * @brief constructor
    */
    Subscriber(const std::string node_name="subscriber_node"): Node(node_name)
    {
        this->__subscription_ptr = this->create_subscription<std_msgs::msg::String>(
            "publish_test", 
            10,
            std::bind(&Subscriber::__callback_publish_test, this, std::placeholders::_1)
        );
    }

};

#endif // SUBSCRIBER__HPP_