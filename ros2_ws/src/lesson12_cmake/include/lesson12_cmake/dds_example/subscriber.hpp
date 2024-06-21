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
#ifndef SUBSCRIBER__HPP_
#define SUBSCRIBER__HPP_


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>



class Subscriber: public rclcpp::Node{
 public:
    Subscriber(std::string node_name="subscriber_node");


 private:
 
   void callback_subscriber_(const std_msgs::msg::String::SharedPtr msg);
   void process_dds_test_data_(const std::string data);

   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
   int id_, id_next_, lost_messages_counter_;

    

};


#endif
