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
#ifndef TURTLE_DRAW_CIRCLE_WITH_PARAMETER__HPP_
#define TURTLE_DRAW_CIRCLE_WITH_PARAMETER__HPP_


#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>


class TurtleDrawCircleWithParameter: public rclcpp::Node{
 public:
    TurtleDrawCircleWithParameter(std::string node_name="turtle_draw_circle_with_parameter_node");


 private:
 
    void declare_ros2_parameter_(std::string parameter_name);
    void callback_timer_();
    geometry_msgs::msg::Twist set_speed_(std::string parameter_name);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string velocity_name_;
  

};


#endif
