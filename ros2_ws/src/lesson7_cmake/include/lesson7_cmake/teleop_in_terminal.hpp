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
* Reference : https://google.github.io/styleguide/cppguide.html#Class_Format
*******************************************************************************/
#ifndef TELEOP_IN_TERMINAL__HPP_
#define TELEOP_IN_TERMINAL__HPP_


#include <string>
#include <cmath>
#include <iostream> 
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class TeleopInTerminal: public rclcpp::Node{
 public:
  TeleopInTerminal(std::string node_name="teleop_in_terminal_node");


 private:

    int  get_key_();
    void forward_();
    void backward_();
    void turn_left_();
    void turn_right_();
    void stop_();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;      
    geometry_msgs::msg::Twist twist_;
    char key_ = ' ';
    std::string msg_ = R"(
Control the Turtle!
-------------------------
Moving around:
    w
a   s   d
    x

w/x: moving the turtle forward/backward
a/d: turning the turtle left/right
s  : force stop the turtle
    )";
};


#endif