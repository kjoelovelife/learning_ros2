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
#ifndef CATCH_THE_TURTLE__HPP_
#define CATCH_THE_TURTLE__HPP_

// merely keep essential header file
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include "lesson_interfaces/msg/turtle.hpp"
#include "lesson_interfaces/msg/turtle_array.hpp"
#include "lesson_interfaces/srv/catch_turtle.hpp"

// notice space after word
// try adding comment

/**
 * @brief create a node to catch the turtle
 * @param node_name
*/
class CatchTheTurtle: public rclcpp::Node {
 public:
    CatchTheTurtle(std::string node_name="catch_the_turtle_node");

 // try adding comment
 private:
 
  void callback_turtle_pose_(const turtlesim::msg::Pose::SharedPtr msg);


  void callback_alive_turtles_(const lesson_interfaces::msg::TurtleArray msg);
  void controller_loop_();
  void call_catch_turtle_service_(std::string turtle_name);

  // https://google.github.io/styleguide/cppguide.html#Declaration_Order
  // using alphabetical 
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
  rclcpp::Subscription<lesson_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_subscriber_;
  rclcpp::TimerBase::SharedPtr control_loop_timer_;
  rclcpp::Client<lesson_interfaces::srv::CatchTurtle>::SharedPtr call_catch_turtle_client_;

  lesson_interfaces::msg::Turtle turtle_to_catch_;
  turtlesim::msg::Pose pose_;
  double p_controller_coefficient_;
  bool get_turtle_pose_;
  bool get_turtle_to_catch_;

  std::vector<std::shared_ptr<std::thread>> catch_turtle_threads_;



    


};


#endif
