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
* Maintainer: Brady Guo (brady_guo@brogent.com)
* Reference : https://google.github.io/styleguide/cppguide.html#Class_Format
*******************************************************************************/
#ifndef TURTLE_SPAWNER__HPP_
#define TURTLE_SPAWNER__HPP_


#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <random>
#include <iostream>
#include <vector>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/kill.hpp>
#include "lesson_interfaces/msg/turtle.hpp"
#include "lesson_interfaces/msg/turtle_array.hpp"
#include "lesson_interfaces/srv/catch_turtle.hpp"


class TurtleSpawner: public rclcpp::Node{
 public:

  TurtleSpawner(std::string node_name="turtle_spawner_node");

 private:

  void callback_spawn_turtle_timer_();
  void call_spawn_service_(std::string turtle_name, float x, float y, float theta);
  void publish_alive_turtles_();
  void callback_catch_turtle_(const lesson_interfaces::srv::CatchTurtle::Request::SharedPtr request, 
                              const lesson_interfaces::srv::CatchTurtle::Response::SharedPtr response);
  void call_kill_turtle_service_(std::string turtle_name);

  rclcpp::Publisher<lesson_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_publisher_;
  rclcpp::TimerBase::SharedPtr spawn_turtle_timer_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr turtle_spawn_client_;
  rclcpp::Service<lesson_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_service_;
  rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_turtle_service_client_;

  int turtle_counter_;
  float turtlesim_bound_;
  std::string turtle_name_prefix_;
  lesson_interfaces::msg::TurtleArray alive_turtles_;

  std::vector<std::shared_ptr<std::thread>> spawn_turtle_threads_;
  std::vector<std::shared_ptr<std::thread>> kill_turtle_threads_;



};


#endif
