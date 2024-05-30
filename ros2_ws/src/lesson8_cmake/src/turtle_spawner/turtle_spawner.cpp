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
*******************************************************************************/

#include "lesson8_cmake/turtle_spawner.hpp"

  TurtleSpawner::TurtleSpawner(std::string node_name)
      : Node(node_name) {

    this->turtle_counter_ = 0;
    this->turtlesim_bound_ = 11.0;
    this->turtle_name_prefix_ = "turtle";

    this->turtle_spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
    this->kill_turtle_service_client_ = this->create_client<turtlesim::srv::Kill>("kill");
    this->alive_turtles_publisher_ = this->create_publisher<lesson_interfaces::msg::TurtleArray>("alive_turtles",10);
    this->spawn_turtle_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&TurtleSpawner::callback_spawn_turtle_timer_,this));
    this->catch_turtle_service_ = this->create_service<lesson_interfaces::srv::CatchTurtle>(
      "catch_turtle",std::bind(&TurtleSpawner::callback_catch_turtle_,this,std::placeholders::_1,std::placeholders::_2));
  }
