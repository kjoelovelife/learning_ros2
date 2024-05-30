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
* Reference : https://google.github.io/styleguide/cppguide.html#Class_Format
*******************************************************************************/
#ifndef MOVE_TURTLESIM_CLIENT__HPP_
#define MOVE_TURTLESIM_CLIENT__HPP_

#include <chrono>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <lesson_interfaces/srv/move_turtlesim.hpp>
#include "turtlesim_path.h"

class MoveTurtlesimClient: public rclcpp::Node{
 public:
    MoveTurtlesimClient(std::string node_name="move_turtlesim_client_node");
    
 private:
   rclcpp::Client<lesson_interfaces::srv::MoveTurtlesim>::SharedPtr client_;      
   std::string service_name_ {"move_turtlesim"};
   std::vector<std::string> paths_ { 
      TurtlesimPath::LINE,
      TurtlesimPath::SQUARE,
      TurtlesimPath::CIRCLE,
      TurtlesimPath::TRIANGLE
    };
};


#endif // MOVE_TURTLESIM_SERVER__HPP_