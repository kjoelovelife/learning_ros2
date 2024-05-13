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
* See the License for the specific language governing permissons and
* limitations under the License.
* Author    : Joe Lin
* Maintainer: Brady Guo
* Reference : https://google.github.io/styleguide/cppguide.html#Class_Format
*******************************************************************************/

#include "lesson6_cmake/move_turtlesim_server.hpp"

void MoveTurtlesimServer::stop_() {
  
  RCLCPP_INFO_STREAM(this->get_logger(), "Stop the turtlesim!");
  this->twist_.linear.x = 0.0;
  this->twist_.angular.z = 0.0;
  this->publisher_->publish(this->twist_);
  
}