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

#include "lesson6_cmake/move_turtlesim_server.hpp"

void MoveTurtlesimServer::move_triangle_() {
  
  RCLCPP_INFO_STREAM(this->get_logger(), "Moving around a triangle!");

  for (int number = 0; number < 3; number++) {
    this->twist_.linear.x = 0.0;
    this->twist_.angular.z = -M_PI_4 - (M_PI_4*number);
    this->publisher_->publish(this->twist_);
    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    this->twist_.linear.x = 2.0;
    this->twist_.angular.z = 0.0;
    this->publisher_->publish(this->twist_);
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
  }
  
  this->stop_();
}