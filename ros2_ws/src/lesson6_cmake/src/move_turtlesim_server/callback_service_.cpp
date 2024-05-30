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
* Maintainer: Brady guo
* Reference : https://google.github.io/styleguide/cppguide.html#Class_Format
*******************************************************************************/

#include "lesson6_cmake/move_turtlesim_server.hpp"

void MoveTurtlesimServer::callback_service_(
    const lesson_interfaces::srv::MoveTurtlesim::Request::SharedPtr request_ptr, 
    const lesson_interfaces::srv::MoveTurtlesim::Response::SharedPtr response_ptr){

  std::string path = request_ptr->path;

  if (path == TurtlesimPath::LINE)
    this->move_line_();
  else if (path == TurtlesimPath::SQUARE)
    this->move_square_();
  else if (path == TurtlesimPath::CIRCLE)
    this->move_circle_();
  else if (path == TurtlesimPath::TRIANGLE)
    this->move_triangle_();
  else {
    RCLCPP_WARN_STREAM(this->get_logger(), "Wrong path!");
    this->stop_();
  }

  response_ptr->successful = true;

}