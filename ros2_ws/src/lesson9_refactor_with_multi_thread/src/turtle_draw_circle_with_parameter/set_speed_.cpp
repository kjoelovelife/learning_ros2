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

#include "lesson9_refactor_with_multi_thread/turtle_draw_circle_with_parameter.hpp"

geometry_msgs::msg::Twist TurtleDrawCircleWithParameter::set_speed_(std::string parameter_name){

    geometry_msgs::msg::Twist twist;
    twist.linear.x = this->get_parameter(parameter_name).as_double();
    twist.angular.z = this->get_parameter(parameter_name).as_double();
    return twist;
    
}