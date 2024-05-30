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

#include "lesson9_cmake/turtle_draw_circle_with_parameter.hpp"

void TurtleDrawCircleWithParameter::declare_ros2_parameter_(std::string parameter_name){

    std::stringstream parameter_descriptor_description;
    parameter_descriptor_description << "Set the ";
    parameter_descriptor_description << parameter_name;
    parameter_descriptor_description << " of the turtlesim.";

    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
    rcl_interfaces::msg::FloatingPointRange parameter_descriptor_floating_point_range;

    parameter_descriptor_floating_point_range.from_value =-1.0;
    parameter_descriptor_floating_point_range.to_value = 1.0;
    parameter_descriptor_floating_point_range.step = 0.001;

    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    parameter_descriptor.description = parameter_descriptor_description.str();
    parameter_descriptor.floating_point_range.push_back(parameter_descriptor_floating_point_range); 

    this->declare_parameter(parameter_name, 0.0, parameter_descriptor);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
}