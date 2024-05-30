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

#include "lesson9_cmake/set_turtle_velocity.hpp"

void SetTurtleVelocity::callback_timer_() {
    
    if((this->velocity_ > this->velocity_upper_bound_)  || (this->velocity_ < this->velocity_lower_bound_)){
        this->acceleration_ = this->acceleration_ * -1.0;
    }
    
    rcl_interfaces::msg::Parameter velocity_parameter;
    velocity_parameter.name = "velocity";
    velocity_parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    this->velocity_ = this->velocity_ + this->acceleration_*this->control_frequency_;
    velocity_parameter.value.double_value = this->velocity_;
    this->set_parameter_atomically_threads_.push_back(std::make_shared<std::thread>(std::bind(&SetTurtleVelocity::call_set_parameter_atomically_service_, this, velocity_parameter)));

}