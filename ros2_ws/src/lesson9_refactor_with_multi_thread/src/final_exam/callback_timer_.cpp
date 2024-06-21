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

#include "lesson9_refactor_with_multi_thread/final_exam.hpp"

void FinalExam::callback_timer_(){
    
    this->call_get_parameters_service_(this->get_velocity_name_vec_);
    if(this->has_to_change_color_){
        double velocity_ratio = ((this->velocity_ + std::fabs(this->velocity_lower_bound_)) / 
                                 (this->velocity_upper_bound_ + std::fabs(this->velocity_lower_bound_)));
        double rgb_value = (this->rgb_upper_bound_ - this->rgb_lower_bound_) * velocity_ratio;
        rcl_interfaces::msg::Parameter parameter;
        rcl_interfaces::msg::ParameterValue parameter_value;
        parameter_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        parameter_value.integer_value = std::round(rgb_value);
        parameter.name = TurtlesimBackgroundName::BackgroundR;
        parameter.value = parameter_value;

        this->call_set_parameter_atomically_service_(parameter);
    }   
}