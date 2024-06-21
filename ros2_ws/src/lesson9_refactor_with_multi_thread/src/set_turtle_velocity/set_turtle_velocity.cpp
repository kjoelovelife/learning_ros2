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

#include "lesson9_refactor_with_multi_thread/set_turtle_velocity.hpp"

SetTurtleVelocity::SetTurtleVelocity(std::string node_name)
        : Node(node_name) {
        
    this->velocity_upper_bound_ = 1.0;
    this->velocity_lower_bound_ = -1.0;
    this->velocity_ = 0.0;
    this->request_parameter_dict_ = {{"velocity",0}};
    this->control_frequency_ = 20.0;
    this->acceleration_ = 0.001;
    this->callback_group_set_parameter_atomically_service_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    this->set_velocity_bound_(this->request_parameter_dict_);

    this->get_describe_parameters_client_ = this->create_client<rcl_interfaces::srv::DescribeParameters>("describe_parameters");
    this->set_turtle_velocity_client_ = this->create_client<rcl_interfaces::srv::SetParametersAtomically>("set_parameters_atomically", rmw_qos_profile_services_default, this->callback_group_set_parameter_atomically_service_);
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000 / this->control_frequency_)), std::bind(&SetTurtleVelocity::callback_timer_, this));




}