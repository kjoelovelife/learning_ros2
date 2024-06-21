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

void SetTurtleVelocity::call_describe_parameter_service_(std::vector<std::string> names_vector) {
    
    while (!this->get_describe_parameters_client_->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_WARN(this->get_logger(), "Waiting for Describe Parameter Service Server to be up...");
    }

    auto request = std::make_shared<rcl_interfaces::srv::DescribeParameters::Request>();
    request->names = names_vector;
    auto future = this->get_describe_parameters_client_->async_send_request(request);
    
    try{
        auto response = future.get();
        rcl_interfaces::msg::ParameterDescriptor velocity_parameter_descriptor;
        rcl_interfaces::msg::FloatingPointRange velocity_range;
        velocity_parameter_descriptor = response->descriptors[this->request_parameter_dict_["velocity"]];
        velocity_range = velocity_parameter_descriptor.floating_point_range.at(0);
        this->velocity_upper_bound_ = velocity_range.to_value;
        this->velocity_lower_bound_ = velocity_range.from_value;
    }
    catch (const std::exception &e){
        RCLCPP_ERROR(this->get_logger(), "Describe_parameter service call failed.");
    }   

}