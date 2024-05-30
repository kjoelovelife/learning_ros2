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

#include "lesson9_cmake/final_exam.hpp"

void FinalExam::call_get_parameters_service_(std::vector<std::string> velocity_name_vec) {
    
    while (!this->velocity_get_parameter_client_->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_WARN(this->get_logger(), "Waiting for Get Parameters Service Server to be up...");
    }
    
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = velocity_name_vec;
    auto future = this->velocity_get_parameter_client_->async_send_request(request);
    
    try{
        auto response = future.get();
        rcl_interfaces::msg::ParameterValue velocity_from_parameter_server = response->values[0];
        if (this->velocity_ == velocity_from_parameter_server.double_value){
            this->has_to_change_color_ = false;
        }
        else{
            this->has_to_change_color_ = true;
            this->velocity_ = velocity_from_parameter_server.double_value;
        }
    }
    catch (const std::exception &e){
        RCLCPP_ERROR(this->get_logger(), "Get Parameters Service call failed.");
    }
    
}