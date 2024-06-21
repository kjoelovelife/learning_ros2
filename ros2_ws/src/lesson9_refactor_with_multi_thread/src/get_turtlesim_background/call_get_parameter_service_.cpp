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

#include "lesson9_refactor_with_multi_thread/get_turtlesim_background.hpp"

void GetTurtlesimBackground::call_get_parameter_service_(std::vector<std::string> turtlesim_bg_name_vec) {
    
    while (!this->get_background_parameter_client_->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
    }
    
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = turtlesim_bg_name_vec;
    auto future = this->get_background_parameter_client_->async_send_request(request);
    
    try{
        auto response = future.get();
        for(auto it = this->request_parameter_dict_.begin(); it!= this->request_parameter_dict_.end(); it++){

            rcl_interfaces::msg::ParameterValue parameter = response->values[it->second];
            RCLCPP_INFO(this->get_logger(),"%s : %ld", it->first.c_str(), parameter.integer_value);
        }
    }
    catch (const std::exception &e){
        RCLCPP_ERROR(this->get_logger(), "Service call failed.");
    }
    
}