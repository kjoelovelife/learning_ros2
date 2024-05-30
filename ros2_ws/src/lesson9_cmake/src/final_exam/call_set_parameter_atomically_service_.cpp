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

void FinalExam::call_set_parameter_atomically_service_(rcl_interfaces::msg::Parameter parameter) {
    
    while (!this->set_bg_parameters_client_->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_WARN(this->get_logger(), "Waiting for Set Parameter Atomically Service Server to be up...");
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    request->parameters.push_back(parameter);

    auto future = this->set_bg_parameters_client_->async_send_request(request);
    
    try{
        auto response = future.get();
        if(response->result.successful)
            RCLCPP_INFO(this->get_logger(), "Change background color");
    }
    catch (const std::exception &e){
        RCLCPP_ERROR(this->get_logger(), "Set_parameter_atomically_service call failed.");
    }   

}