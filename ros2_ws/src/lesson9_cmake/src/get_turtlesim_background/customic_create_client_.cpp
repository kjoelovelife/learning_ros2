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

#include "lesson9_cmake/get_turtlesim_background.hpp"

template<typename ServiceT>
typename rclcpp::Client<ServiceT>::SharedPtr GetTurtlesimBackground::customic_create_client_(const std::string service_name) {

    auto client = this->create_client<ServiceT>(service_name);
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for %s service...", service_name.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "Find service %s!", service_name.c_str()); 
     
    return client;
}