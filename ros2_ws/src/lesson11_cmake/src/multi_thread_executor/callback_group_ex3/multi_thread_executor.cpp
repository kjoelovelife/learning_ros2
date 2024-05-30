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

#include "lesson11_cmake/multi_thread_executor.hpp"

MultiThreadExecutor::MultiThreadExecutor(std::string node_name)
    : Node(node_name) {
    
    this->callback_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->callback_group_2_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    this->callback_group_3_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);


    this->timer_1_ = this->create_wall_timer(seconds(1), 
                                             std::bind(&MultiThreadExecutor::callback_timer_1_, this), 
                                             this->callback_group_1_);       
    this->timer_2_ = this->create_wall_timer(seconds(1), 
                                             std::bind(&MultiThreadExecutor::callback_timer_2_, this), 
                                             this->callback_group_2_);       
    this->timer_3_ = this->create_wall_timer(seconds(1), 
                                             std::bind(&MultiThreadExecutor::callback_timer_3_, this), 
                                             this->callback_group_3_);       

}