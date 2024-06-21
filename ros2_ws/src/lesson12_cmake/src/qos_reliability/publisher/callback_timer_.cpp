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

#include "lesson12_cmake/qos_reliability/publisher.hpp"

void Publisher::callback_timer_() {
    auto msg = std::make_shared<std_msgs::msg::String>();
    auto now = this->get_clock()->now();
    std::stringstream time_str;
    time_str << now.seconds() << "," << now.nanoseconds();
    msg->data = std::to_string(this->msg_id_) + ":" + time_str.str();
    publisher_->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
    this->msg_id_++;

}