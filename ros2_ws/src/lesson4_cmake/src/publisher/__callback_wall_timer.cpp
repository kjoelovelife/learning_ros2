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
* Author    : Joe Lin
* Maintainer: Joe Lin
*******************************************************************************/

#include "lesson4_cmake/publisher.hpp"

void Publisher::__callback_wall_timer()
{
    this->__counter += 1;
    std::ostringstream oss {};
    oss << "Publish number: " << this->__counter;
    std_msgs::msg::String msg;
    msg.data = oss.str();
    this->__publisher_ptr->publish(msg);
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "I publish message: " << msg.data
    );
};