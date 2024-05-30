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
* Maintainer: Brady Guo
*******************************************************************************/
#include <iostream>
#include <sstream>
#include <string>
#include "lesson5_cmake/subscriber.hpp"

void Subscriber::callback_lunch_info_(const lesson_interfaces::msg::Lunch::SharedPtr msg_ptr)
{
    std::ostringstream meats_and_vegetables_oss {};
    for (auto kind_of_meat: msg_ptr->meats)
    {
        meats_and_vegetables_oss << kind_of_meat << ", ";
    }

    for (auto kind_of_vegetable: msg_ptr->vegetables)
    {
        meats_and_vegetables_oss << kind_of_vegetable << ", ";
    }


    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Oh you eat " << (int)msg_ptr->bowls_of_rice 
        << " bows of rice with " << meats_and_vegetables_oss.str()
        << "and your bow color is with RGBA." << std::endl
        << "R: " << msg_ptr->color_of_bowls.r << std::endl
        << "G: " << msg_ptr->color_of_bowls.g << std::endl
        << "B: " << msg_ptr->color_of_bowls.b << std::endl
        << "A: " << msg_ptr->color_of_bowls.a << std::endl
    );
};