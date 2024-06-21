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

#include "lesson12_cmake/dds_example/subscriber.hpp"

std::vector<std::string> split(const std::string &s, char delimiter) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter)) {
            tokens.push_back(token);
        }
        return tokens;
}

void Subscriber::process_dds_test_data_(const std::string data) {
    auto array = split(data, ':');
    if (array.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Invalid data format: %s", data.c_str());
        return;
    }
    
    try {
        this->id_ = std::stoi(array[0]);
    } catch (const std::invalid_argument &e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid ID format: %s", array[0].c_str());
        return;
    } catch (const std::out_of_range &e) {
        RCLCPP_ERROR(this->get_logger(), "ID out of range: %s", array[0].c_str());
        return;
    }

    if (this->id_ == 0) {
        this->id_next_ = 0;
    }

    if (this->id_ == this->id_next_) {
        RCLCPP_WARN(this->get_logger(), "MESSAGE OK: ID=%d ,Next ID=%d, TOTAL MSG LOST=%d", this->id_, this->id_next_, this->lost_messages_counter_);
    } else {
        if (this->id_next_ != -1) {
            int delta_messages_lost = this->id_ - this->id_next_;
            this->lost_messages_counter_ += delta_messages_lost;
            RCLCPP_ERROR(this->get_logger(), "Message LOST: ID=%d ,Next ID=%d, DELTA MSG LOST=%d, TOTAL MSG LOST=%d", this->id_, this->id_next_, delta_messages_lost, this->lost_messages_counter_);
        }
    }

    this->id_next_ = this->id_ + 1;

    auto time_parts = split(array[1], ',');
    if (time_parts.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Invalid time format: %s", array[1].c_str());
        return;
    }

    int seconds;
    int nano_seconds;
    
    try {
        seconds = std::stoi(time_parts[0]);
        nano_seconds = std::stoll(time_parts[1]);
    } catch (const std::invalid_argument& e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid time format: %s, %s", time_parts[0].c_str(), time_parts[1].c_str());
        return;
    } catch (const std::out_of_range& e) {
        RCLCPP_ERROR(this->get_logger(), "Time out of range: %s, %s", time_parts[0].c_str(), time_parts[1].c_str());
        return; 
    }

    rclcpp::Time time_obj(seconds, nano_seconds, RCL_ROS_TIME);
    rclcpp::Time time_now_obj = this->get_clock()->now();
    rclcpp::Duration delta = time_now_obj - time_obj;

    RCLCPP_INFO(this->get_logger(), "DELTA Between PUB and SUB = %ld", delta.nanoseconds());

}

