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
#ifndef FINAL_EXAM__HPP_
#define FINAL_EXAM__HPP_

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <map>
#include <vector>
#include <string>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/describe_parameters.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include "lesson9_refactor_with_multi_thread/turtlesim_background_name.h"





class FinalExam: public rclcpp::Node{
 public:
   FinalExam(std::string node_name="final_exam_node");


 private:

    void call_get_describe_parameter_service_(std::vector<std::string> velocity_name_vec);
    void callback_timer_();
    void call_get_parameters_service_(std::vector<std::string> velocity_name_vec);
    void call_set_parameter_atomically_service_(rcl_interfaces::msg::Parameter parameter);
    
    int rgb_upper_bound_, rgb_lower_bound_;
    double velocity_upper_bound_, velocity_lower_bound_, velocity_;
    bool has_to_change_color_;
    double change_bg_frequency_;

    std::vector<std::string> get_velocity_name_vec_;

    rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedPtr describe_parameters_client_;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr velocity_get_parameter_client_;
    rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_bg_parameters_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::CallbackGroup::SharedPtr callback_group_get_parameters_service_;
    rclcpp::CallbackGroup::SharedPtr callback_group_set_parameters_atomically_;

    
};


#endif
