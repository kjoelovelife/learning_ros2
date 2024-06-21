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
#ifndef GET_TURTLESIM_BACKGROUND__HPP_
#define GET_TURTLESIM_BACKGROUND__HPP_


#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include "lesson9_refactor_with_multi_thread/turtlesim_background_name.h"
#include <any>
#include <map>
#include <vector>
#include <string>

class GetTurtlesimBackground: public rclcpp::Node{
 public:

  GetTurtlesimBackground(std::string node_name="get_turtlesim_background_node");
  


 private:
  
  void callback_timer_();
  void call_get_parameter_service_(std::vector<std::string> turtlesim_bg_name_vec);

  // template<typename ServiceT>
  // typename rclcpp::Client<ServiceT>::SharedPtr customic_create_client_(const std::string service_name);
  
  // std::any get_background_parameter_client_;

  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_background_parameter_client_;
  
  rclcpp::TimerBase::SharedPtr timer_;

  std::map<std::string, int> request_parameter_dict_;
  
};


#endif
