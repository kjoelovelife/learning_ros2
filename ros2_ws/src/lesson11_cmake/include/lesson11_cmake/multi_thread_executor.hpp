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
#ifndef MULTI_THREAD_EXECUTOR__HPP_
#define MULTI_THREAD_EXECUTOR__HPP_


#include <rclcpp/rclcpp.hpp>

using namespace std::chrono;

class MultiThreadExecutor: public rclcpp::Node{
 public:
    MultiThreadExecutor(std::string node_name="multi_thread_executor_node");


 private:
   void callback_timer_1_(){

    RCLCPP_INFO(this->get_logger(), "Timer1 call counts: %d", ++this->count_1_);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(this->get_logger(), "Callback from Timer 1.");
   
   }
   void callback_timer_2_(){

    RCLCPP_INFO(this->get_logger(), "Timer2 call counts: %d", ++this->count_2_);
    
    std::this_thread::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(this->get_logger(), "Callback from Timer 2.");
   
   }
   
   void callback_timer_3_(){
   
    RCLCPP_INFO(this->get_logger(), "Timer3 call counts: %d", ++this->count_3_);

    std::this_thread::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(this->get_logger(), "Callback from Timer 3.");
   
   }

   rclcpp::TimerBase::SharedPtr timer_1_;
   rclcpp::TimerBase::SharedPtr timer_2_;
   rclcpp::TimerBase::SharedPtr timer_3_;

   rclcpp::CallbackGroup::SharedPtr callback_group_1_;
   rclcpp::CallbackGroup::SharedPtr callback_group_2_;
   rclcpp::CallbackGroup::SharedPtr callback_group_3_;

   int count_1_, count_2_, count_3_ = 0; 

};


#endif
