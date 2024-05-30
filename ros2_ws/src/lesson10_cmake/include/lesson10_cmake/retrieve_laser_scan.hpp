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
#ifndef RETRIEVE_LASER_SCAN__HPP_
#define RETRIEVE_LASER_SCAN__HPP_

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>




class RetrieveLaserScan: public rclcpp::Node{
 public:
   RetrieveLaserScan(std::string node_name="retrieve_laser_scan_node");


 private:
    
    void callback_laser_scan_subscriber_(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    std::string scan_angle_range_parameter_;
    rcl_interfaces::msg::ParameterDescriptor scan_angle_parameter_descriptor_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;

    
};


#endif
