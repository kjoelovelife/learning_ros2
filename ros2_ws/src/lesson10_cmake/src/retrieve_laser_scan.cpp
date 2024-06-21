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

#include "lesson10_cmake/retrieve_laser_scan.hpp"

RetrieveLaserScan::RetrieveLaserScan(std::string node_name)
    : Node(node_name) {
    
        this->scan_angle_range_parameter_ = "scan_angle_range";
        rcl_interfaces::msg::FloatingPointRange scan_angle_parameter_floating_point_range;
        scan_angle_parameter_floating_point_range.from_value = 0.0;
        scan_angle_parameter_floating_point_range.to_value = 270.0;
        scan_angle_parameter_floating_point_range.step = 0.001;
        this->scan_angle_parameter_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        this->scan_angle_parameter_descriptor_.floating_point_range.push_back(scan_angle_parameter_floating_point_range);
        this->scan_angle_parameter_descriptor_.description = "Specify the angle range for lidar scanning";
        this->declare_parameter(this->scan_angle_range_parameter_, 270.0, this->scan_angle_parameter_descriptor_);

        this->laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&RetrieveLaserScan::callback_laser_scan_subscriber_, this, std::placeholders::_1));
        this->laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("custom_scan", 10);
}