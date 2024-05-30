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

void RetrieveLaserScan::callback_laser_scan_subscriber_(const sensor_msgs::msg::LaserScan::SharedPtr msg){

    double angle_range = this->get_parameter(this->scan_angle_range_parameter_).as_double();
    
    // short hand condition can be seaprated 
    double radian_range = (angle_range >= 270.0) 
        ? msg->ranges.size() * msg->angle_increment 
        : angle_range*M_PI/180.0;

    int max_index = std::floor(radian_range / msg->angle_increment);
    auto publish_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
    publish_msg->ranges.resize(max_index);
    this->laser_scan_publisher_->publish(*publish_msg);

}