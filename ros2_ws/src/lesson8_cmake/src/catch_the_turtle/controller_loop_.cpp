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

#include "lesson8_cmake/catch_the_turtle.hpp"

void CatchTheTurtle::controller_loop_(){
    if(!this->get_turtle_pose_ || !this->get_turtle_to_catch_)
        return;
    double distance_x = this->turtle_to_catch_.pose.x - this->pose_.x;
    double distance_y = this->turtle_to_catch_.pose.y - this->pose_.y;
    double distance = std::sqrt(std::pow(distance_x, 2) + std::pow(distance_y, 2));

    geometry_msgs::msg::Twist twist_msg;

    if(distance > 0.5){
        twist_msg.linear.x = this->p_controller_coefficient_ * 0.5 * distance;
        double goal_theta = std::atan2(distance_y,distance_x);
        double theta_to_goal = goal_theta - this->pose_.theta;
        if (theta_to_goal > M_PI)
            theta_to_goal -= (2*M_PI);
        else if (theta_to_goal < -M_PI)
            theta_to_goal += (2*M_PI);
        else {}
        twist_msg.angular.z = this->p_controller_coefficient_ * theta_to_goal;
    }
    else{
        twist_msg.linear.x = 0;
        twist_msg.angular.z = 0;
        // this->call_catch_turtle_service_(turtle_to_catch_.name);
        this->catch_turtle_threads_.push_back(std::make_shared<std::thread>(std::bind(&CatchTheTurtle::call_catch_turtle_service_, this, this->turtle_to_catch_.name)));
        this->turtle_to_catch_.name = "";
    }
    cmd_vel_publisher_->publish(twist_msg);
}