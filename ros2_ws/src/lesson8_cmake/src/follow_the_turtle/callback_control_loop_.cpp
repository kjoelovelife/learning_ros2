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

#include "lesson8_cmake/follow_the_turtle.hpp"

void FollowTheTurtle::callback_control_loop_(){
    if (!this->check_turtle1_up && !this->check_turtle2_up){
        return;
    }
    double distance_x = this->turtle1_pose_.x - this->turtle2_pose_.x;
    double distance_y = this->turtle1_pose_.y - this->turtle2_pose_.y;
    double distance = std::sqrt(std::pow(distance_x, 2) + std::pow(distance_y, 2));
    
    geometry_msgs::msg::Twist twist_msg;
    if(distance > 0.5){
        twist_msg.linear.x = this->p_controller_coefficient_ * 0.5 * distance;
        double goal_theta = std::atan2(distance_y,distance_x);
        double theta_to_goal = goal_theta - this->turtle2_pose_.theta;
        if (theta_to_goal > M_PI)
            theta_to_goal -= (2*M_PI);
        else if (theta_to_goal < -M_PI)
            theta_to_goal += (2*M_PI);
        twist_msg.angular.z = this->p_controller_coefficient_ * theta_to_goal;
    }
    else{
        twist_msg.linear.x = 0;
        twist_msg.angular.z = 0;
    }

    this->turtle2_cmd_vel_publisher_->publish(twist_msg);

    
}