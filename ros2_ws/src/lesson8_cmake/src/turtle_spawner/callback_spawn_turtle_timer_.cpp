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
* Maintainer: Brady Guo (brady_guo@brogent.com)
*******************************************************************************/

#include "lesson8_cmake/turtle_spawner.hpp"

void TurtleSpawner::callback_spawn_turtle_timer_(){
    
    turtle_counter_ += 1;
    auto turtle_name = turtle_name_prefix_ + std::to_string(turtle_counter_);
    std::random_device rd;
    std::default_random_engine gen(rd());
    std::uniform_real_distribution<float> dist_xy(0.0,turtlesim_bound_);
    std::uniform_real_distribution<float> dist_theta(-M_PI, M_PI);
    float x = dist_xy(gen);
    float y = dist_xy(gen);
    float theta = dist_theta(gen);
    spawn_turtle_threads_.push_back(
        std::make_shared<std::thread>(
            std::bind(&TurtleSpawner::call_spawn_service_, this, turtle_name, x, y, theta)));

}