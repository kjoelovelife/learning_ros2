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

#include "lesson9_cmake/set_turtle_velocity.hpp"

void SetTurtleVelocity::set_velocity_bound_(std::map<std::string, int> names_dict) {

    std::vector<std::string> names_dict_to_vec;
    for(auto it = names_dict.begin(); it!=names_dict.end(); it++){
        names_dict_to_vec.push_back(it->first);
    }

    this->set_velocity_bound_threads_.push_back(std::make_shared<std::thread>(std::bind(&SetTurtleVelocity::call_describe_parameter_service_, this, names_dict_to_vec)));

}