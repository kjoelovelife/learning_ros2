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

#include "lesson9_refactor_with_multi_thread/get_turtlesim_background.hpp"

void GetTurtlesimBackground::callback_timer_() {
    
    std::vector<std::string> turtlesim_bg_name_vec;
    for(auto it = this->request_parameter_dict_.begin(); it!=this->request_parameter_dict_.end(); it++){
        turtlesim_bg_name_vec.push_back(it->first);
    }

    this->call_get_parameter_service_(turtlesim_bg_name_vec);


}