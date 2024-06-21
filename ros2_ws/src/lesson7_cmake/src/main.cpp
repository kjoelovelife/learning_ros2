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

#include "lesson7_cmake/teleop_in_terminal.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);    
    std::shared_ptr<TeleopInTerminal> node = std::make_shared<TeleopInTerminal>("teleop_in_terminal_node");
    int key {};

    while(rclcpp::ok()){
        // get the pressed key
        key = node->get_key();
        if (key == 'w')
            node->forward();
        else if (key == 'x')
            node->backward();
        else if (key == 'a')
            node->turn_left();
        else if (key == 'd')
            node->turn_right();
        else if (key == 's')
            node->stop();
        else {
            node->stop();
            if (key == '\x03')
            break;
        }
    }
        
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}