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
#ifndef COLOR__HPP_
#define COLOR__HPP_

#include <vector>

enum class ColorIndex: int
{
    RED = 0,
    GREEN,
    BLUE,
    ALPHA
};

class ColorStorer
{
public:
    ColorStorer() = default;
    ~ColorStorer() = default;
    std::vector<double> get_purple_vector(){return this->purple_;}

private:
    std::vector<double> purple_ = {106.0, 90.0, 205.0, 1.0};


};

#endif // COLOR__HPP_