// Copyright 2018-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RAW_VEHICLE_CMD_CONVERTER__INTERPOLATE_HPP_
#define RAW_VEHICLE_CMD_CONVERTER__INTERPOLATE_HPP_

#include <cmath>
#include <iostream>
#include <vector>

class LinearInterpolate
{
public:
  LinearInterpolate() {}
  ~LinearInterpolate() {}
  static bool interpolate(
    const std::vector<double> & base_index, const std::vector<double> & base_value,
    const double & return_index, double & return_value);
};

#endif  // RAW_VEHICLE_CMD_CONVERTER__INTERPOLATE_HPP_
