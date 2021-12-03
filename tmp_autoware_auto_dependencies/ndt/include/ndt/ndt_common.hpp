// Copyright 2019 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef NDT__NDT_COMMON_HPP_
#define NDT__NDT_COMMON_HPP_

#include <ndt/visibility_control.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <helper_functions/crtp.hpp>
#include <Eigen/Core>
#include <vector>
#include <algorithm>
#include "common/types.hpp"

using autoware::common::types::float64_t;

namespace autoware
{
namespace localization
{
namespace ndt
{
using Real = float64_t;
}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // NDT__NDT_COMMON_HPP_
