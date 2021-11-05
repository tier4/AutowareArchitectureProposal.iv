// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE_UTILS__TRAJECTORY__CONVERT_HPP_
#define AUTOWARE_UTILS__TRAJECTORY__CONVERT_HPP_

#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/geometry/pose_deviation.hpp"

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"

#include <boost/optional.hpp>

#include <algorithm>
#include <vector>

namespace autoware_utils
{
using autoware_auto_planning_msgs::msg::Trajectory;
using TrajectoryPointArray = std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>;

// convertToTrajectoryByClipping() just clips TrajectoryPointArray up to the capacity of Trajectory.
// Therefore, the error handling out of this function is necessary if the size of
// TrajectoryPointArray greater than the capacity
Trajectory convertToTrajectoryByClipping(const TrajectoryPointArray & trajectory)
{
  Trajectory output{};
  for (const auto & pt : trajectory) {
    output.points.push_back(pt);
    if (output.points.size() >= output.CAPACITY) {
      break;
    }
  }
  return output;
}

TrajectoryPointArray convertToTrajectoryPointArray(const Trajectory & trajectory)
{
  TrajectoryPointArray output(trajectory.points.size());
  std::copy(trajectory.points.begin(), trajectory.points.end(), output.begin());
  return output;
}

}  // namespace autoware_utils

#endif  // AUTOWARE_UTILS__TRAJECTORY__CONVERT_HPP_
