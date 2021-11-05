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
/**
 * @brief Convert std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> to
 * autoware_auto_planning_msgs::msg::Trajectory. This function is temporarily added for porting to
 * autoware_auto_msgs. We should consider whether to remove this function after the porting is done.
 * @attention This function just clips
 * std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> up to the capacity of Trajectory.
 * Therefore, the error handling out of this function is necessary if the size of the input greater
 * than the capacity.
 * @todo Decide how to handle the situation that we need to use the trajectory with the size of
 * points larger than the capacity. (Tier IV)
 */
autoware_auto_planning_msgs::msg::Trajectory convertToTrajectory(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & trajectory)
{
  autoware_auto_planning_msgs::msg::Trajectory output{};
  for (const auto & pt : trajectory) {
    output.points.push_back(pt);
    if (output.points.size() >= output.CAPACITY) {
      break;
    }
  }
  return output;
}

/**
 * @brief Convert autoware_auto_planning_msgs::msg::Trajectory to
 * std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>.
 */
std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> convertToTrajectoryPointArray(
  const autoware_auto_planning_msgs::msg::Trajectory & trajectory)
{
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> output(trajectory.points.size());
  std::copy(trajectory.points.begin(), trajectory.points.end(), output.begin());
  return output;
}

}  // namespace autoware_utils

#endif  // AUTOWARE_UTILS__TRAJECTORY__CONVERT_HPP_
