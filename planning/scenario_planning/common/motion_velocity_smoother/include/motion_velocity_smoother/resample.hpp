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

#ifndef MOTION_VELOCITY_SMOOTHER__RESAMPLE_HPP_
#define MOTION_VELOCITY_SMOOTHER__RESAMPLE_HPP_

#include "motion_velocity_smoother/trajectory_utils.hpp"

#include <autoware_utils/trajectory/trajectory.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <boost/optional.hpp>

namespace motion_velocity_smoother
{
namespace resampling
{
using autoware_planning_msgs::msg::Trajectory;

struct ResampleParam
{
  double max_trajectory_length;         // max length of the objective trajectory for resample
  double min_trajectory_length;         // min length of the objective trajectory for resample
  double resample_time;                 // max time to calculate trajectory length
  double dense_resample_dt;             // resample time interval for dense sampling [s]
  double dense_min_interval_distance;   // minimum points-interval length for dense sampling [m]
  double sparse_resample_dt;            // resample time interval for sparse sampling [s]
  double sparse_min_interval_distance;  // minimum points-interval length for sparse sampling [m]
};

boost::optional<Trajectory> resampleTrajectory(
  const Trajectory & input, const double v_current, const size_t closest_id,
  const ResampleParam & param);

boost::optional<Trajectory> resampleTrajectory(
  const Trajectory & input, const size_t closest_id, const ResampleParam & param,
  const double nominal_ds);
}  // namespace resampling
}  // namespace motion_velocity_smoother

#endif  // MOTION_VELOCITY_SMOOTHER__RESAMPLE_HPP_
