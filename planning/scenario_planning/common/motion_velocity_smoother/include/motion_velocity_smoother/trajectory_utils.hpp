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

#ifndef MOTION_VELOCITY_SMOOTHER__TRAJECTORY_UTILS_HPP_
#define MOTION_VELOCITY_SMOOTHER__TRAJECTORY_UTILS_HPP_

#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/trajectory/trajectory.hpp"

#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "boost/optional.hpp"

#include <iostream>
#include <map>
#include <numeric>
#include <tuple>
#include <vector>

namespace motion_velocity_smoother
{
namespace trajectory_utils
{
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPointArray = std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>;
using geometry_msgs::msg::Pose;

TrajectoryPoint calcInterpolatedTrajectoryPoint(
  const TrajectoryPointArray & trajectory, const Pose & target_pose);

boost::optional<TrajectoryPointArray> extractPathAroundIndex(
  const TrajectoryPointArray & trajectory, const size_t index, const double & ahead_length,
  const double & behind_length);

double calcArcLength(const TrajectoryPointArray & trajectory, const int idx1, const int idx2);

std::vector<double> calcArclengthArray(const TrajectoryPointArray & trajectory);

std::vector<double> calcTrajectoryIntervalDistance(const TrajectoryPointArray & trajectory);

boost::optional<std::vector<double>> calcTrajectoryCurvatureFrom3Points(
  const TrajectoryPointArray & trajectory, const size_t & idx_dist);

void setZeroVelocity(TrajectoryPointArray & trajectory);

double getMaxVelocity(const TrajectoryPointArray & trajectory);

double getMaxAbsVelocity(const TrajectoryPointArray & trajectory);

void applyMaximumVelocityLimit(
  const size_t from, const size_t to, const double max_vel, TrajectoryPointArray & trajectory);

boost::optional<size_t> searchZeroVelocityIdx(const TrajectoryPointArray & trajectory);

boost::optional<TrajectoryPointArray> applyLinearInterpolation(
  const std::vector<double> & base_index, const TrajectoryPointArray & base_trajectory,
  const std::vector<double> & out_index, const bool use_spline_for_pose = false);

bool calcStopDistWithJerkConstraints(
  const double v0, const double a0, const double jerk_acc, const double jerk_dec,
  const double min_acc, const double target_vel, std::map<double, double> & jerk_profile,
  double & stop_dist);

bool isValidStopDist(
  const double v_end, const double a_end, const double v_target, const double a_target,
  const double v_margin, const double a_margin);

boost::optional<TrajectoryPointArray> applyDecelFilterWithJerkConstraint(
  const TrajectoryPointArray & input, const size_t start_index, const double v0, const double a0,
  const double min_acc, const double decel_target_vel,
  const std::map<double, double> & jerk_profile);

boost::optional<std::tuple<double, double, double, double>> updateStateWithJerkConstraint(
  const double v0, const double a0, const std::map<double, double> & jerk_profile, const double t);

}  // namespace trajectory_utils
}  // namespace motion_velocity_smoother

#endif  // MOTION_VELOCITY_SMOOTHER__TRAJECTORY_UTILS_HPP_
