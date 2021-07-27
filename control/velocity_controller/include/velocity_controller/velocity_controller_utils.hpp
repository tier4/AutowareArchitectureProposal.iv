// Copyright 2018 Tier IV, Inc. All rights reserved.
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

#ifndef VELOCITY_CONTROLLER__VELOCITY_CONTROLLER_UTILS_HPP_
#define VELOCITY_CONTROLLER__VELOCITY_CONTROLLER_UTILS_HPP_

#include <cmath>
#include <limits>

#include "boost/optional.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_utils/autoware_utils.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/utils.h"

namespace velocity_controller_utils
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Quaternion;

bool isValidTrajectory(const Trajectory & traj);

double calcStopDistance(
  const Point & current_pos, const Trajectory & traj);

double getPitchByPose(const Quaternion & quaternion);

double getPitchByTraj(
  const Trajectory & msg, const size_t closest_idx, const double wheel_base);

double calcElevationAngle(const Point & p_from, const Point & p_to);

Pose calcPoseAfterTimeDelay(
  const Pose & current_pose, const double delay_time, const double current_vel);

double lerp(const double src_val, const double dst_val, const double ratio);

template<class T>
T lerpXYZ(const T & p_from, const T & p_to, const double ratio)
{
  T point;
  point.x = lerp(p_from.x, p_to.x, ratio);
  point.y = lerp(p_from.y, p_to.y, ratio);
  point.z = lerp(p_from.z, p_to.z, ratio);
  return point;
}

Quaternion lerpOrientation(
  const Quaternion & o_from, const Quaternion & o_to, const double ratio);

template<class T>
TrajectoryPoint lerpTrajectoryPoint(const T & points, const Point & point)
{
  TrajectoryPoint interpolated_point;

  const size_t closest_seg_idx = autoware_utils::findNearestSegmentIndex(points, point);

  const double len_to_interpolated =
    autoware_utils::calcLongitudinalOffsetToSegment(points, closest_seg_idx, point);
  const double len_segment =
    autoware_utils::calcSignedArcLength(points, closest_seg_idx, closest_seg_idx + 1);
  const double interpolate_ratio = len_to_interpolated / len_segment;

  {
    const size_t i = closest_seg_idx;

    interpolated_point.pose.position =
      lerpXYZ(points.at(i).pose.position, points.at(i + 1).pose.position, interpolate_ratio);
    interpolated_point.pose.orientation = lerpOrientation(
      points.at(i).pose.orientation, points.at(i + 1).pose.orientation, interpolate_ratio);
    interpolated_point.twist.linear =
      lerpXYZ(points.at(i).twist.linear, points.at(i + 1).twist.linear, interpolate_ratio);
    interpolated_point.twist.angular =
      lerpXYZ(points.at(i).twist.angular, points.at(i + 1).twist.angular, interpolate_ratio);
    interpolated_point.accel.linear =
      lerpXYZ(points.at(i).accel.linear, points.at(i + 1).accel.linear, interpolate_ratio);
    interpolated_point.accel.angular =
      lerpXYZ(points.at(i).accel.angular, points.at(i + 1).accel.angular, interpolate_ratio);
  }

  return interpolated_point;
}

double applyLimitFilter(const double input_val, const double max_val, const double min_val);
double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double lim_val);
double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double max_val,
  const double min_val);
}  // namespace velocity_controller_utils

#endif  // VELOCITY_CONTROLLER__VELOCITY_CONTROLLER_UTILS_HPP_
