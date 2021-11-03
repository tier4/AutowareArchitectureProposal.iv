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

#include <autoware_utils/autoware_utils.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/optional.hpp>

#include <tf2/utils.h>

#include <cmath>
#include <limits>

namespace velocity_controller_utils
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Quaternion;

/**
 * @brief check if trajectory is invalid or not
 */
bool isValidTrajectory(const Trajectory & traj);

/**
 * @brief calculate distance to stopline from current vehicle position where velocity is 0
 */
double calcStopDistance(const Point & current_pos, const Trajectory & traj);

/**
 * @brief calculate pitch angle from estimated current pose
 */
double getPitchByPose(const Quaternion & quaternion);

/**
 * @brief calculate pitch angle from trajectory on map
 * @param [in] trajectory input trajectory
 * @param [in] closest_idx nearest index to current vehicle position
 * @param [in] wheel_base length of wheel base
 */
double getPitchByTraj(
  const Trajectory & trajectory, const size_t closest_idx, const double wheel_base);

/**
 * @brief calculate elevation angle
 */
double calcElevationAngle(const Point & p_from, const Point & p_to);

/**
 * @brief calculate vehicle pose after time delay by moving the vehicle at current velocity for
 * delayed time
 */
Pose calcPoseAfterTimeDelay(
  const Pose & current_pose, const double delay_time, const double current_vel);

/**
 * @brief apply linear interpolation
 * @param [in] v_from first value
 * @param [in] v_to second value
 * @param [in] ratio ratio between o_from and o_to for interpolation
 */
double lerp(const double v_from, const double v_to, const double ratio);

/**
 * @brief apply linear interpolation to position
 * @param [in] p_from first position
 * @param [in] p_to second position
 * @param [in] ratio ratio between o_from and o_to for interpolation
 */
template <class T>
T lerpXYZ(const T & p_from, const T & p_to, const double ratio)
{
  T point;
  point.x = lerp(p_from.x, p_to.x, ratio);
  point.y = lerp(p_from.y, p_to.y, ratio);
  point.z = lerp(p_from.z, p_to.z, ratio);
  return point;
}

/**
 * @brief apply linear interpolation to orientation
 * @param [in] o_from first orientation
 * @param [in] o_to second orientation
 * @param [in] ratio ratio between o_from and o_to for interpolation
 */
Quaternion lerpOrientation(const Quaternion & o_from, const Quaternion & o_to, const double ratio);

/**
 * @brief apply linear interpolation to trajectory point that is nearest to a certain point
 * @param [in] points trajectory points
 * @param [in] point Interpolated point is nearest to this point.
 */
template <class T>
TrajectoryPoint lerpTrajectoryPoint(const T & points, const Point & point)
{
  TrajectoryPoint interpolated_point;

  const size_t nearest_seg_idx = autoware_utils::findNearestSegmentIndex(points, point);

  const double len_to_interpolated =
    autoware_utils::calcLongitudinalOffsetToSegment(points, nearest_seg_idx, point);
  const double len_segment =
    autoware_utils::calcSignedArcLength(points, nearest_seg_idx, nearest_seg_idx + 1);
  const double interpolate_ratio = std::clamp(len_to_interpolated / len_segment, 0.0, 1.0);

  {
    const size_t i = nearest_seg_idx;

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

/**
 * @brief limit variable whose differential is within a certain value
 * @param [in] input_val current value
 * @param [in] prev_val previous value
 * @param [in] dt time between current and previous one
 * @param [in] lim_val limitation value for differential
 */
double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double lim_val);

/**
 * @brief limit variable whose differential is within a certain value
 * @param [in] input_val current value
 * @param [in] prev_val previous value
 * @param [in] dt time between current and previous one
 * @param [in] max_val maximum value for differential
 * @param [in] min_val minimum value for differential
 */
double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double max_val,
  const double min_val);
}  // namespace velocity_controller_utils

#endif  // VELOCITY_CONTROLLER__VELOCITY_CONTROLLER_UTILS_HPP_
