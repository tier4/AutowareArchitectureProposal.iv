// Copyright 2020 Tier IV, Inc.
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

#ifndef MOTION_VELOCITY_OPTIMIZER__MOTION_VELOCITY_OPTIMIZER_UTILS_HPP_
#define MOTION_VELOCITY_OPTIMIZER__MOTION_VELOCITY_OPTIMIZER_UTILS_HPP_

#include <iostream>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"

#include "autoware_planning_msgs/msg/trajectory.hpp"

namespace vpu
{
double square(const double & a);
double calcSquaredDist2d(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);
double calcSquaredDist2d(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b);
double calcSquaredDist2d(
  const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b);
double calcSquaredDist2d(
  const autoware_planning_msgs::msg::TrajectoryPoint & a,
  const autoware_planning_msgs::msg::TrajectoryPoint & b);
double calcDist2d(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);
double calcDist2d(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b);
double calcDist2d(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b);
double calcDist2d(
  const autoware_planning_msgs::msg::TrajectoryPoint & a,
  const autoware_planning_msgs::msg::TrajectoryPoint & b);
double calcDist2dToLine(
  const geometry_msgs::msg::Pose & pose1, const geometry_msgs::msg::Pose & pose2,
  const geometry_msgs::msg::Pose & pose_target);
double calcTriangleVerticalInterpolatedLength(
  const geometry_msgs::msg::Point & top, const geometry_msgs::msg::Point & bottom_target,
  const geometry_msgs::msg::Point & bottom_other);
bool calcWhichSideOfLine(
  const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to,
  const geometry_msgs::msg::Point & p_target);
int calcClosestWaypoint(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Point & point);
autoware_planning_msgs::msg::TrajectoryPoint calcClosestTrajectoryPointWithInterpolation(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & target_pose);
tf2::Vector3 getTransVector3(
  const geometry_msgs::msg::Pose & from,
  const geometry_msgs::msg::Pose & to);
int calcClosestWaypoint(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const geometry_msgs::msg::Pose & pose,
  const double delta_yaw_threshold);
bool extractPathAroundIndex(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const int index,
  const double & ahead_length, const double & behind_length,
  autoware_planning_msgs::msg::Trajectory & extracted_base_waypoints);
double calcLengthOnWaypoints(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const int idx1, const int idx2);
void calcTrajectoryArclength(
  const autoware_planning_msgs::msg::Trajectory & trajectory, std::vector<double> & arclength);
void calcTrajectoryIntervalDistance(
  const autoware_planning_msgs::msg::Trajectory & trajectory, std::vector<double> & intervals);
void setZeroVelocity(autoware_planning_msgs::msg::Trajectory & trajectory);
double getMaxVelocity(const autoware_planning_msgs::msg::Trajectory & trajectory);
double getMaxAbsVelocity(const autoware_planning_msgs::msg::Trajectory & trajectory);
void minimumVelocityFilter(
  const double & min_vel,
  autoware_planning_msgs::msg::Trajectory & trajectory);
void maximumVelocityFilter(
  const double & max_vel,
  autoware_planning_msgs::msg::Trajectory & trajectory);
void multiplyConstantToTrajectoryVelocity(
  const double & scalar, autoware_planning_msgs::msg::Trajectory & trajectory);
void insertZeroVelocityAfterIdx(
  const int & stop_idx, autoware_planning_msgs::msg::Trajectory & trajectory);
double getVx(const autoware_planning_msgs::msg::Trajectory & trajectory, const int & i);
bool searchZeroVelocityIdx(const autoware_planning_msgs::msg::Trajectory & trajectory, int & idx);
bool calcTrajectoryCurvatureFrom3Points(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const unsigned int & idx_dist,
  std::vector<double> & k_arr);
double normalizeRadian(const double _angle);
void convertEulerAngleToMonotonic(std::vector<double> & a);
geometry_msgs::msg::Quaternion getQuaternionFromYaw(double yaw);
bool linearInterpTrajectory(
  const std::vector<double> & base_index,
  const autoware_planning_msgs::msg::Trajectory & base_trajectory,
  const std::vector<double> & out_index,
  autoware_planning_msgs::msg::Trajectory & out_trajectory);
}  // namespace vpu

#endif  // MOTION_VELOCITY_OPTIMIZER__MOTION_VELOCITY_OPTIMIZER_UTILS_HPP_
