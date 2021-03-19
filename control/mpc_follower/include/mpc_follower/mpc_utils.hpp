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

#ifndef MPC_FOLLOWER__MPC_UTILS_HPP_
#define MPC_FOLLOWER__MPC_UTILS_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "mpc_follower/interpolate.hpp"
#include "mpc_follower/mpc_trajectory.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"

#include "eigen3/Eigen/Core"


namespace MPCUtils
{
/**
 * @brief convert from yaw to ros-Quaternion
 * @param [in] yaw input yaw angle
 * @return quaternion
 */
geometry_msgs::msg::Quaternion getQuaternionFromYaw(const double & yaw);

/**
 * @brief normalize angle into [-pi to pi]
 * @param [in] _angle input angle
 * @return normalized angle
 */
double normalizeRadian(const double angle);

/**
 * @brief convert Euler angle vector including +-2pi to 0 jump to continuous series data
 * @param [out] a input angle vector
 */
void convertEulerAngleToMonotonic(std::vector<double> * a);
double calcDist2d(
  const geometry_msgs::msg::PoseStamped & p0, const geometry_msgs::msg::PoseStamped & p1);
double calcDist2d(const geometry_msgs::msg::Pose & p0, const geometry_msgs::msg::Pose & p1);
double calcDist2d(const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1);
double calcDist3d(const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1);
double calcSquaredDist2d(
  const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1);
double calcLateralError(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & ref_pose);

bool convertToMPCTrajectory(
  const autoware_planning_msgs::msg::Trajectory & input, MPCTrajectory * output);
bool convertToAutowareTrajectory(
  const MPCTrajectory & input, autoware_planning_msgs::msg::Trajectory * output);
void calcMPCTrajectoryArclength(const MPCTrajectory & trajectory, std::vector<double> * arclength);
bool resampleMPCTrajectoryByDistance(
  const MPCTrajectory & input, const double resample_interval_dist, MPCTrajectory * output);
bool linearInterpMPCTrajectory(
  const std::vector<double> & in_index, const MPCTrajectory & in_traj,
  const std::vector<double> & out_index, MPCTrajectory * out_traj);
bool splineInterpMPCTrajectory(
  const std::vector<double> & in_index, const MPCTrajectory & in_traj,
  const std::vector<double> & out_index, MPCTrajectory * out_traj);
bool calcMPCTrajectoryTime(MPCTrajectory * traj);
void dynamicSmoothingVelocity(
  const int start_idx, const double start_vel, const double acc_lim, const double tau,
  MPCTrajectory * traj);

/**
 * @brief calculate yaw angle in MPCTrajectory from xy vector
 * @param [inout] traj object trajectory
 */
void calcTrajectoryYawFromXY(MPCTrajectory * traj);

/**
 * @brief Calculate path curvature by 3-points circle fitting with smoothing num (use nearest 3 points when num = 1)
 * @param [in] curvature_smoothing_num index distance for 3 points for curvature calculation
 * @param [inout] traj object trajectory
 */
bool calcTrajectoryCurvature(int curvature_smoothing_num, MPCTrajectory * traj);

/**
 * @brief Calculate path curvature by 3-points circle fitting with smoothing num (use nearest 3 points when num = 1)
 * @param [in] curvature_smoothing_num index distance for 3 points for curvature calculation
 * @param [inout] curvature vector
 */
std::vector<double> calcTrajectoryCurvature(
  const int curvature_smoothing_num, const MPCTrajectory & traj);

/**
 * @brief calculate nearest pose on MPCTrajectory with linear interpolation
 * @param [in] traj reference trajectory
 * @param [in] self_pose object pose
 * @param [out] nearest_pose nearest pose on path
 * @param [out] nearest_index path index of nearest pose
 * @param [out] nearest_time time of nearest pose on trajectory
 * @param [out] logger to output the reason for failure
 * @param [in] clock to throttle log output
 * @return false when nearest pose couldn't find for some reasons
 */
bool calcNearestPoseInterp(
  const MPCTrajectory & traj, const geometry_msgs::msg::Pose & self_pose,
  geometry_msgs::msg::Pose * nearest_pose, int * nearest_index, double * nearest_time,
  rclcpp::Logger logger, rclcpp::Clock & clock);

int calcNearestIndex(const MPCTrajectory & traj, const geometry_msgs::msg::Pose & self_pose);
int calcNearestIndex(
  const autoware_planning_msgs::msg::Trajectory & traj, const geometry_msgs::msg::Pose & self_pose);
/**
 * @brief convert MPCTraj to visualization marker for visualization
 */
visualization_msgs::msg::MarkerArray convertTrajToMarker(
  const MPCTrajectory & traj, std::string ns, double r, double g, double b, double z,
  const std::string & frame_id, const builtin_interfaces::msg::Time & stamp);

}  // namespace MPCUtils
#endif  // MPC_FOLLOWER__MPC_UTILS_HPP_
