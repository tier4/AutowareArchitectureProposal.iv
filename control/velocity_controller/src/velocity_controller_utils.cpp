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

#include "velocity_controller/velocity_controller_utils.hpp"

#include <algorithm>
#include <limits>

namespace velocity_controller_utils
{
bool isValidTrajectory(const Trajectory & traj)
{
  for (const auto & points : traj.points) {
    const auto & p = points.pose.position;
    const auto & o = points.pose.orientation;
    const auto & t = points.twist.linear;
    const auto & a = points.accel.linear;
    if (
      !isfinite(p.x) || !isfinite(p.y) || !isfinite(p.z) || !isfinite(o.x) || !isfinite(o.y) ||
      !isfinite(o.z) || !isfinite(o.w) || !isfinite(t.x) || !isfinite(t.y) || !isfinite(t.z) ||
      !isfinite(a.x) || !isfinite(a.y) || !isfinite(a.z)) {
      return false;
    }
  }

  // when trajectory is empty
  if (traj.points.empty()) {
    return false;
  }

  return true;
}

double calcStopDistance(const Point & current_pos, const Trajectory & traj)
{
  const boost::optional<size_t> stop_idx_opt = autoware_utils::searchZeroVelocityIndex(traj.points);

  // If no zero velocity point, return the length between current_pose to the end of trajectory.
  if (!stop_idx_opt) {
    return autoware_utils::calcSignedArcLength(traj.points, current_pos, traj.points.size() - 1);
  }

  return autoware_utils::calcSignedArcLength(traj.points, current_pos, *stop_idx_opt);
}

double getPitchByPose(const Quaternion & quaternion)
{
  const Eigen::Quaterniond q(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
  const Eigen::Vector3d v = q.toRotationMatrix() * Eigen::Vector3d::UnitX();
  const double xy = std::max(std::hypot(v.x(), v.y()), 1e-8 /* avoid 0 divide */);
  const double pitch = -1.0 * std::atan2(v.z(), xy);
  return pitch;
}

double getPitchByTraj(
  const Trajectory & trajectory, const size_t nearest_idx, const double wheel_base)
{
  // cannot calculate pitch
  if (trajectory.points.size() <= 1) {
    return 0.0;
  }

  for (size_t i = nearest_idx + 1; i < trajectory.points.size(); ++i) {
    const double dist = autoware_utils::calcDistance2d(
      trajectory.points.at(nearest_idx).pose, trajectory.points.at(i).pose);
    if (dist > wheel_base) {
      // calculate pitch from trajectory between rear wheel (nearest) and front center (i)
      return velocity_controller_utils::calcElevationAngle(
        trajectory.points.at(nearest_idx).pose.position, trajectory.points.at(i).pose.position);
    }
  }

  // close to goal
  for (int i = trajectory.points.size() - 1; i > 0; --i) {
    const double dist =
      autoware_utils::calcDistance2d(trajectory.points.back().pose, trajectory.points.at(i).pose);

    if (dist > wheel_base) {
      // calculate pitch from trajectory
      // between wheelbase behind the end of trajectory (i) and the end of trajectory (back)
      return velocity_controller_utils::calcElevationAngle(
        trajectory.points.at(i).pose.position, trajectory.points.back().pose.position);
    }
  }

  // calculate pitch from trajectory between the beginning and end of trajectory
  return calcElevationAngle(
    trajectory.points.at(0).pose.position, trajectory.points.back().pose.position);
}

double calcElevationAngle(const Point & p_from, const Point & p_to)
{
  const double dx = p_from.x - p_to.x;
  const double dy = p_from.y - p_to.y;
  const double dz = p_from.z - p_to.z;

  const double dxy = std::max(std::hypot(dx, dy), std::numeric_limits<double>::epsilon());
  const double pitch = std::atan2(dz, dxy);

  return pitch;
}

Pose calcPoseAfterTimeDelay(
  const Pose & current_pose, const double delay_time, const double current_vel)
{
  // simple linear prediction
  const double yaw = tf2::getYaw(current_pose.orientation);
  const double running_distance = delay_time * current_vel;
  const double dx = running_distance * std::cos(yaw);
  const double dy = running_distance * std::sin(yaw);

  auto pred_pose = current_pose;
  pred_pose.position.x += dx;
  pred_pose.position.y += dy;
  return pred_pose;
}

double lerp(const double v_from, const double v_to, const double ratio)
{
  return v_from + (v_to - v_from) * ratio;
}

Quaternion lerpOrientation(const Quaternion & o_from, const Quaternion & o_to, const double ratio)
{
  tf2::Quaternion q_from, q_to;
  tf2::fromMsg(o_from, q_from);
  tf2::fromMsg(o_to, q_to);

  const auto q_interpolated = q_from.slerp(q_to, ratio);
  return tf2::toMsg(q_interpolated);
}

double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double max_val,
  const double min_val)
{
  const double diff_raw = (input_val - prev_val) / dt;
  const double diff = std::min(std::max(diff_raw, min_val), max_val);
  const double filtered_val = prev_val + diff * dt;
  return filtered_val;
}

double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double lim_val)
{
  const double max_val = std::fabs(lim_val);
  const double min_val = -max_val;
  return applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val);
}

}  // namespace velocity_controller_utils
