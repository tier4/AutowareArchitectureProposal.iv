// Copyright 2015-2019 Autoware Foundation
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

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "utilization/util.hpp"

namespace autoware
{
namespace planning
{
namespace behavior_velocity_planner_nodes
{
namespace planning_utils
{
double normalizeEulerAngle(double euler)
{
  double res = euler;
  while (res > M_PI) {
    res -= (2.0 * M_PI);
  }
  while (res < -M_PI) {
    res += 2.0 * M_PI;
  }

  return res;
}

geometry_msgs::msg::Quaternion getQuaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

template<class T>
bool calcClosestIndex(
  const T & path, const geometry_msgs::msg::Pose & pose, int & closest, double dist_thr,
  double angle_thr)
{
  double dist_squared_min = std::numeric_limits<double>::max();
  double yaw_pose = tf2::getYaw(pose.orientation);
  closest = -1;

  for (size_t i = 0; i < path.points.size(); ++i) {
    const double dist_squared = calcSquaredDist2d(getPose(path, i), pose);

    /* check distance threshold */
    if (dist_squared > dist_thr * dist_thr) {continue;}

    /* check angle threshold */
    double yaw_i = tf2::getYaw(getPose(path, i).orientation);
    double yaw_diff = normalizeEulerAngle(yaw_pose - yaw_i);

    if (std::fabs(yaw_diff) > angle_thr) {continue;}

    if (dist_squared < dist_squared_min) {
      dist_squared_min = dist_squared;
      closest = static_cast<int>(i);
    }
  }

  return closest != -1;
}

template bool calcClosestIndex<autoware_auto_msgs::msg::Trajectory>(
  const autoware_auto_msgs::msg::Trajectory & path, const geometry_msgs::msg::Pose & pose,
  int & closest, double dist_thr, double angle_thr);
template bool calcClosestIndex<autoware_auto_msgs::msg::PathWithLaneId>(
  const autoware_auto_msgs::msg::PathWithLaneId & path, const geometry_msgs::msg::Pose & pose,
  int & closest, double dist_thr, double angle_thr);
template bool calcClosestIndex<autoware_auto_msgs::msg::Path>(
  const autoware_auto_msgs::msg::Path & path, const geometry_msgs::msg::Pose & pose,
  int & closest, double dist_thr, double angle_thr);

template<class T>
bool calcClosestIndex(
  const T & path, const geometry_msgs::msg::Point & point, int & closest, double dist_thr)
{
  double dist_squared_min = std::numeric_limits<double>::max();
  closest = -1;

  for (size_t i = 0; i < path.points.size(); ++i) {
    const double dist_squared = calcSquaredDist2d(getPose(path, i), point);

    /* check distance threshold */
    if (dist_squared > dist_thr * dist_thr) {continue;}

    if (dist_squared < dist_squared_min) {
      dist_squared_min = dist_squared;
      closest = static_cast<int>(i);
    }
  }

  return closest != -1;
}
template bool calcClosestIndex<autoware_auto_msgs::msg::Trajectory>(
  const autoware_auto_msgs::msg::Trajectory & path, const geometry_msgs::msg::Point & point,
  int & closest, double dist_thr);
template bool calcClosestIndex<autoware_auto_msgs::msg::PathWithLaneId>(
  const autoware_auto_msgs::msg::PathWithLaneId & path, const geometry_msgs::msg::Point & point,
  int & closest, double dist_thr);
template bool calcClosestIndex<autoware_auto_msgs::msg::Path>(
  const autoware_auto_msgs::msg::Path & path, const geometry_msgs::msg::Point & point,
  int & closest, double dist_thr);

geometry_msgs::msg::Pose transformRelCoordinate2D(
  const geometry_msgs::msg::Pose & target, const geometry_msgs::msg::Pose & origin)
{
  // translation
  geometry_msgs::msg::Point trans_p;
  trans_p.x = target.position.x - origin.position.x;
  trans_p.y = target.position.y - origin.position.y;

  // rotation (use inverse matrix of rotation)
  double yaw = tf2::getYaw(origin.orientation);

  geometry_msgs::msg::Pose res;
  res.position.x = (std::cos(yaw) * trans_p.x) + (std::sin(yaw) * trans_p.y);
  res.position.y = ((-1.0) * std::sin(yaw) * trans_p.x) + (std::cos(yaw) * trans_p.y);
  res.position.z = target.position.z - origin.position.z;
  res.orientation = getQuaternionFromYaw(tf2::getYaw(target.orientation) - yaw);

  return res;
}

geometry_msgs::msg::Pose transformAbsCoordinate2D(
  const geometry_msgs::msg::Pose & relative, const geometry_msgs::msg::Pose & origin)
{
  // rotation
  geometry_msgs::msg::Point rot_p;
  double yaw = tf2::getYaw(origin.orientation);
  rot_p.x = (std::cos(yaw) * relative.position.x) + (-std::sin(yaw) * relative.position.y);
  rot_p.y = (std::sin(yaw) * relative.position.x) + (std::cos(yaw) * relative.position.y);

  // translation
  geometry_msgs::msg::Pose absolute;
  absolute.position.x = rot_p.x + origin.position.x;
  absolute.position.y = rot_p.y + origin.position.y;
  absolute.position.z = relative.position.z + origin.position.z;
  absolute.orientation = getQuaternionFromYaw(tf2::getYaw(relative.orientation) + yaw);

  return absolute;
}

double calcJudgeLineDistWithAccLimit(
  const double velocity, const double max_stop_acceleration,
  const double delay_response_time)
{
  double judge_line_dist =
    (velocity * velocity) / (2.0 * (-max_stop_acceleration)) + delay_response_time * velocity;
  return judge_line_dist;
}

double calcJudgeLineDistWithJerkLimit(
  const double velocity, const double acceleration,
  const double max_stop_jerk, const double max_stop_acceleration,
  const double delay_response_time)
{
  if (velocity <= 0.0) {
    return 0.0;
  }

  /* t0: subscribe traffic light state and decide to stop */
  /* t1: braking start (with jerk limitation) */
  /* t2: reach max stop acceleration */
  /* t3: stop */

  const double t1 = delay_response_time;
  const double x1 = velocity * t1;

  const double v2 = velocity +
    (std::pow(max_stop_acceleration, 2) - std::pow(acceleration, 2)) / (2.0 * max_stop_jerk);

  if (v2 <= 0.0) {
    const double t2 = -1.0 *
      (max_stop_acceleration +
      std::sqrt(acceleration * acceleration - 2.0 * max_stop_jerk * velocity)) / max_stop_jerk;
    const double x2 = velocity * t2 +
      acceleration * std::pow(t2, 2) / 2.0 +
      max_stop_jerk * std::pow(t2, 3) / 6.0;
    return std::max(0.0, x1 + x2);
  }

  const double t2 = (max_stop_acceleration - acceleration) / max_stop_jerk;
  const double x2 = velocity * t2 +
    acceleration * std::pow(t2, 2) / 2.0 +
    max_stop_jerk * std::pow(t2, 3) / 6.0;

  const double x3 = -1.0 * std::pow(v2, 2) / (2.0 * max_stop_acceleration);
  return std::max(0.0, x1 + x2 + x3);
}

std::vector<geometry_msgs::msg::Point> toRosPoints(
  const autoware_auto_msgs::msg::PredictedObjects & objects)
{
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto & obj : objects.objects) {
    points.emplace_back(obj.kinematics.initial_pose.pose.position);
  }
  return points;
}

geometry_msgs::msg::Point toRosPoint(const autoware::common::types::PointXYZI & pcl_point)
{
  geometry_msgs::msg::Point point;
  point.x = pcl_point.x;
  point.y = pcl_point.y;
  point.z = pcl_point.z;
  return point;
}

geometry_msgs::msg::Point toRosPoint(const Point2d & boost_point, const double z)
{
  geometry_msgs::msg::Point point;
  point.x = boost_point.x();
  point.y = boost_point.y();
  point.z = z;
  return point;
}

autoware_auto_msgs::msg::Complex32 quat_to_complex(
  const geometry_msgs::msg::Quaternion & quat)
{
  double yaw = tf2::getYaw(quat);
  autoware_auto_msgs::msg::Complex32 complex;
  complex.real = static_cast<float>(std::cos(yaw));
  complex.imag = static_cast<float>(std::sin(yaw));
  return complex;
}

geometry_msgs::msg::Quaternion complex_to_quat(
  const autoware_auto_msgs::msg::Complex32 & complex)
{
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, std::atan2(complex.imag, complex.real));
  geometry_msgs::msg::Quaternion quat_geo;
  quat_geo.set__x(quat.x());
  quat_geo.set__y(quat.y());
  quat_geo.set__z(quat.z());
  quat_geo.set__w(quat.w());
  return quat_geo;
}

geometry_msgs::msg::Pose traj_point_to_pose(
  const autoware_auto_msgs::msg::TrajectoryPoint & traj_point)
{
  geometry_msgs::msg::Pose pose;
  pose.position.set__x(traj_point.x);
  pose.position.set__y(traj_point.y);
  pose.position.set__z(traj_point.z);
  pose.orientation = complex_to_quat(traj_point.heading);
  return pose;
}

void set_traj_point_from_pose(
  const geometry_msgs::msg::Pose & pose,
  autoware_auto_msgs::msg::TrajectoryPoint & traj_point)
{
  traj_point.set__x(static_cast<float>(pose.position.x));
  traj_point.set__y(static_cast<float>(pose.position.y));
  traj_point.set__z(static_cast<float>(pose.position.z));
  traj_point.set__heading(quat_to_complex(pose.orientation));
}

}  // namespace planning_utils
}  // namespace behavior_velocity_planner_nodes
}  // namespace planning
}  // namespace autoware
