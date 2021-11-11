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

#ifndef UTILIZATION__UTIL_HPP_
#define UTILIZATION__UTIL_HPP_

#include <common/types.hpp>

#include <string>
#include <vector>

#include "autoware_auto_msgs/msg/path.hpp"
#include "autoware_auto_msgs/msg/path_with_lane_id.hpp"
#include "autoware_auto_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_msgs/msg/trajectory.hpp"
#include "autoware_auto_msgs/msg/trajectory_point.hpp"
#include "boost/geometry.hpp"
#include "boost/geometry/geometries/linestring.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/utils.h"
#include "visualization_msgs/msg/marker.hpp"

namespace autoware
{
namespace planning
{
namespace behavior_velocity_planner_nodes
{
using Point2d = boost::geometry::model::d2::point_xy<double>;
namespace planning_utils
{
autoware_auto_msgs::msg::Complex32 quat_to_complex(const geometry_msgs::msg::Quaternion & quat);

geometry_msgs::msg::Quaternion complex_to_quat(const autoware_auto_msgs::msg::Complex32 & complex);

geometry_msgs::msg::Pose traj_point_to_pose(
  const autoware_auto_msgs::msg::TrajectoryPoint & traj_point);

void set_traj_point_from_pose(
  const geometry_msgs::msg::Pose & pose, autoware_auto_msgs::msg::TrajectoryPoint & traj_point);

inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::Point & p) {return p;}

inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::Pose & p) {return p.position;}

inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::PoseStamped & p)
{
  return p.pose.position;
}

inline geometry_msgs::msg::Point getPoint(const autoware_auto_msgs::msg::PathPoint & p)
{
  return p.pose.position;
}

inline geometry_msgs::msg::Point getPoint(const autoware_auto_msgs::msg::PathPointWithLaneId & p)
{
  return p.point.pose.position;
}

inline geometry_msgs::msg::Point getPoint(const autoware_auto_msgs::msg::TrajectoryPoint & p)
{
  geometry_msgs::msg::Point point;
  point.set__x(p.x);
  point.set__y(p.y);
  point.set__z(p.z);
  return point;
}
inline geometry_msgs::msg::Pose getPose(const autoware_auto_msgs::msg::Path & path, size_t idx)
{
  return path.points.at(idx).pose;
}

inline geometry_msgs::msg::Pose getPose(
  const autoware_auto_msgs::msg::PathWithLaneId & path, size_t idx)
{
  return path.points.at(idx).point.pose;
}

inline geometry_msgs::msg::Pose getPose(
  const autoware_auto_msgs::msg::Trajectory & traj, size_t idx)
{
  return traj_point_to_pose(traj.points.at(idx));
}

inline int64_t bitShift(int64_t original_id) {return original_id << (sizeof(int32_t) * 8 / 2);}

inline double square(const double & a) {return a * a;}
double normalizeEulerAngle(double euler);
geometry_msgs::msg::Quaternion getQuaternionFromYaw(double yaw);

template<class T1, class T2>
double calcSquaredDist2d(const T1 & a, const T2 & b)
{
  return square(getPoint(a).x - getPoint(b).x) + square(getPoint(a).y - getPoint(b).y);
}

template<class T1, class T2>
double calcDist2d(const T1 & a, const T2 & b)
{
  return std::sqrt(calcSquaredDist2d<T1, T2>(a, b));
}

template<class T>
bool calcClosestIndex(
  const T & path,
  const geometry_msgs::msg::Pose & pose,
  int & closest,
  double dist_thr = 3.0,
  double angle_thr = M_PI_4);

template<class T>
bool calcClosestIndex(
  const T & path, const geometry_msgs::msg::Point & point, int & closest, double dist_thr = 3.0);

geometry_msgs::msg::Pose transformRelCoordinate2D(
  const geometry_msgs::msg::Pose & target, const geometry_msgs::msg::Pose & origin);
geometry_msgs::msg::Pose transformAbsCoordinate2D(
  const geometry_msgs::msg::Pose & relative, const geometry_msgs::msg::Pose & origin);

double calcJudgeLineDistWithAccLimit(
  const double velocity, const double max_stop_acceleration, const double delay_response_time);

double calcJudgeLineDistWithJerkLimit(
  const double velocity,
  const double acceleration,
  const double max_stop_acceleration,
  const double max_stop_jerk,
  const double delay_response_time);

std::vector<geometry_msgs::msg::Point> toRosPoints(
  const autoware_auto_msgs::msg::PredictedObjects & objects);

geometry_msgs::msg::Point toRosPoint(const autoware::common::types::PointXYZI & pcl_point);
geometry_msgs::msg::Point toRosPoint(const Point2d & boost_point, const double z);

template<class T>
std::vector<T> concatVector(const std::vector<T> & vec1, const std::vector<T> & vec2)
{
  auto concat_vec = vec1;
  concat_vec.insert(std::end(concat_vec), std::begin(vec2), std::end(vec2));
  return concat_vec;
}
}  // namespace planning_utils
}  // namespace behavior_velocity_planner_nodes
}  // namespace planning
}  // namespace autoware

#endif  // UTILIZATION__UTIL_HPP_
