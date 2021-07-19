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

#include <algorithm>
#include <limits>
#include <memory>

#include "velocity_controller/velocity_controller_mathutils.hpp"

namespace vcutils
{
double calcDistance2D(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2)
{
  const double dx = p1.position.x - p2.position.x;
  const double dy = p1.position.y - p2.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

double calcDistSquared2D(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2)
{
  const double dx = p1.position.x - p2.position.x;
  const double dy = p1.position.y - p2.position.y;
  return dx * dx + dy * dy;
}

double calcPitch(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2)
{
  const double dx = p1.position.x - p2.position.x;
  const double dy = p1.position.y - p2.position.y;
  const double dz = p1.position.z - p2.position.z;

  const double dist_2d = std::max(std::hypot(dx, dy), 1e-02);
  const double pitch = atan2(dz, dist_2d);
  return pitch;
}

double normalizeEulerAngle(double euler)
{
  double res = euler;
  while (res > M_PI) {
    res -= (2 * M_PI);
  }
  while (res < -M_PI) {
    res += 2 * M_PI;
  }

  return res;
}

bool calcClosestWithThr(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const geometry_msgs::msg::Pose & pose,
  const double angle_thr, const double dist_thr, int32_t & closest_idx)
{
  double dist_squared_min = std::numeric_limits<double>::max();
  closest_idx = -1;

  for (int32_t i = 0; i < (int32_t)trajectory.points.size(); ++i) {
    const double ds = calcDistSquared2D(trajectory.points.at(i).pose, pose);
    // printf("i = %d, ds = %f\n", i, ds);
    if (ds > dist_thr * dist_thr) {continue;}

    double yaw_pose = tf2::getYaw(pose.orientation);
    double yaw_ref = tf2::getYaw(trajectory.points.at(i).pose.orientation);
    double yaw_diff = normalizeEulerAngle(yaw_pose - yaw_ref);

    if (std::fabs(yaw_diff) > angle_thr) {continue;}

    if (ds < dist_squared_min) {
      dist_squared_min = ds;
      closest_idx = i;
    }
  }

  return (closest_idx >= 0) ? true : false;
}

geometry_msgs::msg::Point transformToRelativeCoordinate2D(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & origin)
{
  // translation
  geometry_msgs::msg::Point trans_p;
  trans_p.x = point.x - origin.position.x;
  trans_p.y = point.y - origin.position.y;

  // rotation (use inverse matrix of rotation)
  double yaw = tf2::getYaw(origin.orientation);

  geometry_msgs::msg::Point res;
  res.x = (std::cos(yaw) * trans_p.x) + (std::sin(yaw) * trans_p.y);
  res.y = (-std::sin(yaw) * trans_p.x) + (std::cos(yaw) * trans_p.y);
  res.z = origin.position.z;

  return res;
}

}  // namespace vcutils
