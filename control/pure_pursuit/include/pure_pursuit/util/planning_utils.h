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

#pragma once

#include <memory>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"
#include "Eigen/Geometry"

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>

#include "interpolate.hpp"

#define PLANNING_UTILS_LOGGER "planning_utils"

namespace planning_utils
{
constexpr double ERROR = 1e-6;

double calcCurvature(
  const geometry_msgs::msg::Point & target,
  const geometry_msgs::msg::Pose & curr_pose);
double calcDistance2D(const geometry_msgs::msg::Point & p, const geometry_msgs::msg::Point & q);
double calcDistSquared2D(const geometry_msgs::msg::Point & p, const geometry_msgs::msg::Point & q);
double calcStopDistanceWithConstantJerk(const double & v_init, const double & j);
double calcLateralError2D(
  const geometry_msgs::msg::Point & a_start, const geometry_msgs::msg::Point & a_end,
  const geometry_msgs::msg::Point & b);
double calcRadius(
  const geometry_msgs::msg::Point & target,
  const geometry_msgs::msg::Pose & current_pose);
double convertCurvatureToSteeringAngle(double wheel_base, double kappa);

std::vector<geometry_msgs::msg::Pose> extractPoses(
  const autoware_planning_msgs::msg::Trajectory & motions);

std::pair<bool, int32_t> findClosestIdxWithDistAngThr(
  const std::vector<geometry_msgs::msg::Pose> & poses,
  const geometry_msgs::msg::Pose & current_pose,
  const double th_dist = 3.0, const double th_yaw = M_PI_2);

int8_t getLaneDirection(const std::vector<geometry_msgs::msg::Pose> & poses, double th_dist = 0.5);
bool isDirectionForward(
  const geometry_msgs::msg::Pose & prev,
  const geometry_msgs::msg::Pose & next);
bool isDirectionForward(
  const geometry_msgs::msg::Pose & prev,
  const geometry_msgs::msg::Point & next);

// refer from apache's pointinpoly in http://www.visibone.com/inpoly/
template<typename T>
bool isInPolygon(const std::vector<T> & polygon, const T & point)
{
  // polygons with fewer than 3 sides are excluded
  if (polygon.size() < 3) {return false;}

  bool in_poly = false;
  double x1, x2, y1, y2;

  uint32_t nr_poly_points = polygon.size();
  // start with the last point to make the check last point<->first point the first one
  double xold = polygon.at(nr_poly_points - 1).x();
  double yold = polygon.at(nr_poly_points - 1).y();
  for (const auto & poly_p : polygon) {
    double xnew = poly_p.x();
    double ynew = poly_p.y();
    if (xnew > xold) {
      x1 = xold;
      x2 = xnew;
      y1 = yold;
      y2 = ynew;
    } else {
      x1 = xnew;
      x2 = xold;
      y1 = ynew;
      y2 = yold;
    }

    if (
      (xnew < point.x()) == (point.x() <= xold) &&
      (point.y() - y1) * (x2 - x1) < (y2 - y1) * (point.x() - x1))
    {
      in_poly = !in_poly;
    }
    xold = xnew;
    yold = ynew;
  }

  return in_poly;
}

template<>
bool isInPolygon(
  const std::vector<geometry_msgs::msg::Point> & polygon, const geometry_msgs::msg::Point & point);

double kmph2mps(const double velocity_kmph);
double normalizeEulerAngle(const double euler);

geometry_msgs::msg::Point transformToAbsoluteCoordinate2D(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & current_pose);

geometry_msgs::msg::Point transformToRelativeCoordinate2D(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & current_pose);

geometry_msgs::msg::Quaternion getQuaternionFromYaw(const double _yaw);

}  // namespace planning_utils
