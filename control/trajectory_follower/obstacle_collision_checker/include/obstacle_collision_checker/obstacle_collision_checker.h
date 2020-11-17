/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include <boost/optional.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <autoware_utils/geometry/boost_geometry.h>
#include <autoware_utils/ros/vehicle_info.h>

namespace obstacle_collision_checker
{
using autoware_utils::LinearRing2d;

struct Param
{
  VehicleInfo vehicle_info;
  double delay_time;
  double footprint_margin;
  double max_deceleration;
  double resample_interval;
  double search_radius;
};

struct Input
{
  geometry_msgs::PoseStamped::ConstPtr current_pose;
  geometry_msgs::TwistStamped::ConstPtr current_twist;
  sensor_msgs::PointCloud2::ConstPtr obstacle_pointcloud;
  geometry_msgs::TransformStamped::ConstPtr obstacle_transform;
  autoware_planning_msgs::Trajectory::ConstPtr reference_trajectory;
  autoware_planning_msgs::Trajectory::ConstPtr predicted_trajectory;
};

struct Output
{
  std::map<std::string, double> processing_time_map;
  bool will_collide;
  autoware_planning_msgs::Trajectory resampled_trajectory;
  std::vector<LinearRing2d> vehicle_footprints;
  std::vector<LinearRing2d> vehicle_passing_areas;
};

class ObstacleCollisionChecker
{
public:
  Output update(const Input & input);

  void setParam(const Param & param) { param_ = param; }

private:
  Param param_;

  //! This function assumes the input trajectory is sampled dense enough
  static autoware_planning_msgs::Trajectory resampleTrajectory(
    const autoware_planning_msgs::Trajectory & trajectory, const double interval);

  static autoware_planning_msgs::Trajectory cutTrajectory(
    const autoware_planning_msgs::Trajectory & trajectory, const double length);

  static std::vector<LinearRing2d> createVehicleFootprints(
    const autoware_planning_msgs::Trajectory & trajectory, const Param & param);

  static std::vector<LinearRing2d> createVehiclePassingAreas(
    const std::vector<LinearRing2d> & vehicle_footprints);

  static LinearRing2d createHullFromFootprints(
    const LinearRing2d & area1, const LinearRing2d & area2);

  static bool willCollide(
    const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud,
    const std::vector<LinearRing2d> & vehicle_footprints);

  static bool hasCollision(
    const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud,
    const LinearRing2d & vehicle_footprint);
};
}  // namespace obstacle_collision_checker
