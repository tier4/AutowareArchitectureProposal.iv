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

#include <iostream>

#include <obstacle_collision_checker/obstacle_collision_checker.h>
#include <boost/geometry.hpp>

#include <autoware_utils/geometry/geometry.h>
#include <autoware_utils/math/normalization.h>
#include <autoware_utils/math/unit_conversion.h>
#include <autoware_utils/system/stop_watch.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/console.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>

#include <obstacle_collision_checker/util/create_vehicle_footprint.h>

namespace
{
pcl::PointCloud<pcl::PointXYZ> getTransformedPointCloud(
  const sensor_msgs::PointCloud2 & pointcloud_msg, const geometry_msgs::Transform & transform)
{
  const Eigen::Matrix4f transform_matrix = tf2::transformToEigen(transform).matrix().cast<float>();

  sensor_msgs::PointCloud2 transformed_msg;
  pcl_ros::transformPointCloud(transform_matrix, pointcloud_msg, transformed_msg);

  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(transformed_msg, transformed_pointcloud);

  return transformed_pointcloud;
}

pcl::PointCloud<pcl::PointXYZ> filterPointCloudByTrajectory(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud,
  const autoware_planning_msgs::Trajectory & trajectory, const double radius)
{
  pcl::PointCloud<pcl::PointXYZ> filtered_pointcloud;
  for (const auto & trajectory_point : trajectory.points) {
    for (const auto & point : pointcloud.points) {
      const double dx = trajectory_point.pose.position.x - point.x;
      const double dy = trajectory_point.pose.position.y - point.y;
      if (std::hypot(dx, dy) < radius) {
        filtered_pointcloud.points.push_back(point);
      }
    }
  }
  return filtered_pointcloud;
}

double calcBrakingDistance(
  const double abs_velocity, const double max_deceleration, const double delay_time)
{
  const double idling_distance = abs_velocity * delay_time;
  const double braking_distance = (abs_velocity * abs_velocity) / (2.0 * max_deceleration);
  return idling_distance + braking_distance;
}

}  // namespace

namespace obstacle_collision_checker
{
Output ObstacleCollisionChecker::update(const Input & input)
{
  Output output{};
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  // resample trajectory by braking distance
  constexpr double min_velocity = 0.01;
  const auto & raw_abs_velocity = std::abs(input.current_twist->twist.linear.x);
  const auto abs_velocity = raw_abs_velocity < min_velocity ? 0.0 : raw_abs_velocity;
  const auto braking_distance =
    calcBrakingDistance(abs_velocity, param_.max_deceleration, param_.delay_time);
  output.resampled_trajectory = cutTrajectory(
    resampleTrajectory(*input.predicted_trajectory, param_.resample_interval), braking_distance);
  output.processing_time_map["resampleTrajectory"] = stop_watch.toc(true);

  // resample pointcloud
  const auto obstacle_pointcloud =
    getTransformedPointCloud(*input.obstacle_pointcloud, input.obstacle_transform->transform);
  const auto filtered_obstacle_pointcloud = filterPointCloudByTrajectory(
    obstacle_pointcloud, output.resampled_trajectory, param_.search_radius);

  output.vehicle_footprints = createVehicleFootprints(output.resampled_trajectory, param_);
  output.processing_time_map["createVehicleFootprints"] = stop_watch.toc(true);

  output.vehicle_passing_areas = createVehiclePassingAreas(output.vehicle_footprints);
  output.processing_time_map["createVehiclePassingAreas"] = stop_watch.toc(true);

  output.will_collide = willCollide(filtered_obstacle_pointcloud, output.vehicle_passing_areas);
  output.processing_time_map["willCollide"] = stop_watch.toc(true);

  return output;
}

autoware_planning_msgs::Trajectory ObstacleCollisionChecker::resampleTrajectory(
  const autoware_planning_msgs::Trajectory & trajectory, const double interval)
{
  autoware_planning_msgs::Trajectory resampled;
  resampled.header = trajectory.header;

  resampled.points.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
    const auto & point = trajectory.points.at(i);

    const auto p1 = autoware_utils::fromMsg(resampled.points.back().pose.position).to_2d();
    const auto p2 = autoware_utils::fromMsg(point.pose.position).to_2d();

    if (boost::geometry::distance(p1, p2) > interval) {
      resampled.points.push_back(point);
    }
  }
  resampled.points.push_back(trajectory.points.back());

  return resampled;
}

autoware_planning_msgs::Trajectory ObstacleCollisionChecker::cutTrajectory(
  const autoware_planning_msgs::Trajectory & trajectory, const double length)
{
  autoware_planning_msgs::Trajectory cut;
  cut.header = trajectory.header;

  double total_length = 0.0;
  cut.points.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points.at(i);

    const auto p1 = autoware_utils::fromMsg(cut.points.back().pose.position);
    const auto p2 = autoware_utils::fromMsg(point.pose.position);
    const auto points_distance = boost::geometry::distance(p1.to_2d(), p2.to_2d());

    const auto remain_distance = length - total_length;

    // Over length
    if (remain_distance <= 0) {
      break;
    }

    // Require interpolation
    if (remain_distance <= points_distance) {
      const Eigen::Vector3d p_interpolated = p1 + remain_distance * (p2 - p1).normalized();

      autoware_planning_msgs::TrajectoryPoint p;
      p.pose.position.x = p_interpolated.x();
      p.pose.position.y = p_interpolated.y();
      p.pose.position.z = p_interpolated.z();
      p.pose.orientation = point.pose.orientation;

      cut.points.push_back(p);
      break;
    }

    cut.points.push_back(point);
    total_length += points_distance;
  }

  return cut;
}

std::vector<LinearRing2d> ObstacleCollisionChecker::createVehicleFootprints(
  const autoware_planning_msgs::Trajectory & trajectory, const Param & param)
{
  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint =
    createVehicleFootprint(param.vehicle_info, param.footprint_margin);

  // Create vehicle footprint on each TrajectoryPoint
  std::vector<LinearRing2d> vehicle_footprints;
  for (const auto & p : trajectory.points) {
    vehicle_footprints.push_back(
      transformVector(local_vehicle_footprint, autoware_utils::pose2transform(p.pose)));
  }

  return vehicle_footprints;
}

std::vector<LinearRing2d> ObstacleCollisionChecker::createVehiclePassingAreas(
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  // Create hull from two adjacent vehicle footprints
  std::vector<LinearRing2d> areas;
  for (size_t i = 0; i < vehicle_footprints.size() - 1; ++i) {
    const auto & footprint1 = vehicle_footprints.at(i);
    const auto & footprint2 = vehicle_footprints.at(i + 1);
    areas.push_back(createHullFromFootprints(footprint1, footprint2));
  }

  return areas;
}

LinearRing2d ObstacleCollisionChecker::createHullFromFootprints(
  const LinearRing2d & area1, const LinearRing2d & area2)
{
  autoware_utils::MultiPoint2d combined;
  for (const auto & p : area1) {
    combined.push_back(p);
  }
  for (const auto & p : area2) {
    combined.push_back(p);
  }
  LinearRing2d hull;
  boost::geometry::convex_hull(combined, hull);
  return hull;
}

bool ObstacleCollisionChecker::willCollide(
  const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud,
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  for (const auto & vehicle_footprint : vehicle_footprints) {
    if (hasCollision(obstacle_pointcloud, vehicle_footprint)) {
      ROS_WARN("ObstacleCollisionChecker::willCollide");
      return true;
    }
  }

  return false;
}

bool ObstacleCollisionChecker::hasCollision(
  const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud,
  const LinearRing2d & vehicle_footprint)
{
  for (const auto & point : obstacle_pointcloud) {
    if (boost::geometry::within(autoware_utils::Point2d{point.x, point.y}, vehicle_footprint)) {
      ROS_WARN("[ObstacleCollisionChecker] Collide to Point x: %f y: %f", point.x, point.y);
      return true;
    }
  }

  return false;
}
}  // namespace obstacle_collision_checker
