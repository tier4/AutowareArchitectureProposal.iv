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

#include <obstacle_collision_checker/obstacle_collision_checker_node.h>

#include <autoware_utils/geometry/geometry.h>
#include <autoware_utils/math/unit_conversion.h>
#include <autoware_utils/ros/marker_helper.h>

#include <obstacle_collision_checker/util/create_vehicle_footprint.h>

namespace obstacle_collision_checker
{
ObstacleCollisionCheckerNode::ObstacleCollisionCheckerNode()
{
  // Node Parameter
  private_nh_.param("update_rate", node_param_.update_rate, 10.0);

  // Core Parameter
  param_.vehicle_info = waitForVehicleInfo();
  private_nh_.param("delay_time", param_.delay_time, 0.3);
  private_nh_.param("footprint_margin", param_.footprint_margin, 0.0);
  private_nh_.param("max_deceleration", param_.max_deceleration, 2.0);
  private_nh_.param("resample_interval", param_.resample_interval, 0.5);
  private_nh_.param("search_radius", param_.search_radius, 5.0);

  // Dynamic Reconfigure
  dynamic_reconfigure_.setCallback(
    boost::bind(&ObstacleCollisionCheckerNode::onConfig, this, _1, _2));

  // Core
  obstacle_collision_checker_ = std::make_unique<ObstacleCollisionChecker>();
  obstacle_collision_checker_->setParam(param_);

  // Subscriber
  sub_obstacle_pointcloud_ = private_nh_.subscribe(
    "input/obstacle_pointcloud", 1, &ObstacleCollisionCheckerNode::onObstaclePointcloud, this);
  sub_reference_trajectory_ = private_nh_.subscribe(
    "input/reference_trajectory", 1, &ObstacleCollisionCheckerNode::onReferenceTrajectory, this);
  sub_predicted_trajectory_ = private_nh_.subscribe(
    "input/predicted_trajectory", 1, &ObstacleCollisionCheckerNode::onPredictedTrajectory, this);
  sub_twist_ =
    private_nh_.subscribe("input/twist", 1, &ObstacleCollisionCheckerNode::onTwist, this);

  // Publisher
  // Nothing

  // Diagnostic Updater
  updater_.setHardwareID("obstacle_collision_checker");

  updater_.add(
    "obstacle_collision_checker",
    boost::bind(&ObstacleCollisionCheckerNode::checkLaneDeparture, this, _1));

  // Wait for first self pose
  self_pose_listener_.waitForFirstPose();

  // Timer
  timer_ = private_nh_.createTimer(
    ros::Rate(node_param_.update_rate), &ObstacleCollisionCheckerNode::onTimer, this);
}

void ObstacleCollisionCheckerNode::onObstaclePointcloud(
  const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  obstacle_pointcloud_ = msg;
}

void ObstacleCollisionCheckerNode::onReferenceTrajectory(
  const autoware_planning_msgs::Trajectory::ConstPtr & msg)
{
  reference_trajectory_ = msg;
}

void ObstacleCollisionCheckerNode::onPredictedTrajectory(
  const autoware_planning_msgs::Trajectory::ConstPtr & msg)
{
  predicted_trajectory_ = msg;
}

void ObstacleCollisionCheckerNode::onTwist(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
  current_twist_ = msg;
}

bool ObstacleCollisionCheckerNode::isDataReady()
{
  if (!current_pose_) {
    ROS_INFO_THROTTLE(5.0, "waiting for current_pose...");
    return false;
  }

  if (!obstacle_pointcloud_) {
    ROS_INFO_THROTTLE(5.0, "waiting for obstacle_pointcloud msg...");
    return false;
  }

  if (!obstacle_transform_) {
    ROS_INFO_THROTTLE(5.0, "waiting for obstacle_transform...");
    return false;
  }

  if (!reference_trajectory_) {
    ROS_INFO_THROTTLE(5.0, "waiting for reference_trajectory msg...");
    return false;
  }

  if (!predicted_trajectory_) {
    ROS_INFO_THROTTLE(5.0, "waiting for predicted_trajectory msg...");
    return false;
  }

  if (!current_twist_) {
    ROS_INFO_THROTTLE(5.0, "waiting for current_twist msg...");
    return false;
  }

  return true;
}

bool ObstacleCollisionCheckerNode::isDataTimeout()
{
  const auto now = ros::Time::now();

  constexpr double th_pose_timeout = 1.0;
  const auto pose_time_diff = current_pose_->header.stamp - now;
  if (pose_time_diff.toSec() > th_pose_timeout) {
    ROS_WARN_THROTTLE(1.0, "pose is timeout...");
    return true;
  }

  return false;
}

void ObstacleCollisionCheckerNode::onTimer(const ros::TimerEvent & event)
{
  current_pose_ = self_pose_listener_.getCurrentPose();
  if (obstacle_pointcloud_) {
    const auto & header = obstacle_pointcloud_->header;
    obstacle_transform_ =
      transform_listener_.getTransform("map", header.frame_id, header.stamp, ros::Duration(0.01));
  }

  if (!isDataReady()) {
    return;
  }

  if (isDataTimeout()) {
    return;
  }

  input_.current_pose = current_pose_;
  input_.obstacle_pointcloud = obstacle_pointcloud_;
  input_.obstacle_transform = obstacle_transform_;
  input_.reference_trajectory = reference_trajectory_;
  input_.predicted_trajectory = predicted_trajectory_;
  input_.current_twist = current_twist_;

  output_ = obstacle_collision_checker_->update(input_);

  updater_.force_update();

  debug_publisher_.publish<visualization_msgs::MarkerArray>("marker_array", createMarkerArray());

  time_publisher_.publish(output_.processing_time_map);
}

void ObstacleCollisionCheckerNode::onConfig(
  const ObstacleCollisionCheckerConfig & config, const uint32_t level)
{
  param_.delay_time = config.delay_time;
  param_.footprint_margin = config.footprint_margin;
  param_.max_deceleration = config.max_deceleration;
  param_.search_radius = config.search_radius;
  param_.resample_interval = config.resample_interval;
  if (obstacle_collision_checker_) {
    obstacle_collision_checker_->setParam(param_);
  }
}

void ObstacleCollisionCheckerNode::checkLaneDeparture(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (output_.will_collide) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "vehicle will collide with obstacles";
  }

  stat.summary(level, msg);
}

visualization_msgs::MarkerArray ObstacleCollisionCheckerNode::createMarkerArray() const
{
  using autoware_utils::createDefaultMarker;
  using autoware_utils::createMarkerColor;
  using autoware_utils::createMarkerScale;

  visualization_msgs::MarkerArray marker_array;

  const auto base_link_z = current_pose_->pose.position.z;

  if (output_.resampled_trajectory.points.size() >= 2) {
    // Line of resampled_trajectory
    {
      auto marker = createDefaultMarker(
        "map", "resampled_trajectory_line", 0, visualization_msgs::Marker::LINE_STRIP,
        createMarkerScale(0.05, 0, 0), createMarkerColor(1.0, 1.0, 1.0, 0.999));

      for (const auto & p : output_.resampled_trajectory.points) {
        marker.points.push_back(p.pose.position);
        marker.colors.push_back(marker.color);
      }

      marker_array.markers.push_back(marker);
    }

    // Points of resampled_trajectory
    {
      auto marker = createDefaultMarker(
        "map", "resampled_trajectory_points", 0, visualization_msgs::Marker::SPHERE_LIST,
        createMarkerScale(0.1, 0.1, 0.1), createMarkerColor(0.0, 1.0, 0.0, 0.999));

      for (const auto & p : output_.resampled_trajectory.points) {
        marker.points.push_back(p.pose.position);
        marker.colors.push_back(marker.color);
      }

      marker_array.markers.push_back(marker);
    }
  }

  // Vehicle Footprints
  {
    const auto color_ok = createMarkerColor(0.0, 1.0, 0.0, 0.5);
    const auto color_will_collide = createMarkerColor(1.0, 0.0, 0.0, 0.5);

    auto color = color_ok;
    if (output_.will_collide) {
      color = color_will_collide;
    }

    auto marker = createDefaultMarker(
      "map", "vehicle_footprints", 0, visualization_msgs::Marker::LINE_LIST,
      createMarkerScale(0.05, 0, 0), color);

    for (const auto & vehicle_footprint : output_.vehicle_footprints) {
      for (size_t i = 0; i < vehicle_footprint.size() - 1; ++i) {
        const auto p1 = vehicle_footprint.at(i);
        const auto p2 = vehicle_footprint.at(i + 1);

        marker.points.push_back(toMsg(p1.to_3d(base_link_z)));
        marker.points.push_back(toMsg(p2.to_3d(base_link_z)));
      }
    }

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}
}  // namespace obstacle_collision_checker
