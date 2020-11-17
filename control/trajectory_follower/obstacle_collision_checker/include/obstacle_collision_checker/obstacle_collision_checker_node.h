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

#include <memory>

#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <autoware_utils/geometry/geometry.h>
#include <autoware_utils/ros/debug_publisher.h>
#include <autoware_utils/ros/processing_time_publisher.h>
#include <autoware_utils/ros/self_pose_listener.h>
#include <autoware_utils/ros/transform_listener.h>

#include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>

#include <obstacle_collision_checker/ObstacleCollisionCheckerConfig.h>
#include <obstacle_collision_checker/obstacle_collision_checker.h>

namespace obstacle_collision_checker
{
struct NodeParam
{
  double update_rate;
};

class ObstacleCollisionCheckerNode
{
public:
  ObstacleCollisionCheckerNode();

private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  // Subscriber
  autoware_utils::SelfPoseListener self_pose_listener_;
  TransformListener transform_listener_;
  ros::Subscriber sub_obstacle_pointcloud_;
  ros::Subscriber sub_reference_trajectory_;
  ros::Subscriber sub_predicted_trajectory_;
  ros::Subscriber sub_twist_;

  // Data Buffer
  geometry_msgs::PoseStamped::ConstPtr current_pose_;
  geometry_msgs::TwistStamped::ConstPtr current_twist_;
  sensor_msgs::PointCloud2::ConstPtr obstacle_pointcloud_;
  geometry_msgs::TransformStamped::ConstPtr obstacle_transform_;
  autoware_planning_msgs::Trajectory::ConstPtr reference_trajectory_;
  autoware_planning_msgs::Trajectory::ConstPtr predicted_trajectory_;

  // Callback
  void onObstaclePointcloud(const sensor_msgs::PointCloud2::ConstPtr & msg);
  void onReferenceTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr & msg);
  void onPredictedTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr & msg);
  void onTwist(const geometry_msgs::TwistStamped::ConstPtr & msg);

  // Publisher
  autoware_utils::DebugPublisher debug_publisher_;
  autoware_utils::ProcessingTimePublisher time_publisher_;

  // Timer
  ros::Timer timer_;

  bool isDataReady();
  bool isDataTimeout();
  void onTimer(const ros::TimerEvent & event);

  // Parameter
  NodeParam node_param_;
  Param param_;

  // Dynamic Reconfigure
  void onConfig(const ObstacleCollisionCheckerConfig & config, const uint32_t level);
  dynamic_reconfigure::Server<ObstacleCollisionCheckerConfig> dynamic_reconfigure_;

  // Core
  Input input_;
  Output output_;
  std::unique_ptr<ObstacleCollisionChecker> obstacle_collision_checker_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;

  void checkLaneDeparture(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Visualization
  visualization_msgs::MarkerArray createMarkerArray() const;
};
}  // namespace obstacle_collision_checker
