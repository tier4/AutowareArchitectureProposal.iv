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

#include <autoware_utils/ros/debug_publisher.h>
#include <autoware_utils/ros/processing_time_publisher.h>
#include <autoware_utils/ros/self_pose_listener.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <ros/ros.h>

#include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>

#include <lane_departure_checker/LaneDepartureCheckerConfig.h>
#include <lane_departure_checker/lane_departure_checker.h>

namespace lane_departure_checker
{
struct NodeParam
{
  double update_rate;
  bool visualize_lanelet;
};

class LaneDepartureCheckerNode
{
public:
  LaneDepartureCheckerNode();

private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  // Subscriber
  autoware_utils::SelfPoseListener self_pose_listener_;
  ros::Subscriber sub_twist_;
  ros::Subscriber sub_lanelet_map_bin_;
  ros::Subscriber sub_route_;
  ros::Subscriber sub_reference_trajectory_;
  ros::Subscriber sub_predicted_trajectory_;

  // Data Buffer
  geometry_msgs::PoseStamped::ConstPtr current_pose_;
  geometry_msgs::TwistStamped::ConstPtr current_twist_;
  lanelet::LaneletMapPtr lanelet_map_;
  lanelet::traffic_rules::TrafficRulesPtr traffif_rules_;
  lanelet::routing::RoutingGraphPtr routing_graph_;
  autoware_planning_msgs::Route::ConstPtr route_;
  autoware_planning_msgs::Route::ConstPtr last_route_;
  lanelet::ConstLanelets route_lanelets_;
  autoware_planning_msgs::Trajectory::ConstPtr reference_trajectory_;
  autoware_planning_msgs::Trajectory::ConstPtr predicted_trajectory_;

  // Callback
  void onTwist(const geometry_msgs::TwistStamped::ConstPtr & msg);
  void onLaneletMapBin(const autoware_lanelet2_msgs::MapBin & msg);
  void onRoute(const autoware_planning_msgs::Route::ConstPtr & msg);
  void onReferenceTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr & msg);
  void onPredictedTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr & msg);

  // Publisher
  autoware_utils::DebugPublisher debug_publisher_;
  autoware_utils::ProcessingTimePublisher processing_time_publisher_;

  // Timer
  ros::Timer timer_;

  bool isDataReady();
  bool isDataTimeout();
  void onTimer(const ros::TimerEvent & event);

  // Parameter
  NodeParam node_param_;
  Param param_;

  // Dynamic Reconfigure
  void onConfig(const LaneDepartureCheckerConfig & config, const uint32_t level);
  dynamic_reconfigure::Server<LaneDepartureCheckerConfig> dynamic_reconfigure_;

  // Core
  Input input_;
  Output output_;
  std::unique_ptr<LaneDepartureChecker> lane_departure_checker_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;

  void checkLaneDeparture(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void checkTrajectoryDeviation(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Visualization
  visualization_msgs::MarkerArray createMarkerArray() const;
};
}  // namespace lane_departure_checker
