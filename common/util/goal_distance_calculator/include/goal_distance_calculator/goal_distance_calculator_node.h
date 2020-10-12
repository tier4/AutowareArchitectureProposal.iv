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

#include <ros/ros.h>

#include <autoware_planning_msgs/Route.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include <autoware_utils/ros/debug_publisher.h>
#include <autoware_utils/ros/self_pose_listener.h>

#include <goal_distance_calculator/goal_distance_calculator.h>

namespace goal_distance_calculator
{
struct NodeParam
{
  double update_rate;
  bool oneshot;
};

class GoalDistanceCalculatorNode
{
public:
  GoalDistanceCalculatorNode();

private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  // Subscriber
  autoware_utils::SelfPoseListener self_pose_listener_;
  ros::Subscriber sub_route_;

  // Data Buffer
  geometry_msgs::PoseStamped::ConstPtr current_pose_;
  autoware_planning_msgs::Route::ConstPtr route_;

  // Callback
  void onRoute(const autoware_planning_msgs::Route::ConstPtr & msg);

  // Publisher
  autoware_utils::DebugPublisher debug_publisher_;

  // Timer
  ros::Timer timer_;

  bool isDataReady();
  bool isDataTimeout();
  void onTimer(const ros::TimerEvent & event);

  // Parameter
  NodeParam node_param_;
  Param param_;

  // Core
  Input input_;
  Output output_;
  std::unique_ptr<GoalDistanceCalculator> goal_distance_calculator_;
};
}  // namespace goal_distance_calculator
