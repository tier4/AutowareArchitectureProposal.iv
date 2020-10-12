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

#include <goal_distance_calculator/goal_distance_calculator_node.h>

#include <autoware_utils/math/unit_conversion.h>

namespace goal_distance_calculator
{
GoalDistanceCalculatorNode::GoalDistanceCalculatorNode()
{
  // Node Parameter
  private_nh_.param("update_rate", node_param_.update_rate, 10.0);
  private_nh_.param("oneshot", node_param_.oneshot, true);

  // Core Parameter
  // Nothing

  // Core
  goal_distance_calculator_ = std::make_unique<GoalDistanceCalculator>();
  goal_distance_calculator_->setParam(param_);

  // Subscriber
  sub_route_ = private_nh_.subscribe(
    "/planning/mission_planning/route", 1, &GoalDistanceCalculatorNode::onRoute, this);

  // Publisher
  // Nothing

  // Wait for first self pose
  self_pose_listener_.waitForFirstPose();

  // Timer
  timer_ = private_nh_.createTimer(
    ros::Rate(node_param_.update_rate), &GoalDistanceCalculatorNode::onTimer, this);
}

void GoalDistanceCalculatorNode::onRoute(const autoware_planning_msgs::Route::ConstPtr & msg)
{
  route_ = msg;
}

bool GoalDistanceCalculatorNode::isDataReady()
{
  if (!current_pose_) {
    ROS_INFO_THROTTLE(5.0, "waiting for current_pose...");
    return false;
  }

  if (!route_) {
    ROS_INFO_THROTTLE(5.0, "waiting for route msg...");
    return false;
  }

  return true;
}

bool GoalDistanceCalculatorNode::isDataTimeout()
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

void GoalDistanceCalculatorNode::onTimer(const ros::TimerEvent & event)
{
  current_pose_ = self_pose_listener_.getCurrentPose();

  if (!isDataReady()) {
    return;
  }

  if (isDataTimeout()) {
    return;
  }

  input_.current_pose = current_pose_;
  input_.route = route_;

  output_ = goal_distance_calculator_->update(input_);

  {
    using autoware_utils::rad2deg;
    const auto & deviation = output_.goal_deviation;

    debug_publisher_.publish<std_msgs::Float64>("deviation/lateral", deviation.lateral);
    debug_publisher_.publish<std_msgs::Float64>("deviation/longitudinal", deviation.longitudinal);
    debug_publisher_.publish<std_msgs::Float64>("deviation/yaw", deviation.yaw);
    debug_publisher_.publish<std_msgs::Float64>("deviation/yaw_deg", rad2deg(deviation.yaw));

    ROS_INFO_THROTTLE(
      1.0, "lateral: %f[mm], longitudinal: %f[mm], yaw: %f[deg]", 1000 * deviation.lateral,
      1000 * deviation.longitudinal, rad2deg(deviation.yaw));
  }

  if (node_param_.oneshot) {
    ros::shutdown();
  }
}

}  // namespace goal_distance_calculator
