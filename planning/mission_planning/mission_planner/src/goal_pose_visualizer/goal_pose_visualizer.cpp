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

#include <mission_planner/goal_pose_visualizer.hpp>

namespace mission_planner
{
GoalPoseVisualizer::GoalPoseVisualizer() : Node("goal_pose_visualizer_node")
{
  sub_route_ = create_subscription<autoware_planning_msgs::msg::Route>(
    "input/route", rclcpp::QoS{1},
    std::bind(&GoalPoseVisualizer::echoBackRouteCallback, this, std::placeholders::_1));
  pub_goal_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "output/goal_pose", rclcpp::QoS{1}.transient_local());
}

void GoalPoseVisualizer::echoBackRouteCallback(
  const autoware_planning_msgs::msg::Route::ConstSharedPtr msg)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header = msg->header;
  goal_pose.pose = msg->goal_pose;
  pub_goal_pose_->publish(goal_pose);
}
}  // namespace mission_planner
