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

#include <mission_planner/goal_pose_visualizer.h>

namespace mission_planner
{
GoalPoseVisualizer::GoalPoseVisualizer()
{
  sub_route_ = pnh_.subscribe("input/route", 10, &GoalPoseVisualizer::echoBackRouteCallback, this);

  pub_goal_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("output/goal_pose", 1, true);
}

void GoalPoseVisualizer::echoBackRouteCallback(const autoware_planning_msgs::RouteConstPtr & msg)
{
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header = msg->header;
  goal_pose.pose = msg->goal_pose;
  pub_goal_pose_.publish(goal_pose);
}
}  // namespace mission_planner
