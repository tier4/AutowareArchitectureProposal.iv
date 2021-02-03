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

#include <boost/optional.hpp>
#include <map>
#include <string>
#include <vector>
#include "autoware_planning_msgs/msg/route.hpp"
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
//#include "autoware_utils/pose_deviation.h"

namespace goal_distance_calculator
{
//using autoware_utils::PoseDeviation;

struct Param
{
};

struct Input
{
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose;
  autoware_planning_msgs::msg::Route::ConstSharedPtr route;
};

struct Output
{
  //PoseDeviation goal_deviation;
};

class GoalDistanceCalculator
{
public:
  Output update(const Input & input);

  void setParam(const Param & param) {param_ = param;}

private:
  Param param_;
};
}  // namespace goal_distance_calculator
