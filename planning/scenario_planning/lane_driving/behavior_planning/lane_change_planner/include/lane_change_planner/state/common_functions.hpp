// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LANE_CHANGE_PLANNER__STATE__COMMON_FUNCTIONS_HPP_
#define LANE_CHANGE_PLANNER__STATE__COMMON_FUNCTIONS_HPP_

#include <memory>
#include <vector>

#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lane_change_planner/state/state_base_class.hpp"
#include "lanelet2_core/primitives/Primitive.h"

namespace lane_change_planner
{
namespace state_machine
{
namespace common_functions
{
std::vector<LaneChangePath> selectValidPaths(
  const std::vector<LaneChangePath> & paths, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes,
  const lanelet::routing::RoutingGraphContainer & overall_graphs,
  const geometry_msgs::msg::Pose & current_pose, const bool isInGoalRouteSection,
  const geometry_msgs::msg::Pose & goal_pose);
bool selectSafePath(
  const std::vector<LaneChangePath> & paths, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes,
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr & dynamic_objects,
  const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Twist & current_twist,
  const LaneChangerParameters & ros_parameters, LaneChangePath * selected_path,
  const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr & clock);
bool isLaneChangePathSafe(
  const autoware_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr & dynamic_objects,
  const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Twist & current_twist,
  const LaneChangerParameters & ros_parameters, const bool use_buffer, const double acceleration,
  const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr & clock);
bool hasEnoughDistance(
  const LaneChangePath & path, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes, const geometry_msgs::msg::Pose & current_pose,
  const bool isInGoalRouteSection, const geometry_msgs::msg::Pose & goal_pose,
  const lanelet::routing::RoutingGraphContainer & overall_graphs);
bool isObjectFront(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & obj_pose);
}  // namespace common_functions
}  // namespace state_machine
}  // namespace lane_change_planner

#endif  // LANE_CHANGE_PLANNER__STATE__COMMON_FUNCTIONS_HPP_
