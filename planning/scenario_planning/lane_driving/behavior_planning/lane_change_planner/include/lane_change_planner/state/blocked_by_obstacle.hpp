// Copyright 2019 Autoware Foundation. All rights reserved.
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

#ifndef LANE_CHANGE_PLANNER__STATE__BLOCKED_BY_OBSTACLE_HPP_
#define LANE_CHANGE_PLANNER__STATE__BLOCKED_BY_OBSTACLE_HPP_

#include <memory>
#include <vector>

#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lane_change_planner/state/state_base_class.hpp"
#include "lanelet2_core/primitives/Primitive.h"

namespace lane_change_planner
{
class BlockedByObstacleState : public StateBase
{
private:
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_twist_;
  autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr dynamic_objects_;
  bool lane_change_approved_;
  bool force_lane_change_;
  bool found_valid_path_;
  bool found_safe_path_;
  lanelet::ConstLanelets current_lanes_;
  lanelet::ConstLanelets lane_change_lanes_;

  // State transition conditions
  bool foundSafeLaneChangePath() const;
  bool foundValidPath() const;
  bool hasEnoughDistanceToComeBack(const lanelet::ConstLanelets & target_lanes) const;
  bool isLaneChangeApproved() const;
  bool isLaneBlocked() const;
  bool isOutOfCurrentLanes() const;
  bool isLaneChangeAvailable() const;
  bool isLaneChangeReady() const;
  bool laneChangeForcedByOperator() const;

  // utility function
  std::vector<autoware_perception_msgs::msg::DynamicObject> getBlockingObstacles() const;
  autoware_planning_msgs::msg::PathWithLaneId setStopPointFromObstacle(
    const autoware_planning_msgs::msg::PathWithLaneId & path);

public:
  BlockedByObstacleState(
    const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr,
    const std::shared_ptr<RouteHandler> & route_handler_ptr, const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr & clock);

  // override virtual functions
  void entry() override;
  void update() override;
  State getNextState() const override;
  State getCurrentState() const override;
  autoware_planning_msgs::msg::PathWithLaneId getPath() const override;
};
}  // namespace lane_change_planner

#endif  // LANE_CHANGE_PLANNER__STATE__BLOCKED_BY_OBSTACLE_HPP_
