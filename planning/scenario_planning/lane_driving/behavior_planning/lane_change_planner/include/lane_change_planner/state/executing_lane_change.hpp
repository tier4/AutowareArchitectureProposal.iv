// Copyright 2019 Autoware Foundation. All rights reserved.
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

#ifndef LANE_CHANGE_PLANNER__STATE__EXECUTING_LANE_CHANGE_HPP_
#define LANE_CHANGE_PLANNER__STATE__EXECUTING_LANE_CHANGE_HPP_

#include <lane_change_planner/state/state_base_class.hpp>
#include <autoware_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/dynamic_object_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <lanelet2_core/primitives/Lanelet.h>
#include <memory>

namespace lane_change_planner
{
class ExecutingLaneChangeState : public StateBase
{
private:
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_twist_;
  autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr dynamic_objects_;
  double start_distance_;

  lanelet::ConstLanelets original_lanes_;
  lanelet::ConstLanelets target_lanes_;

  // State transition conditions
  bool isNearEndOfLane() const;
  bool isCurrentSpeedLow() const;
  bool isAbortConditionSatisfied() const;
  bool hasFinishedLaneChange() const;

public:
  ExecutingLaneChangeState(
    const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr,
    const std::shared_ptr<RouteHandler> & route_handler_ptr,
    const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr & clock);

  // override virtual functions
  void entry() override;
  void update() override;
  State getNextState() const override;
  State getCurrentState() const override;
  autoware_planning_msgs::msg::PathWithLaneId getPath() const override;
};
}  // namespace lane_change_planner

#endif  // LANE_CHANGE_PLANNER__STATE__EXECUTING_LANE_CHANGE_HPP_
