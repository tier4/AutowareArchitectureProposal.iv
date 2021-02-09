/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#ifndef LANE_CHANGE_PLANNER_STATE_ABORTING_LANE_CHANGE_HPP
#define LANE_CHANGE_PLANNER_STATE_ABORTING_LANE_CHANGE_HPP

#include <lane_change_planner/state/state_base_class.hpp>

namespace lane_change_planner
{
class AbortingLaneChangeState : public StateBase
{
private:
  // State transition conditions
  bool hasReturnedToOriginalLane() const;

public:
  AbortingLaneChangeState(
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

#endif  // LANE_CHANGE_PLANNER_STATE_ABORTING_LANE_CHANGE_H
