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

#include "lane_change_planner/state_machine.hpp"
#include <memory>
#include <limits>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "lane_change_planner/state/aborting_lane_change.hpp"
#include "lane_change_planner/state/blocked_by_obstacle.hpp"
#include "lane_change_planner/state/executing_lane_change.hpp"
#include "lane_change_planner/state/following_lane.hpp"
#include "lane_change_planner/state/forcing_lane_change.hpp"
#include "lane_change_planner/state/stopping_lane_change.hpp"

namespace lane_change_planner
{
StateMachine::StateMachine(
  const std::shared_ptr<DataManager> & data_manager_ptr,
  const std::shared_ptr<RouteHandler> & route_handler_ptr,
  const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr & clock)
: data_manager_ptr_(data_manager_ptr), route_handler_ptr_(route_handler_ptr), logger_(logger),
  clock_(clock)
{
}

void StateMachine::init()
{
  Status empty_status{};
  state_obj_ptr_ =
    std::make_unique<FollowingLaneState>(
    empty_status, data_manager_ptr_, route_handler_ptr_,
    logger_, clock_);
  state_obj_ptr_->entry();
}

void StateMachine::initCallback(const autoware_planning_msgs::msg::Route::ConstSharedPtr route)
{
  init();
}

void StateMachine::updateState()
{
  // update state status
  state_obj_ptr_->update();
  State current_state = state_obj_ptr_->getCurrentState();
  State next_state = state_obj_ptr_->getNextState();

  // Transit to next state
  if (next_state != current_state) {
    RCLCPP_INFO_STREAM(logger_, "changing state: " << current_state << " => " << next_state);
    const auto previous_status = state_obj_ptr_->getStatus();
    switch (next_state) {
      case State::FOLLOWING_LANE:
        state_obj_ptr_ = std::make_unique<FollowingLaneState>(
          previous_status, data_manager_ptr_, route_handler_ptr_, logger_, clock_);
        break;
      case State::EXECUTING_LANE_CHANGE:
        state_obj_ptr_ = std::make_unique<ExecutingLaneChangeState>(
          previous_status, data_manager_ptr_, route_handler_ptr_, logger_, clock_);
        break;
      case State::STOPPING_LANE_CHANGE:
        state_obj_ptr_ = std::make_unique<StoppingLaneChangeState>(
          previous_status, data_manager_ptr_, route_handler_ptr_, logger_, clock_);
        break;
      case State::ABORTING_LANE_CHANGE:
        state_obj_ptr_ = std::make_unique<AbortingLaneChangeState>(
          previous_status, data_manager_ptr_, route_handler_ptr_, logger_, clock_);
        break;
      case State::FORCING_LANE_CHANGE:
        state_obj_ptr_ = std::make_unique<ForcingLaneChangeState>(
          previous_status, data_manager_ptr_, route_handler_ptr_, logger_, clock_);
        break;
      case State::BLOCKED_BY_OBSTACLE:
        state_obj_ptr_ = std::make_unique<BlockedByObstacleState>(
          previous_status, data_manager_ptr_, route_handler_ptr_, logger_, clock_);
        break;
      case State::NO_STATE:
        break;
    }
    state_obj_ptr_->entry();
    state_obj_ptr_->update();
  }
}

autoware_planning_msgs::msg::PathWithLaneId StateMachine::getPath() const
{
  return state_obj_ptr_->getPath();
}

Status StateMachine::getStatus() const {return state_obj_ptr_->getStatus();}
DebugData StateMachine::getDebugData() const {return state_obj_ptr_->getDebugData();}
State StateMachine::getState() const {return state_obj_ptr_->getCurrentState();}

}  // namespace lane_change_planner
