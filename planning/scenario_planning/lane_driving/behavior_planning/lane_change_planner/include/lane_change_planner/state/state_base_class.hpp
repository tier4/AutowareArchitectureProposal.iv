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

#ifndef LANE_CHANGE_PLANNER__STATE__STATE_BASE_CLASS_HPP_
#define LANE_CHANGE_PLANNER__STATE__STATE_BASE_CLASS_HPP_

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include "autoware_planning_msgs/msg/path_with_lane_id.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "lane_change_planner/data_manager.hpp"
#include "lane_change_planner/parameters.hpp"
#include "lane_change_planner/route_handler.hpp"

namespace lane_change_planner
{
enum State
{
  NO_STATE,
  FOLLOWING_LANE,
  EXECUTING_LANE_CHANGE,
  ABORTING_LANE_CHANGE,
  STOPPING_LANE_CHANGE,
  FORCING_LANE_CHANGE,
  BLOCKED_BY_OBSTACLE
};
std::ostream & operator<<(std::ostream & ostream, const State & state);

struct Status
{
  autoware_planning_msgs::msg::PathWithLaneId lane_follow_path{};
  LaneChangePath lane_change_path{};
  std::vector<uint64_t> lane_follow_lane_ids;
  std::vector<uint64_t> lane_change_lane_ids;
  bool lane_change_available;
  bool lane_change_ready;
};

struct DebugData
{
  std::vector<LaneChangePath> lane_change_candidate_paths;
  autoware_planning_msgs::msg::PathWithLaneId selected_path;
  autoware_planning_msgs::msg::PathPointWithLaneId stop_point;
  geometry_msgs::msg::Point stop_factor_point;
};

class StateBase
{
protected:
  StateBase(
    const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr,
    const std::shared_ptr<RouteHandler> & route_handler_ptr,
    const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr & clock);
  Status status_;
  LaneChangerParameters ros_parameters_;
  std::shared_ptr<DataManager> data_manager_ptr_;
  std::shared_ptr<RouteHandler> route_handler_ptr_;
  DebugData debug_data_;
  const rclcpp::Logger logger_;
  const rclcpp::Clock::SharedPtr clock_;

public:
  virtual ~StateBase() = default;
  virtual void entry() = 0;
  virtual void update() = 0;
  virtual State getNextState() const = 0;
  virtual State getCurrentState() const = 0;
  virtual autoware_planning_msgs::msg::PathWithLaneId getPath() const = 0;

  Status getStatus() const;
  DebugData getDebugData() const;
};
}  // namespace lane_change_planner

#endif  // LANE_CHANGE_PLANNER__STATE__STATE_BASE_CLASS_HPP_
