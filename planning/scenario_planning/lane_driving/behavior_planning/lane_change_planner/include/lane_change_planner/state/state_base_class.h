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

#ifndef LANE_CHANGE_PLANNER_STATE_STATE_BASE_CLASS_H
#define LANE_CHANGE_PLANNER_STATE_STATE_BASE_CLASS_H

#include <autoware_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Point.h>
#include <lane_change_planner/data_manager.h>
#include <lane_change_planner/parameters.h>
#include <lane_change_planner/route_handler.h>
#include <iostream>
#include <string>

namespace lane_change_planner
{
enum State {
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
  autoware_planning_msgs::PathWithLaneId lane_follow_path;
  LaneChangePath lane_change_path;
  std::vector<uint64_t> lane_follow_lane_ids;
  std::vector<uint64_t> lane_change_lane_ids;
  bool lane_change_available;
  bool lane_change_ready;
};

struct DebugData
{
  std::vector<LaneChangePath> lane_change_candidate_paths;
  autoware_planning_msgs::PathWithLaneId selected_path;
  autoware_planning_msgs::PathPointWithLaneId stop_point;
  geometry_msgs::Point stop_factor_point;
};

class StateBase
{
protected:
  StateBase(
    const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr,
    const std::shared_ptr<RouteHandler> & route_handler_ptr);
  Status status_;
  LaneChangerParameters ros_parameters_;
  std::shared_ptr<DataManager> data_manager_ptr_;
  std::shared_ptr<RouteHandler> route_handler_ptr_;
  DebugData debug_data_;

public:
  virtual void entry() = 0;
  virtual void update() = 0;
  virtual State getNextState() const = 0;
  virtual State getCurrentState() const = 0;
  virtual autoware_planning_msgs::PathWithLaneId getPath() const = 0;

  Status getStatus() const;
  DebugData getDebugData() const;
};
}  // namespace lane_change_planner

#endif  // LANE_CHANGE_PLANNER_STATE_STATE_BASE_CLASS_H
