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
#include <lane_change_planner/data_manager.h>
#include <lane_change_planner/route_handler.h>
#include <lane_change_planner/state/common_functions.h>
#include <lane_change_planner/state/stopping_lane_change.h>
#include <lane_change_planner/utilities.h>

#include <lanelet2_extension/utility/utilities.h>

namespace lane_change_planner
{
StoppingLaneChangeState::StoppingLaneChangeState(
  const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr,
  const std::shared_ptr<RouteHandler> & route_handler_ptr)
: StateBase(status, data_manager_ptr, route_handler_ptr)
{
}
State StoppingLaneChangeState::getCurrentState() const { return State::STOPPING_LANE_CHANGE; }

void StoppingLaneChangeState::entry()
{
  ros_parameters_ = data_manager_ptr_->getLaneChangerParameters();

  original_lanes_ = route_handler_ptr_->getLaneletsFromIds(status_.lane_follow_lane_ids);
  target_lanes_ = route_handler_ptr_->getLaneletsFromIds(status_.lane_change_lane_ids);
  status_.lane_change_available = false;
  status_.lane_change_ready = false;
}

autoware_planning_msgs::PathWithLaneId StoppingLaneChangeState::getPath() const
{
  return isVehicleInOriginalLanes() ? status_.lane_change_path.path : stop_path_;
}

void StoppingLaneChangeState::update()
{
  current_twist_ = data_manager_ptr_->getCurrentSelfVelocity();
  current_pose_ = data_manager_ptr_->getCurrentSelfPose();
  dynamic_objects_ = data_manager_ptr_->getDynamicObjects();
  if (isVehicleInOriginalLanes()) {
    stop_path_ = setStopPoint(status_.lane_change_path.path);
  }
}

State StoppingLaneChangeState::getNextState() const
{
  if (isSafe() || !isVehicleInOriginalLanes()) {
    return State::EXECUTING_LANE_CHANGE;
  }
  return State::STOPPING_LANE_CHANGE;
}

bool StoppingLaneChangeState::isSafe() const
{
  // check if lane change path is still safe
  bool is_path_safe = false;
  {
    constexpr double check_distance = 100.0;
    // get lanes used for detection
    const auto & check_lanes = route_handler_ptr_->getCheckTargetLanesFromPath(
      status_.lane_change_path.path, target_lanes_, check_distance);

    is_path_safe = state_machine::common_functions::isLaneChangePathSafe(
      status_.lane_change_path.path, original_lanes_, check_lanes, dynamic_objects_,
      current_pose_.pose, current_twist_->twist, ros_parameters_, false,
      status_.lane_change_path.acceleration);
  }
  return is_path_safe;
}

bool StoppingLaneChangeState::isVehicleInOriginalLanes() const
{
  const auto lane_length = lanelet::utils::getLaneletLength2d(original_lanes_);
  const auto lane_poly = lanelet::utils::getPolygonFromArcLength(original_lanes_, 0, lane_length);
  const auto vehicle_poly = util::getVehiclePolygon(
    current_pose_.pose, ros_parameters_.vehicle_width, ros_parameters_.base_link2front);

  std::vector<lanelet::BasicPolygon2d> intersection_poly;
  boost::geometry::intersection(
    lanelet::utils::to2D(vehicle_poly).basicPolygon(),
    lanelet::utils::to2D(lane_poly).basicPolygon(), intersection_poly);

  const double vehicle_area =
    boost::geometry::area(lanelet::utils::to2D(vehicle_poly).basicPolygon());
  const double intersection_area = boost::geometry::area(intersection_poly.at(0));

  return intersection_area / vehicle_area > 0.9;
}

autoware_planning_msgs::PathWithLaneId StoppingLaneChangeState::setStopPoint(
  const autoware_planning_msgs::PathWithLaneId & path)
{
  autoware_planning_msgs::PathWithLaneId modified_path = path;
  debug_data_.stop_point = util::insertStopPoint(0.1, &modified_path);
  return modified_path;
}

}  // namespace lane_change_planner
