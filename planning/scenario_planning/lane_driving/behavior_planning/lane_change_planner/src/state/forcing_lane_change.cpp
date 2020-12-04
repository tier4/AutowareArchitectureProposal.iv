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
#include <lane_change_planner/state/forcing_lane_change.h>
#include <lane_change_planner/utilities.h>

#include <lanelet2_extension/utility/utilities.h>

namespace lane_change_planner
{
ForcingLaneChangeState::ForcingLaneChangeState(
  const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr,
  const std::shared_ptr<RouteHandler> & route_handler_ptr)
: StateBase(status, data_manager_ptr, route_handler_ptr)
{
}

State ForcingLaneChangeState::getCurrentState() const { return State::FORCING_LANE_CHANGE; }

void ForcingLaneChangeState::entry()
{
  ros_parameters_ = data_manager_ptr_->getLaneChangerParameters();
  original_lanes_ = route_handler_ptr_->getLaneletsFromIds(status_.lane_follow_lane_ids);
  target_lanes_ = route_handler_ptr_->getLaneletsFromIds(status_.lane_change_lane_ids);
  status_.lane_change_available = false;
  status_.lane_change_ready = false;

  // get start arclength
  const auto start = data_manager_ptr_->getCurrentSelfPose();
  const auto arclength_start =
    lanelet::utils::getArcCoordinates(target_lanes_, start.pose);
  start_distance_ = arclength_start.length;
}

autoware_planning_msgs::PathWithLaneId ForcingLaneChangeState::getPath() const
{
  return status_.lane_change_path.path;
}

void ForcingLaneChangeState::update()
{
  current_pose_ = data_manager_ptr_->getCurrentSelfPose();

  // update path
  {
    lanelet::ConstLanelets lanes;
    lanes.insert(lanes.end(), original_lanes_.begin(), original_lanes_.end());
    lanes.insert(lanes.end(), target_lanes_.begin(), target_lanes_.end());

    const double width = ros_parameters_.drivable_area_width;
    const double height = ros_parameters_.drivable_area_height;
    const double resolution = ros_parameters_.drivable_area_resolution;
    status_.lane_change_path.path.drivable_area = util::generateDrivableArea(
      lanes, current_pose_, width, height, resolution, ros_parameters_.vehicle_length,
      *route_handler_ptr_);
  }
}

State ForcingLaneChangeState::getNextState() const
{
  if (hasFinishedLaneChange()) {
    return State::FOLLOWING_LANE;
  }
  return State::FORCING_LANE_CHANGE;
}

bool ForcingLaneChangeState::hasFinishedLaneChange() const
{
  const auto arclength_current =
    lanelet::utils::getArcCoordinates(target_lanes_, current_pose_.pose);
  const double travel_distance = arclength_current.length - start_distance_;
  const double finish_distance = status_.lane_change_path.preparation_length +
                                 status_.lane_change_path.lane_change_length +
                                 ros_parameters_.lane_change_finish_judge_buffer;
  return travel_distance > finish_distance;
}

}  // namespace lane_change_planner
