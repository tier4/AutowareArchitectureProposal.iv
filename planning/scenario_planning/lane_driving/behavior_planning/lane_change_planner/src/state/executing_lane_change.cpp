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
#include <lane_change_planner/state/executing_lane_change.h>
#include <lane_change_planner/utilities.h>

#include <lanelet2_extension/utility/utilities.h>
#include <tf2/utils.h>

namespace lane_change_planner
{
ExecutingLaneChangeState::ExecutingLaneChangeState(
  const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr,
  const std::shared_ptr<RouteHandler> & route_handler_ptr)
: StateBase(status, data_manager_ptr, route_handler_ptr)
{
}

State ExecutingLaneChangeState::getCurrentState() const { return State::EXECUTING_LANE_CHANGE; }

void ExecutingLaneChangeState::entry()
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

autoware_planning_msgs::PathWithLaneId ExecutingLaneChangeState::getPath() const
{
  return status_.lane_change_path.path;
}

void ExecutingLaneChangeState::update()
{
  current_twist_ = data_manager_ptr_->getCurrentSelfVelocity();
  current_pose_ = data_manager_ptr_->getCurrentSelfPose();
  dynamic_objects_ = data_manager_ptr_->getDynamicObjects();

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

State ExecutingLaneChangeState::getNextState() const
{
  if (isAbortConditionSatisfied()) {
    if (isNearEndOfLane() && isCurrentSpeedLow()) {
      return State::STOPPING_LANE_CHANGE;
    }
    return State::FOLLOWING_LANE;
  }

  if (hasFinishedLaneChange()) {
    return State::FOLLOWING_LANE;
  }
  return State::EXECUTING_LANE_CHANGE;
}

bool ExecutingLaneChangeState::isNearEndOfLane() const
{
  const double threshold = 5 + ros_parameters_.minimum_lane_change_length;
  return std::max(0.0, util::getDistanceToEndOfLane(current_pose_.pose, original_lanes_)) <
         threshold;
}

bool ExecutingLaneChangeState::isCurrentSpeedLow() const
{
  const double threshold_kmph = 10;
  return util::l2Norm(current_twist_->twist.linear) < threshold_kmph * 1000 / 3600;
}

bool ExecutingLaneChangeState::isAbortConditionSatisfied() const
{
  // check abort enable flag
  if (!ros_parameters_.enable_abort_lane_change) {
    return false;
  }

  // find closest lanelet in original lane
  lanelet::ConstLanelet closest_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(
        original_lanes_, current_pose_.pose, &closest_lanelet)) {
    ROS_ERROR_THROTTLE(
      1, "Failed to find closest lane! Lane change aborting function is not working!");
    return false;
  }

  // check if lane change path is still safe
  bool is_path_safe = false;
  {
    constexpr double check_distance = 100.0;
    // get lanes used for detection
    const auto & path = status_.lane_change_path;
    const double check_distance_with_path =
      check_distance + path.preparation_length + path.lane_change_length;
    const auto check_lanes = route_handler_ptr_->getCheckTargetLanesFromPath(
      path.path, target_lanes_, check_distance_with_path);

    is_path_safe = state_machine::common_functions::isLaneChangePathSafe(
      path.path, original_lanes_, check_lanes, dynamic_objects_, current_pose_.pose,
      current_twist_->twist, ros_parameters_, false, status_.lane_change_path.acceleration);
  }

  // check vehicle velocity thresh
  const bool is_velocity_low =
    util::l2Norm(current_twist_->twist.linear) < ros_parameters_.abort_lane_change_velocity_thresh;

  // check if vehicle is within lane
  bool is_within_original_lane = false;
  {
    const auto lane_length = lanelet::utils::getLaneletLength2d(original_lanes_);
    const auto lane_poly = lanelet::utils::getPolygonFromArcLength(original_lanes_, 0, lane_length);
    const auto vehicle_poly = util::getVehiclePolygon(
      current_pose_.pose, ros_parameters_.vehicle_width, ros_parameters_.base_link2front);
    is_within_original_lane = boost::geometry::within(
      lanelet::utils::to2D(vehicle_poly).basicPolygon(),
      lanelet::utils::to2D(lane_poly).basicPolygon());
  }

  // check distance from original lane's centerline
  bool is_distance_small = false;
  {
    const auto centerline2d = lanelet::utils::to2D(closest_lanelet.centerline()).basicLineString();
    lanelet::BasicPoint2d vehicle_pose2d(
      current_pose_.pose.position.x, current_pose_.pose.position.y);
    const double distance = lanelet::geometry::distance2d(centerline2d, vehicle_pose2d);
    is_distance_small = distance < ros_parameters_.abort_lane_change_distance_thresh;
  }

  // check angle thresh from original lane
  bool is_angle_diff_small = false;
  {
    const double lane_angle =
      lanelet::utils::getLaneletAngle(closest_lanelet, current_pose_.pose.position);
    const double vehicle_yaw = tf2::getYaw(current_pose_.pose.orientation);
    const double yaw_diff = util::normalizeRadian(lane_angle - vehicle_yaw);
    is_angle_diff_small = std::abs(yaw_diff) < ros_parameters_.abort_lane_change_angle_thresh;
  }

  // abort only if velocity is low or vehicle pose is close enough
  if (!is_path_safe) {
    if (is_velocity_low && is_within_original_lane) {
      return true;
    }
    if (is_distance_small && is_angle_diff_small) {
      return true;
    }
    ROS_WARN_STREAM_THROTTLE(
      1, "DANGER!!! Path is not safe anymore, but it is too late to abort! Please be cautious");
  }

  return false;
}  // namespace lane_change_planner

bool ExecutingLaneChangeState::hasFinishedLaneChange() const
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
