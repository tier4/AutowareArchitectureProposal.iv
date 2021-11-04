// Copyright 2021 Tier IV, Inc.
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

#include <lanelet2_extension/utility/utilities.hpp>
#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>
#include <scene_module/occlusion_spot/risk_predictive_braking.hpp>
#include <scene_module/occlusion_spot/scene_occlusion_spot_in_public_road.hpp>
#include <utilization/util.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <memory>
#include <vector>

namespace behavior_velocity_planner
{
OcclusionSpotInPublicModule::OcclusionSpotInPublicModule(
  const int64_t module_id, [[maybe_unused]] std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock)
{
  param_ = planner_param;
}

bool OcclusionSpotInPublicModule::modifyPathVelocity(
  autoware_planning_msgs::msg::PathWithLaneId * path,
  [[maybe_unused]] autoware_planning_msgs::msg::StopReason * stop_reason)
{
  if (path->points.size() < 2) {
    return true;
  }
  param_.vehicle_info.baselink_to_front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  param_.vehicle_info.vehicle_width = planner_data_->vehicle_info_.vehicle_width_m;

  const geometry_msgs::msg::Pose ego_pose = planner_data_->current_pose.pose;
  const double ego_velocity = planner_data_->current_velocity->twist.linear.x;
  const auto & lanelet_map_ptr = planner_data_->lanelet_map;
  const auto & traffic_rules_ptr = planner_data_->traffic_rules;
  const auto & dynamic_obj_arr_ptr = planner_data_->dynamic_objects;
  if (!lanelet_map_ptr || !traffic_rules_ptr || !dynamic_obj_arr_ptr) {
    return true;
  }
  int closest_idx = -1;
  if (!planning_utils::calcClosestIndex<autoware_planning_msgs::msg::PathWithLaneId>(
        *path, ego_pose, closest_idx, param_.dist_thr, param_.angle_thr)) {
    return true;
  }
  const auto target_road_type = occlusion_spot_utils::ROAD_TYPE::PUBLIC;
  autoware_planning_msgs::msg::PathWithLaneId limited_path;
  double offset_from_ego_to_closest = 0;
  double offset_from_closest_to_target = 0;
  {
    // extract lanelet that includes target_road_type only
    if (!behavior_velocity_planner::occlusion_spot_utils::extractTargetRoad(
          closest_idx, lanelet_map_ptr, param_.detection_area_length, *path,
          offset_from_closest_to_target, limited_path, target_road_type)) {
      return true;
    }
    // use path point as origin for stability
    offset_from_ego_to_closest =
      -planning_utils::transformRelCoordinate2D(ego_pose, path->points[closest_idx].point.pose)
         .position.x;
  }
  if (limited_path.points.size() < 4) {
    return true;
  }
  std::vector<behavior_velocity_planner::occlusion_spot_utils::PossibleCollisionInfo>
    possible_collisions;
  RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 3000, "closest_idx : " << closest_idx);
  RCLCPP_DEBUG_STREAM_THROTTLE(
    logger_, *clock_, 3000, "offset_from_ego_to_closest : " << offset_from_ego_to_closest);
  const double offset_from_ego_to_target =
    offset_from_ego_to_closest + offset_from_closest_to_target;
  behavior_velocity_planner::occlusion_spot_utils::createPossibleCollisionBehindParkedVehicle(
    possible_collisions, limited_path, param_, offset_from_ego_to_target, dynamic_obj_arr_ptr);
  // set orientation to each possible collision
  behavior_velocity_planner::occlusion_spot_utils::calcVelocityAndHeightToPossibleCollision(
    closest_idx, *path, offset_from_ego_to_target, possible_collisions);
  // apply safe velocity using ebs and pbs deceleration
  applySafeVelocityConsideringPossibleCollison(
    path, possible_collisions, ego_velocity, param_.public_road, param_);

  debug_data_.possible_collisions = possible_collisions;
  return true;
}

}  // namespace behavior_velocity_planner
