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

#include "scene_module/occlusion_spot/scene_occlusion_spot_in_public_road.hpp"

#include "lanelet2_core/primitives/BasicRegulatoryElements.h"
#include "lanelet2_extension/utility/utilities.hpp"
#include "scene_module/occlusion_spot/occlusion_spot_utils.hpp"
#include "scene_module/occlusion_spot/risk_predictive_braking.hpp"
#include "utilization/boost_geometry_helper.hpp"
#include "utilization/util.hpp"

#include "geometry_msgs/msg/point.h"

#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
using occlusion_spot_utils::PossibleCollisionInfo;
using occlusion_spot_utils::ROAD_TYPE::PUBLIC;

OcclusionSpotInPublicModule::OcclusionSpotInPublicModule(
  const int64_t module_id, [[maybe_unused]] std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock)
{
  param_ = planner_param;
}

bool OcclusionSpotInPublicModule::modifyPathVelocity(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  [[maybe_unused]] autoware_planning_msgs::msg::StopReason * stop_reason)
{
  if (path->points.size() < 2) {
    return true;
  }
  param_.vehicle_info.baselink_to_front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  param_.vehicle_info.vehicle_width = planner_data_->vehicle_info_.vehicle_width_m;
  if (!planner_data_->current_velocity) {
    return true;
  }
  const geometry_msgs::msg::Pose ego_pose = planner_data_->current_pose.pose;
  const double ego_velocity = planner_data_->current_velocity->twist.linear.x;
  const auto & lanelet_map_ptr = planner_data_->lanelet_map;
  const auto & routing_graph_ptr = planner_data_->routing_graph;
  const auto & traffic_rules_ptr = planner_data_->traffic_rules;
  const auto & dynamic_obj_arr_ptr = planner_data_->predicted_objects;

  if (!lanelet_map_ptr || !traffic_rules_ptr || !dynamic_obj_arr_ptr || !routing_graph_ptr) {
    return true;
  }
  std::vector<lanelet::BasicLineString2d> attension_line;
  std::set<int> ids;
  for (const auto & p : path->points) {
    for (const int id : p.lane_ids) {
      ids.insert(id);
    }
  }
  occlusion_spot_utils::generateCenterLaneLine(
    *path, routing_graph_ptr, lanelet_map_ptr, attension_line);
  debug_data_.attension_line = attension_line;
  std::vector<Point> debug_points;
  std::vector<PredictedObject> obj = occlusion_spot_utils::getParkedVehicles(
    *dynamic_obj_arr_ptr, attension_line, param_, debug_data_.parked_vehicle_point);
  PathWithLaneId interp_path;
  occlusion_spot_utils::splineInterpolate(*path, 0.5, &interp_path, logger_);
  int closest_idx = -1;
  if (!planning_utils::calcClosestIndex<PathWithLaneId>(
        interp_path, ego_pose, closest_idx, param_.dist_thr, param_.angle_thr)) {
    return true;
  }
  double offset_from_start_to_ego = offsetFromStartToEgo(interp_path, ego_pose, closest_idx);
  //! Note : Arc Lane from idx[0] to end therefore DO NOT consider offset here
  std::vector<PossibleCollisionInfo> possible_collisions =
    occlusion_spot_utils::generatePossibleCollisionBehindParkedVehicle(
      interp_path, param_, offset_from_start_to_ego, obj);
  //! Note : consider offset_from_start_to_ego here
  occlusion_spot_utils::filterPossibleCollisionByRoadType(
    lanelet_map_ptr, offset_from_start_to_ego, *path, possible_collisions, PUBLIC, param_);
  //! Arc Lane from idx[0] to end therefore DO NOT consider offset here
  occlusion_spot_utils::calcSlowDownPointsForPossibleCollision(
    0, interp_path, -offset_from_start_to_ego - param_.safety_margin, possible_collisions);
  //! apply safe velocity using ebs and pbs deceleration
  applySafeVelocityConsideringPossibleCollison(
    path, possible_collisions, ego_velocity, param_.public_road, param_);

  debug_data_.possible_collisions = possible_collisions;
  return true;
}

}  // namespace behavior_velocity_planner
