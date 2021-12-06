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

#include "autoware_utils/geometry/geometry.hpp"
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
using occlusion_spot_utils::ROAD_TYPE;
std::pair<double, double> extractTargetRoadArcLength(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const double max_range, const PathWithLaneId & path,
  const ROAD_TYPE & target_road_type)
{
  bool found_target = false;
  double start_dist = 0;
  double dist_sum = 0;
  // search lanelet that includes target_road_type only
  for (size_t i = 0; i < path.points.size() - 1; i++) {
    ROAD_TYPE search_road_type = occlusion_spot_utils::getCurrentRoadType(
      lanelet_map_ptr->laneletLayer.get(path.points[i].lane_ids[0]), lanelet_map_ptr);
    if (found_target && search_road_type != target_road_type) {
      break;
    }
    // ignore path farther than max range
    if (dist_sum > max_range) {
      break;
    }
    if (!found_target && search_road_type == target_road_type) {
      start_dist = dist_sum;
      found_target = true;
    }
    const auto & curr_p = path.points[i].point.pose.position;
    const auto & next_p = path.points[i + 1].point.pose.position;
    dist_sum += autoware_utils::calcDistance2d(curr_p, next_p);
  }
  return std::pair<int, int>(start_dist, dist_sum);
}

[[maybe_unused]] lanelet::ConstLanelet toPathLanelet(const PathWithLaneId & path)
{
  lanelet::Points3d path_points;
  for (const auto & point_with_id : path.points) {
    const auto & p = point_with_id.point.pose.position;
    path_points.emplace_back(lanelet::InvalId, p.x, p.y, p.z);
  }
  lanelet::LineString3d centerline(lanelet::InvalId, path_points);
  lanelet::Lanelet path_lanelet(lanelet::InvalId);
  path_lanelet.setCenterline(centerline);
  return lanelet::ConstLanelet(path_lanelet);
}
double offsetFromStartToEgo(
  const PathWithLaneId & path, const geometry_msgs::msg::Pose & ego_pose, const int closest_idx)
{
  double offset_from_ego_to_closest = 0;
  for (int i = 0; i < closest_idx; i++) {
    const auto & curr_p = path.points[i].point.pose.position;
    const auto & next_p = path.points[i + 1].point.pose.position;
    offset_from_ego_to_closest += autoware_utils::calcDistance2d(curr_p, next_p);
  }
  const double offset_from_closest_to_target =
    -planning_utils::transformRelCoordinate2D(ego_pose, path.points[closest_idx].point.pose)
       .position.x;
  return offset_from_ego_to_closest + offset_from_closest_to_target;
}

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

  std::vector<PossibleCollisionInfo> possible_collisions;
  double offset_from_start_to_ego = 0;
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
  offset_from_start_to_ego = offsetFromStartToEgo(interp_path, ego_pose, closest_idx);
  const auto path_lanelet = toPathLanelet(interp_path);
  //! Note : Arc Lane from idx[0] to end therefore DO NOT consider offset here
  possible_collisions = occlusion_spot_utils::generatePossibleCollisionBehindParkedVehicle(
    path_lanelet, param_, offset_from_start_to_ego, obj);
  //! Note : consider offset_from_start_to_ego here
  for (auto & pc : possible_collisions) {
    pc.arc_lane_dist_at_collision.length -= offset_from_start_to_ego;
    pc.arc_lane_dist_at_collision.length -= param_.safety_margin;
  }
  if (param_.consider_road_type) {
    std::pair<double, double> focus_length = extractTargetRoadArcLength(
      lanelet_map_ptr, param_.detection_area_length, *path, occlusion_spot_utils::PUBLIC);
    int idx = 0;
    for (const auto pc : possible_collisions) {
      const auto pc_len = pc.arc_lane_dist_at_collision.length;
      if (focus_length.first < pc_len && pc_len < focus_length.second) {
        continue;
      }
      // -----erase-----|start------target-------end|----erase---
      possible_collisions.erase(possible_collisions.begin() + idx);
    }
    idx++;
  }
  //! Arc Lane from idx[0] to end therefore DO NOT consider offset here
  occlusion_spot_utils::calcSlowDownPointsForPossibleCollision(
    0, interp_path, -offset_from_start_to_ego - param_.safety_margin, possible_collisions);

  // apply safe velocity using ebs and pbs deceleration
  applySafeVelocityConsideringPossibleCollison(
    path, possible_collisions, ego_velocity, param_.public_road, param_);

  debug_data_.possible_collisions = possible_collisions;
  return true;
}

}  // namespace behavior_velocity_planner
