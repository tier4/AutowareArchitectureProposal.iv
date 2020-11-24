/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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
#include <scene_module/intersection/manager.h>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <utilization/boost_geometry_helper.h>
#include <utilization/util.h>

namespace
{
std::vector<lanelet::ConstLanelet> getLaneletsOnPath(
  const autoware_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map)
{
  std::vector<lanelet::ConstLanelet> lanelets;

  for (const auto & p : path.points) {
    const auto lane_id = p.lane_ids.at(0);
    const auto lane = lanelet_map->laneletLayer.get(lane_id);
    if (!lanelet::utils::contains(lanelets, lane)) {
      lanelets.push_back(lane);
    }
  }

  return lanelets;
}

std::set<int64_t> getLaneIdSetOnPath(const autoware_planning_msgs::msg::PathWithLaneId & path)
{
  std::set<int64_t> lane_id_set;

  for (const auto & p : path.points) {
    for (const auto & lane_id : p.lane_ids) {
      lane_id_set.insert(lane_id);
    }
  }

  return lane_id_set;
}

}  // namespace

IntersectionModuleManager::IntersectionModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(getModuleName());
  auto & p = planner_param_;
  vehicle_info_util::VehicleInfo vehicle_info = vehicle_info_util::VehicleInfo::create(node);
  p.state_transit_mergin_time = node.declare_parameter(ns + "/state_transit_mergin_time", 2.0);
  p.decel_velocoity = node.declare_parameter(ns + "/decel_velocoity", 30.0 / 3.6);
  p.path_expand_width = node.declare_parameter(ns + "/path_expand_width", 2.0);
  p.stop_line_margin = node.declare_parameter(ns + "/stop_line_margin", 1.0);
  p.stuck_vehicle_detect_dist = node.declare_parameter(ns + "/stuck_vehicle_detect_dist", 5.0);
  p.stuck_vehicle_ignore_dist = node.declare_parameter(ns + "/stuck_vehicle_ignore_dist", 5.0) +
                                vehicle_info.max_longitudinal_offset_m_;
  p.stuck_vehicle_vel_thr = node.declare_parameter(ns + "/stuck_vehicle_vel_thr", 3.0 / 3.6);
  p.intersection_velocity = node.declare_parameter(ns + "/intersection_velocity", 10.0 / 3.6);
  p.detection_area_length = node.declare_parameter(ns + "/detection_area_length", 200.0);
}

void IntersectionModuleManager::launchNewModules(
  const autoware_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lanelets = getLaneletsOnPath(path, planner_data_->lanelet_map);
  for (size_t i = 0; i < lanelets.size(); i++) {
    const auto ll = lanelets.at(i);
    const auto lane_id = ll.id();
    const auto module_id = lane_id;

    if (isModuleRegistered(module_id)) {
      continue;
    }

    // Is intersection?
    const std::string turn_direction = ll.attributeOr("turn_direction", "else");
    const auto is_intersection =
      turn_direction == "right" || turn_direction == "left" || turn_direction == "straight";
    if (!is_intersection) {
      continue;
    }

    // Is merging from private road?
    if (i + 1 < lanelets.size()) {
      const auto next_lane = lanelets.at(i + 1);
      const std::string lane_location = ll.attributeOr("location", "else");
      const std::string next_lane_location = next_lane.attributeOr("location", "else");
      if (lane_location == "private" && next_lane_location != "private") {
        registerModule(std::make_shared<MergeFromPrivateRoadModule>(
          module_id, lane_id, planner_data_, planner_param_, logger_.get_child("merge_from_private_road_module"), clock_));
      }
    }

    registerModule(std::make_shared<IntersectionModule>(
      module_id, lane_id, planner_data_, planner_param_, logger_.get_child("intersection_module"), clock_));
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
IntersectionModuleManager::getModuleExpiredFunction(
  const autoware_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lane_id_set = getLaneIdSetOnPath(path);

  return [lane_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return lane_id_set.count(scene_module->getModuleId()) == 0;
  };
}
