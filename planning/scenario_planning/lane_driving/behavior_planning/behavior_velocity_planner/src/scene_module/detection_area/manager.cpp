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
#include <scene_module/detection_area/manager.h>

namespace
{
std::vector<lanelet::DetectionAreaConstPtr> getDetectionAreaRegElemsOnPath(
  const autoware_planning_msgs::PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map)
{
  std::vector<lanelet::DetectionAreaConstPtr> detection_area_reg_elems;

  for (const auto & p : path.points) {
    const auto lane_id = p.lane_ids.at(0);
    const auto ll = lanelet_map->laneletLayer.get(lane_id);
    const auto detection_areas = ll.regulatoryElementsAs<const lanelet::autoware::DetectionArea>();
    for (const auto & detection_area : detection_areas) {
      detection_area_reg_elems.push_back(detection_area);
    }
  }

  return detection_area_reg_elems;
}

std::set<int64_t> getDetectionAreaIdSetOnPath(
  const autoware_planning_msgs::PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map)
{
  std::set<int64_t> detection_area_id_set;
  for (const auto & detection_area : getDetectionAreaRegElemsOnPath(path, lanelet_map)) {
    detection_area_id_set.insert(detection_area->id());
  }
  return detection_area_id_set;
}
}  // namespace

DetectionAreaModuleManager::DetectionAreaModuleManager()
: SceneModuleManagerInterface(getModuleName())
{
  ros::NodeHandle pnh("~");
  const std::string ns(getModuleName());
  auto & p = planner_param_;
  pnh.param(ns + "/stop_margin", p.stop_margin, 0.0);
  pnh.param(ns + "/use_dead_line", p.use_dead_line, false);
  pnh.param(ns + "/dead_line_margin", p.dead_line_margin, 5.0);
  pnh.param(ns + "/use_pass_judge_line", p.use_pass_judge_line, false);
  pnh.param(ns + "/state_clear_time", p.state_clear_time, 2.0);
}

void DetectionAreaModuleManager::launchNewModules(
  const autoware_planning_msgs::PathWithLaneId & path)
{
  for (const auto & detection_area :
       getDetectionAreaRegElemsOnPath(path, planner_data_->lanelet_map)) {
    // Use lanelet_id to unregister module when the route is changed
    const auto module_id = detection_area->id();
    if (!isModuleRegistered(module_id)) {
      registerModule(
        std::make_shared<DetectionAreaModule>(module_id, *detection_area, planner_param_));
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
DetectionAreaModuleManager::getModuleExpiredFunction(
  const autoware_planning_msgs::PathWithLaneId & path)
{
  const auto detection_area_id_set = getDetectionAreaIdSetOnPath(path, planner_data_->lanelet_map);

  return [detection_area_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return detection_area_id_set.count(scene_module->getModuleId()) == 0;
  };
}
