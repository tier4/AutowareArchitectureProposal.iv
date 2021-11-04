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

#ifndef SCENE_MODULE__OCCLUSION_SPOT__MANAGER_HPP_
#define SCENE_MODULE__OCCLUSION_SPOT__MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>
#include <scene_module/occlusion_spot/scene_occlusion_spot_in_private_road.hpp>
#include <scene_module/occlusion_spot/scene_occlusion_spot_in_public_road.hpp>
#include <scene_module/scene_module_interface.hpp>

#include <autoware_perception_msgs/msg/dynamic_object.hpp>
#include <autoware_perception_msgs/msg/dynamic_object_array.hpp>
#include <autoware_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
class OcclusionSpotModuleManager : public SceneModuleManagerInterface
{
public:
  explicit OcclusionSpotModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "occlusion_spot"; }

private:
  enum class ModuleID { PRIVATE, PUBLIC };
  using PlannerParam = occlusion_spot_utils::PlannerParam;

  PlannerParam planner_param_;

  void launchNewModules(const autoware_planning_msgs::msg::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const autoware_planning_msgs::msg::PathWithLaneId & path) override;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_debug_occupancy_grid_;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OCCLUSION_SPOT__MANAGER_HPP_
