// Copyright 2019 Autoware Foundation
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

#ifndef BEHAVIOR_VELOCITY_PLANNER__PLANNER_MANAGER_HPP_
#define BEHAVIOR_VELOCITY_PLANNER__PLANNER_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <scene_module/scene_module_interface.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
class BehaviorVelocityPlannerManager
{
public:
  void launchSceneModule(
    const std::shared_ptr<SceneModuleManagerInterface> & scene_module_manager_ptr);

  autoware_auto_planning_msgs::msg::PathWithLaneId planPathVelocity(
    const std::shared_ptr<const PlannerData> & planner_data,
    const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path_msg);

  diagnostic_msgs::msg::DiagnosticStatus getStopReasonDiag();

private:
  std::vector<std::shared_ptr<SceneModuleManagerInterface>> scene_manager_ptrs_;
  diagnostic_msgs::msg::DiagnosticStatus stop_reason_diag_;
};
}  // namespace behavior_velocity_planner

#endif  // BEHAVIOR_VELOCITY_PLANNER__PLANNER_MANAGER_HPP_
