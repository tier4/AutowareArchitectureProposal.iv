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

#ifndef SCENE_MODULE__NO_STOPPING_AREA__MANAGER_HPP_
#define SCENE_MODULE__NO_STOPPING_AREA__MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "scene_module/no_stopping_area/scene_no_stopping_area.hpp"
#include "scene_module/scene_module_interface.hpp"

#include "autoware_planning_msgs/msg/path_with_lane_id.hpp"

#include <functional>
#include <memory>

namespace behavior_velocity_planner
{
class NoStoppingAreaModuleManager : public SceneModuleManagerInterface
{
public:
  explicit NoStoppingAreaModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "no_stopping_area"; }

private:
  NoStoppingAreaModule::PlannerParam planner_param_;
  void launchNewModules(const autoware_planning_msgs::msg::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const autoware_planning_msgs::msg::PathWithLaneId & path) override;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__NO_STOPPING_AREA__MANAGER_HPP_
