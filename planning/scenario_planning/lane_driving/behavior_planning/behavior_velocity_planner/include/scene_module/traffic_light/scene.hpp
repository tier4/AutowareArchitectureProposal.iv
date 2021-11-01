// Copyright 2020 Tier IV, Inc.
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

#ifndef SCENE_MODULE__TRAFFIC_LIGHT__SCENE_HPP_
#define SCENE_MODULE__TRAFFIC_LIGHT__SCENE_HPP_

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "autoware_perception_msgs/msg/looking_traffic_light_state.hpp"
#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_routing/RoutingGraph.h"
#include "rclcpp/rclcpp.hpp"
#include "scene_module/scene_module_interface.hpp"
#include "utilization/boost_geometry_helper.hpp"

namespace behavior_velocity_planner
{
class TrafficLightModule : public SceneModuleInterface
{
public:
  enum class State { APPROACH, GO_OUT };
  enum class Input { PERCEPTION, EXTERNAL, NONE };  // EXTERNAL: FOA, V2X, etc.

  struct DebugData
  {
    double base_link2front;
    std::vector<std::tuple<
      std::shared_ptr<const lanelet::TrafficLight>,
      autoware_perception_msgs::msg::TrafficLightState>>
      tl_state;
    std::vector<geometry_msgs::msg::Pose> stop_poses;
    geometry_msgs::msg::Pose first_stop_pose;
    std::vector<geometry_msgs::msg::Pose> dead_line_poses;
    std::vector<geometry_msgs::msg::Point> traffic_light_points;
    geometry_msgs::msg::Point highest_confidence_traffic_light_point;
  };

  struct PlannerParam
  {
    double stop_margin;
    double tl_state_timeout;
    double external_tl_state_timeout;
    double yellow_lamp_period;
    bool enable_pass_judge;
  };

public:
  TrafficLightModule(
    const int64_t module_id, const lanelet::TrafficLight & traffic_light_reg_elem,
    lanelet::ConstLanelet lane, const PlannerParam & planner_param, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(
    autoware_planning_msgs::msg::PathWithLaneId * path,
    autoware_planning_msgs::msg::StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;

  inline autoware_perception_msgs::msg::LookingTrafficLightState getTrafficLightState() const
  {
    return looking_tl_state_;
  }

  inline State getTrafficLightModuleState() const { return state_; }

  inline boost::optional<int> getFirstRefStopPathPointIndex() const
  {
    return first_ref_stop_path_point_index_;
  }

private:
  bool isStopSignal(const lanelet::ConstLineStringsOrPolygons3d & traffic_lights);

  bool isTrafficLightStateStop(
    const autoware_perception_msgs::msg::TrafficLightState & tl_state) const;

  autoware_planning_msgs::msg::PathWithLaneId insertStopPose(
    const autoware_planning_msgs::msg::PathWithLaneId & input,
    const size_t & insert_target_point_idx, const Eigen::Vector2d & target_point,
    autoware_planning_msgs::msg::StopReason * stop_reason);

  bool isPassthrough(const double & signed_arc_length) const;

  bool hasLampState(
    const autoware_perception_msgs::msg::TrafficLightState & tl_state,
    const uint8_t & lamp_color) const;

  bool getHighestConfidenceTrafficLightState(
    const lanelet::ConstLineStringsOrPolygons3d & traffic_lights,
    autoware_perception_msgs::msg::TrafficLightStateStamped & highest_confidence_tl_state);

  bool getExternalTrafficLightState(
    const lanelet::ConstLineStringsOrPolygons3d & traffic_lights,
    autoware_perception_msgs::msg::TrafficLightStateStamped & external_tl_state);

  bool updateTrafficLightState(const lanelet::ConstLineStringsOrPolygons3d & traffic_lights);

  autoware_perception_msgs::msg::TrafficLightStateWithJudge generateTlStateWithJudgeFromTlState(
    const autoware_perception_msgs::msg::TrafficLightState tl_state) const;

  // Key Feature
  const lanelet::TrafficLight & traffic_light_reg_elem_;
  lanelet::ConstLanelet lane_;

  // State
  State state_;

  // Input
  Input input_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;

  // prevent paththrough chattering
  bool is_prev_state_stop_;

  boost::optional<int> first_ref_stop_path_point_index_;

  // Traffic Light State
  autoware_perception_msgs::msg::LookingTrafficLightState looking_tl_state_;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__TRAFFIC_LIGHT__SCENE_HPP_
