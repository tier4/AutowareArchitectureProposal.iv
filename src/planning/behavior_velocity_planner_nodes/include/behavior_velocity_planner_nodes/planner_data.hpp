// Copyright 2021 The Autoware Foundation
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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the behavior_velocity_planner_node class.

#ifndef BEHAVIOR_VELOCITY_PLANNER_NODES__PLANNER_DATA_HPP_
#define BEHAVIOR_VELOCITY_PLANNER_NODES__PLANNER_DATA_HPP_

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <boost/optional.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_constants_manager/vehicle_constants_manager.hpp>

#include <autoware_auto_msgs/msg/order_movement.hpp>
#include <autoware_auto_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <deque>
#include <map>
#include <memory>
#include <vector>

namespace autoware
{
namespace planning
{
namespace behavior_velocity_planner_nodes
{
class BehaviorVelocityPlannerNode;
struct PlannerData
{
  using VehicleConstants = common::vehicle_constants_manager::VehicleConstants;

  explicit PlannerData(rclcpp::Node & node)
  : vehicle_constants_(common::vehicle_constants_manager::declare_and_get_vehicle_constants(node))
  {
    max_stop_acceleration_threshold = node.declare_parameter(
      "max_accel", -5.0);  // TODO(someone): read min_acc in velocity_controller.param.yaml?
    max_stop_jerk_threshold = node.declare_parameter("max_jerk", -5.0);
    delay_response_time = node.declare_parameter("delay_response_time", 0.50);
  }

  // parameters
  VehicleConstants vehicle_constants_;

  // tf
  geometry_msgs::msg::PoseStamped current_pose;

  // msgs from callbacks that are used for data-ready
  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_velocity;
  double current_accel;
  static constexpr double velocity_buffer_time_sec = 10.0;
  std::deque<geometry_msgs::msg::TwistStamped> velocity_buffer;
  autoware_auto_msgs::msg::PredictedObjects::ConstSharedPtr dynamic_objects;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr no_ground_pointcloud;
  lanelet::LaneletMapPtr lanelet_map;

  // other internal data
  //  std::map<int, autoware_perception_msgs::msg::TrafficLightStateStamped> traffic_light_id_map;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules;
  lanelet::routing::RoutingGraphPtr routing_graph;
  std::shared_ptr<const lanelet::routing::RoutingGraphContainer> overall_graphs;

  // external data
  //  std::map<int, autoware_perception_msgs::msg::TrafficLightStateStamped>
  //    external_traffic_light_id_map;
  boost::optional<autoware_auto_msgs::msg::OrderMovement> external_crosswalk_status_input;
  boost::optional<autoware_auto_msgs::msg::OrderMovement> external_intersection_status_input;


  // additional parameters
  double max_stop_acceleration_threshold;
  double max_stop_jerk_threshold;
  double delay_response_time;

  bool isVehicleStopped(const double stop_duration = 0.0) const;

  //  std::shared_ptr<autoware_perception_msgs::msg::TrafficLightStateStamped> getTrafficLightState(
  //    const int id) const
  //  {
  //    if (traffic_light_id_map.count(id) == 0) { return {}; }
  //    return std::make_shared<autoware_perception_msgs::msg::TrafficLightStateStamped>(
  //      traffic_light_id_map.at(id));
  //  }
  //
  //  std::shared_ptr<autoware_perception_msgs::msg::TrafficLightStateStamped>
  //  getExternalTrafficLightState(const int id) const
  //  {
  //    if (external_traffic_light_id_map.count(id) == 0) { return {}; }
  //    return std::make_shared<autoware_perception_msgs::msg::TrafficLightStateStamped>(
  //      external_traffic_light_id_map.at(id));
  //  }

private:
  double prev_accel_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr prev_velocity_;
  double accel_lowpass_gain_;

  void updateCurrentAcc();

  friend BehaviorVelocityPlannerNode;
};

}  // namespace behavior_velocity_planner_nodes
}  // namespace planning
}  // namespace autoware

#endif  // BEHAVIOR_VELOCITY_PLANNER_NODES__PLANNER_DATA_HPP_
