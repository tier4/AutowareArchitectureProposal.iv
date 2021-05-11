// Copyright 2019 Autoware Foundation. All rights reserved.
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

#ifndef LANE_CHANGE_PLANNER__LANE_CHANGER_HPP_
#define LANE_CHANGE_PLANNER__LANE_CHANGER_HPP_

#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"
#include "tf2_ros/transform_listener.h"
#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_traffic_rules/TrafficRulesFactory.h"
#include "autoware_lanelet2_msgs/msg/map_bin.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_planning_msgs/msg/lane_change_command.hpp"
#include "autoware_planning_msgs/msg/lane_change_status.hpp"
#include "autoware_planning_msgs/msg/path.hpp"
#include "autoware_planning_msgs/msg/path_with_lane_id.hpp"
#include "autoware_planning_msgs/msg/route.hpp"
#include "autoware_planning_msgs/msg/stop_reason_array.hpp"
#include "lane_change_planner/data_manager.hpp"
#include "lane_change_planner/route_handler.hpp"
#include "lane_change_planner/state_machine.hpp"

namespace lane_change_planner
{
class LaneChanger : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<autoware_planning_msgs::msg::PathWithLaneId>::SharedPtr path_publisher_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Path>::SharedPtr candidate_path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_publisher_;
  rclcpp::Publisher<autoware_planning_msgs::msg::StopReasonArray>::SharedPtr stop_reason_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr drivable_area_publisher_;
  rclcpp::Publisher<autoware_planning_msgs::msg::LaneChangeStatus>::SharedPtr
    lane_change_ready_publisher_;
  rclcpp::Publisher<autoware_planning_msgs::msg::LaneChangeStatus>::SharedPtr
    lane_change_available_publisher_;

  rclcpp::Subscription<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr
    perception_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscriber_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneChangeCommand>::SharedPtr
    lane_change_approval_subscriber_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneChangeCommand>::SharedPtr
    force_lane_change_subscriber_;
  rclcpp::Subscription<autoware_lanelet2_msgs::msg::MapBin>::SharedPtr vector_map_subscriber_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Route>::SharedPtr route_subscriber_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Route>::SharedPtr route_init_subscriber_;

  std::shared_ptr<DataManager> data_manager_ptr_;
  std::shared_ptr<StateMachine> state_machine_ptr_;
  std::shared_ptr<RouteHandler> route_handler_ptr_;
  // PathExtender path_extender_;

  void run();
  void publishDebugMarkers();
  void publishDrivableArea(const autoware_planning_msgs::msg::PathWithLaneId & path);
  autoware_planning_msgs::msg::StopReasonArray makeStopReasonArray(
    const DebugData & debug_data, const State & state);
  std::vector<autoware_planning_msgs::msg::StopReason> makeEmptyStopReasons();

public:
  explicit LaneChanger(const rclcpp::NodeOptions & node_options);
  void init();
};
}  // namespace lane_change_planner

#endif  // LANE_CHANGE_PLANNER__LANE_CHANGER_HPP_
