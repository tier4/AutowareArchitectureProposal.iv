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

#include "behavior_velocity_planner_nodes/behavior_velocity_planner_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_auto_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_msgs/msg/order_movement.hpp>
#include <autoware_auto_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <functional>
#include <memory>

namespace autoware
{
namespace planning
{
namespace behavior_velocity_planner_nodes
{
const std::uint32_t QOS_HISTORY_DEPTH = 1;

BehaviorVelocityPlannerNode::BehaviorVelocityPlannerNode(const rclcpp::NodeOptions & options)
: Node("behavior_velocity_planner_node", options),
  tf_buffer_ptr_{std::make_shared<tf2_ros::Buffer>(this->get_clock())},
  tf_listener_ptr_{std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_)},
  pub_path_{this->create_publisher<autoware_auto_msgs::msg::Path>(
      "~/output/path", rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)))},
  pub_diagnostic_status_{this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
      "~/output/stop_reason", rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)))},
  pub_markers_debug_{this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/debug/path", rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)))},
  sub_predicted_objects_{this->create_subscription<autoware_auto_msgs::msg::PredictedObjects>(
      "~/input/dynamic_objects",
      rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)),
      std::bind(
        &BehaviorVelocityPlannerNode::callback_predicted_objects, this, std::placeholders::_1))},
  sub_cloud_no_ground_{this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "~/input/no_ground_pointcloud",
      rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)),
      std::bind(
        &BehaviorVelocityPlannerNode::callback_cloud_no_ground, this, std::placeholders::_1))},
  sub_twist_vehicle_{this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/input/vehicle_velocity",
      rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)),
      std::bind(&BehaviorVelocityPlannerNode::callback_twist_vehicle, this,
      std::placeholders::_1))},
  sub_had_map_bin_lanelet_{this->create_subscription<autoware_auto_msgs::msg::HADMapBin>(
      "~/input/vector_map",
      rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)),
      std::bind(
        &BehaviorVelocityPlannerNode::callback_had_map_bin_lanelet, this, std::placeholders::_1))},
  sub_path_with_lane_id_{this->create_subscription<autoware_auto_msgs::msg::PathWithLaneId>(
      "~/input/path_with_lane_id",
      rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)),
      std::bind(
        &BehaviorVelocityPlannerNode::callback_path_with_lane_id, this, std::placeholders::_1))},
  sub_order_movement_crosswalk_{this->create_subscription<autoware_auto_msgs::msg::OrderMovement>(
      "~/input/external_crosswalk_states",
      rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)),
      std::bind(
        &BehaviorVelocityPlannerNode::callback_order_movement_crosswalk,
        this,
        std::placeholders::_1))},
  sub_order_movement_intersection_{
    this->create_subscription<autoware_auto_msgs::msg::OrderMovement>(
      "~/input/external_intersection_states",
      rclcpp::QoS(rclcpp::KeepLast(QOS_HISTORY_DEPTH)),
      std::bind(
        &BehaviorVelocityPlannerNode::callback_order_movement_intersection,
        this,
        std::placeholders::_1))},
  planner_data_(*this)
{
}

void BehaviorVelocityPlannerNode::callback_predicted_objects(
  const autoware_auto_msgs::msg::PredictedObjects::ConstSharedPtr msg_in)
{
  (void)msg_in;
}

void BehaviorVelocityPlannerNode::callback_cloud_no_ground(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_in)
{
  (void)msg_in;
}

void BehaviorVelocityPlannerNode::callback_twist_vehicle(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg_in)
{
  (void)msg_in;
}

void BehaviorVelocityPlannerNode::callback_had_map_bin_lanelet(
  const autoware_auto_msgs::msg::HADMapBin::ConstSharedPtr msg_in)
{
  (void)msg_in;
}

void BehaviorVelocityPlannerNode::callback_path_with_lane_id(
  const autoware_auto_msgs::msg::PathWithLaneId::ConstSharedPtr msg_in)
{
  (void)msg_in;
}

void BehaviorVelocityPlannerNode::callback_order_movement_crosswalk(
  const autoware_auto_msgs::msg::OrderMovement::ConstSharedPtr msg_in)
{
  (void)msg_in;
}

void BehaviorVelocityPlannerNode::callback_order_movement_intersection(
  const autoware_auto_msgs::msg::OrderMovement::ConstSharedPtr msg_in)
{
  (void)msg_in;
}

}  // namespace behavior_velocity_planner_nodes
}  // namespace planning
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::planning::behavior_velocity_planner_nodes::BehaviorVelocityPlannerNode)
