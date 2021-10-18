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
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include "lanelet2_extension/utility/message_conversion.hpp"
#include <lanelet2_routing/RoutingGraphContainer.h>

#include "utilization/path_utilization.hpp"

#include <autoware_auto_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_msgs/msg/order_movement.hpp>
#include <autoware_auto_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <functional>
#include <memory>

// Scene modules
#include "scene_module/blind_spot/manager.hpp"
#include "scene_module/crosswalk/manager.hpp"
#include "scene_module/detection_area/manager.hpp"
#include "scene_module/intersection/manager.hpp"
#include "scene_module/stop_line/manager.hpp"

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
  // Parameters
  forward_path_length_ = this->declare_parameter("forward_path_length", 1000.0);
  backward_path_length_ = this->declare_parameter("backward_path_length", 5.0);
  // TODO(yukkysaito): This will become unnecessary when acc output from localization is available.
  planner_data_.accel_lowpass_gain_ = this->declare_parameter("lowpass_gain", 0.5);

  // Initialize PlannerManager
  if (this->declare_parameter("launch_stop_line", true)) {
    planner_manager_.launchSceneModule(std::make_shared<StopLineModuleManager>(*this));
  }
  if (this->declare_parameter("launch_crosswalk", true)) {
    planner_manager_.launchSceneModule(std::make_shared<CrosswalkModuleManager>(*this));
  }
  if (this->declare_parameter("launch_intersection", true)) {
    planner_manager_.launchSceneModule(std::make_shared<IntersectionModuleManager>(*this));
  }
  if (this->declare_parameter("launch_blind_spot", true)) {
    planner_manager_.launchSceneModule(std::make_shared<BlindSpotModuleManager>(*this));
  }
  if (this->declare_parameter("launch_detection_area", true)) {
    planner_manager_.launchSceneModule(std::make_shared<DetectionAreaModuleManager>(*this));
  }
}

void BehaviorVelocityPlannerNode::callback_predicted_objects(
  const autoware_auto_msgs::msg::PredictedObjects::ConstSharedPtr msg_in)
{
  planner_data_.dynamic_objects = msg_in;
}

void BehaviorVelocityPlannerNode::callback_cloud_no_ground(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_in)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_ptr_->lookupTransform(
      "map", msg_in->header.frame_id, msg_in->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(get_logger(), "no transform found for no_ground_pointcloud: %s", e.what());
    return;
  }

  using CloudView = point_cloud_msg_wrapper::PointCloud2View<common::types::PointXYZI>;
  CloudView cloud_view(*msg_in);

  using CloudModifier = point_cloud_msg_wrapper::PointCloud2Modifier<common::types::PointXYZI>;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_ptr_trans =
    std::make_shared<sensor_msgs::msg::PointCloud2>();

  Eigen::Affine3f affine = tf2::transformToEigen(transform.transform).cast<float_t>();
  CloudModifier cloud_modifier_trans(*cloud_ptr_trans, "map");
  cloud_modifier_trans.resize(static_cast<uint32_t>(cloud_view.size()));
  std::transform(
    cloud_view.cbegin(),
    cloud_view.cend(),
    cloud_modifier_trans.begin(),
    [&affine](const common::types::PointXYZI & point) {
      Eigen::Vector4f vec_point(point.x, point.y, point.z, 1.0f);
      Eigen::Vector4f vec_point_trans(affine.matrix() * vec_point);
      return common::types::PointXYZI{
        vec_point_trans.x(),
        vec_point_trans.y(),
        vec_point_trans.z(),
        point.intensity};
    });

  planner_data_.no_ground_pointcloud = cloud_ptr_trans;
}

void BehaviorVelocityPlannerNode::callback_twist_vehicle(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg_in)
{
  planner_data_.current_velocity = msg_in;
  planner_data_.updateCurrentAcc();

  // Add velocity to buffer
  planner_data_.velocity_buffer.push_front(*msg_in);
  const auto now = this->now();
  while (true) {
    // Check oldest data time
    const auto time_diff = now - planner_data_.velocity_buffer.back().header.stamp;

    // Finish when oldest data is newer than threshold
    if (time_diff.seconds() <= PlannerData::velocity_buffer_time_sec) {
      break;
    }

    // Remove old data
    planner_data_.velocity_buffer.pop_back();
  }
}

void BehaviorVelocityPlannerNode::callback_had_map_bin_lanelet(
  const autoware_auto_msgs::msg::HADMapBin::ConstSharedPtr msg_in)
{
  // Load map
  planner_data_.lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg_in, planner_data_.lanelet_map, &planner_data_.traffic_rules, &planner_data_.routing_graph);

  // Build graph
  {
    using lanelet::Locations;
    using lanelet::Participants;
    using lanelet::routing::RoutingGraph;
    using lanelet::routing::RoutingGraphConstPtr;
    using lanelet::routing::RoutingGraphContainer;
    using lanelet::traffic_rules::TrafficRulesFactory;

    const auto traffic_rules =
      TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
    const auto pedestrian_rules =
      TrafficRulesFactory::create(Locations::Germany, Participants::Pedestrian);

    RoutingGraphConstPtr vehicle_graph =
      RoutingGraph::build(*planner_data_.lanelet_map, *traffic_rules);
    RoutingGraphConstPtr pedestrian_graph =
      RoutingGraph::build(*planner_data_.lanelet_map, *pedestrian_rules);
    RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});

    planner_data_.overall_graphs = std::make_shared<const RoutingGraphContainer>(overall_graphs);
  }
}

void BehaviorVelocityPlannerNode::callback_path_with_lane_id(
  const autoware_auto_msgs::msg::PathWithLaneId::ConstSharedPtr msg_in)
{
  auto transform2pose =
    [](const geometry_msgs::msg::TransformStamped & transform) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      return pose;
    };

  // Check ready
  try {
    planner_data_.current_pose =
      transform2pose(tf_buffer_ptr_->lookupTransform("map", "base_link", tf2::TimePointZero));
  } catch (tf2::TransformException & e) {
    RCLCPP_INFO(get_logger(), "waiting for transform from `map` to `base_link`");
    return;
  }

  if (!is_data_ready()) {
    return;
  }

  // Plan path velocity
  const auto velocity_planned_path = planner_manager_.planPathVelocity(
    std::make_shared<const PlannerData>(planner_data_), *msg_in);

  auto to_path = [](const autoware_auto_msgs::msg::PathWithLaneId & path_with_id) {
      autoware_auto_msgs::msg::Path path;
      for (const auto & path_point : path_with_id.points) {
        path.points.push_back(path_point.point);
      }
      return path;
    };

  // screening
  const auto filtered_path = filterLitterPathPoint(to_path(velocity_planned_path));

  // interpolation
  const auto interpolated_path_msg =
    interpolatePath(filtered_path, forward_path_length_, this->get_logger());

  // check stop point
  auto output_path_msg = filterStopPathPoint(interpolated_path_msg);
  output_path_msg.header.frame_id = "map";
  output_path_msg.header.stamp = this->now();

  // TODO(someone): This must be updated in each scene module, but copy from input message for now.
  output_path_msg.drivable_area = msg_in->drivable_area;

  pub_path_->publish(output_path_msg);
  pub_diagnostic_status_->publish(planner_manager_.getStopReasonDiag());

  if (pub_markers_debug_->get_subscription_count() > 0) {
    publish_debug_marker(output_path_msg);
  }
}

void BehaviorVelocityPlannerNode::callback_order_movement_crosswalk(
  const autoware_auto_msgs::msg::OrderMovement::ConstSharedPtr msg_in)
{
  planner_data_.external_crosswalk_status_input = *msg_in;
}

void BehaviorVelocityPlannerNode::callback_order_movement_intersection(
  const autoware_auto_msgs::msg::OrderMovement::ConstSharedPtr msg_in)
{
  planner_data_.external_intersection_status_input = *msg_in;
}

bool BehaviorVelocityPlannerNode::is_data_ready()
{
  const auto & d = planner_data_;

  // from tf
  if (d.current_pose.header.frame_id == "") {
    return false;
  }

  // from callbacks
  if (!d.current_velocity) {
    return false;
  }
  if (!d.dynamic_objects) {
    return false;
  }
  if (!d.no_ground_pointcloud) {
    return false;
  }
  if (!d.lanelet_map) {
    return false;
  }

  return true;
}

void BehaviorVelocityPlannerNode::publish_debug_marker(const autoware_auto_msgs::msg::Path & path)
{
  visualization_msgs::msg::MarkerArray output_msg;
  for (size_t i = 0; i < path.points.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.id = static_cast<int32_t>(i);
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.pose = path.points.at(i).pose;
    marker.scale.y = marker.scale.z = 0.05;
    marker.scale.x = 0.25;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.color.a = 0.999f;  // Don't forget to set the alpha!
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    output_msg.markers.push_back(marker);
  }
  pub_markers_debug_->publish(output_msg);
}

}  // namespace behavior_velocity_planner_nodes
}  // namespace planning
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::planning::behavior_velocity_planner_nodes::BehaviorVelocityPlannerNode)
