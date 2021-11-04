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

#include "obstacle_stop_planner/debug_marker.hpp"

#include <autoware_utils/autoware_utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory>
#include <vector>

using autoware_utils::appendMarkerArray;
using autoware_utils::calcOffsetPose;
using autoware_utils::createDefaultMarker;
using autoware_utils::createMarkerColor;
using autoware_utils::createMarkerOrientation;
using autoware_utils::createMarkerScale;
using autoware_utils::createPoint;
using autoware_utils::createSlowDownVirtualWallMarker;
using autoware_utils::createStopVirtualWallMarker;

namespace motion_planning
{
ObstacleStopPlannerDebugNode::ObstacleStopPlannerDebugNode(
  rclcpp::Node * node, const double base_link2front)
: node_(node), base_link2front_(base_link2front)
{
  debug_viz_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/marker", 1);
  stop_reason_pub_ = node_->create_publisher<autoware_planning_msgs::msg::StopReasonArray>(
    "~/output/stop_reasons", 1);
  pub_debug_values_ = node_->create_publisher<Float32MultiArrayStamped>("~/debug/debug_values", 1);
}

bool ObstacleStopPlannerDebugNode::pushPolygon(
  const std::vector<cv::Point2d> & polygon, const double z, const PolygonType & type)
{
  std::vector<Eigen::Vector3d> eigen_polygon;
  for (const auto & point : polygon) {
    Eigen::Vector3d eigen_point;
    eigen_point << point.x, point.y, z;
    eigen_polygon.push_back(eigen_point);
  }
  return pushPolygon(eigen_polygon, type);
}

bool ObstacleStopPlannerDebugNode::pushPolygon(
  const std::vector<Eigen::Vector3d> & polygon, const PolygonType & type)
{
  switch (type) {
    case PolygonType::Vehicle:
      if (!polygon.empty()) {
        vehicle_polygons_.push_back(polygon);
      }
      return true;
    case PolygonType::Collision:
      if (!polygon.empty()) {
        collision_polygons_.push_back(polygon);
      }
      return true;
    case PolygonType::SlowDownRange:
      if (!polygon.empty()) {
        slow_down_range_polygons_.push_back(polygon);
      }
      return true;
    case PolygonType::SlowDown:
      if (!polygon.empty()) {
        slow_down_polygons_.push_back(polygon);
      }
      return true;
    default:
      return false;
  }
}

bool ObstacleStopPlannerDebugNode::pushPose(
  const geometry_msgs::msg::Pose & pose, const PoseType & type)
{
  switch (type) {
    case PoseType::Stop:
      stop_pose_ptr_ = std::make_shared<geometry_msgs::msg::Pose>(pose);
      return true;
    case PoseType::SlowDownStart:
      slow_down_start_pose_ptr_ = std::make_shared<geometry_msgs::msg::Pose>(pose);
      return true;
    case PoseType::SlowDownEnd:
      slow_down_end_pose_ptr_ = std::make_shared<geometry_msgs::msg::Pose>(pose);
      return true;
    default:
      return false;
  }
}

bool ObstacleStopPlannerDebugNode::pushObstaclePoint(
  const geometry_msgs::msg::Point & obstacle_point, const PointType & type)
{
  switch (type) {
    case PointType::Stop:
      stop_obstacle_point_ptr_ = std::make_shared<geometry_msgs::msg::Point>(obstacle_point);
      return true;
    case PointType::SlowDown:
      slow_down_obstacle_point_ptr_ = std::make_shared<geometry_msgs::msg::Point>(obstacle_point);
      return true;
    default:
      return false;
  }
}

bool ObstacleStopPlannerDebugNode::pushObstaclePoint(
  const pcl::PointXYZ & obstacle_point, const PointType & type)
{
  geometry_msgs::msg::Point ros_point;
  ros_point.x = obstacle_point.x;
  ros_point.y = obstacle_point.y;
  ros_point.z = obstacle_point.z;
  return pushObstaclePoint(ros_point, type);
}

void ObstacleStopPlannerDebugNode::publish()
{
  /* publish debug marker for rviz */
  const auto visualization_msg = makeVisualizationMarker();
  debug_viz_pub_->publish(visualization_msg);

  /* publish stop reason for autoware api */
  const auto stop_reason_msg = makeStopReasonArray();
  stop_reason_pub_->publish(stop_reason_msg);

  // publish debug values
  autoware_debug_msgs::msg::Float32MultiArrayStamped debug_msg{};
  debug_msg.stamp = node_->now();
  for (const auto & v : debug_values_.getValues()) {
    debug_msg.data.push_back(v);
  }
  pub_debug_values_->publish(debug_msg);

  /* reset variables */
  vehicle_polygons_.clear();
  collision_polygons_.clear();
  slow_down_range_polygons_.clear();
  slow_down_polygons_.clear();
  stop_pose_ptr_ = nullptr;
  slow_down_start_pose_ptr_ = nullptr;
  slow_down_end_pose_ptr_ = nullptr;
  stop_obstacle_point_ptr_ = nullptr;
  slow_down_obstacle_point_ptr_ = nullptr;
}

visualization_msgs::msg::MarkerArray ObstacleStopPlannerDebugNode::makeVisualizationMarker()
{
  visualization_msgs::msg::MarkerArray msg;
  rclcpp::Time current_time = node_->now();

  // polygon
  if (!vehicle_polygons_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "detection_polygons", 0, visualization_msgs::msg::Marker::LINE_LIST,
      createMarkerScale(0.01, 0.0, 0.0), createMarkerColor(0.0, 1.0, 0.0, 0.999));

    for (size_t i = 0; i < vehicle_polygons_.size(); ++i) {
      for (size_t j = 0; j < vehicle_polygons_.at(i).size(); ++j) {
        {
          const auto & p = vehicle_polygons_.at(i).at(j);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
        if (j + 1 == vehicle_polygons_.at(i).size()) {
          const auto & p = vehicle_polygons_.at(i).at(0);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        } else {
          const auto & p = vehicle_polygons_.at(i).at(j + 1);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
      }
    }
    msg.markers.push_back(marker);
  }

  if (!collision_polygons_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "collision_polygons", 0, visualization_msgs::msg::Marker::LINE_LIST,
      createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(1.0, 0.0, 0.0, 0.999));

    for (size_t i = 0; i < collision_polygons_.size(); ++i) {
      for (size_t j = 0; j < collision_polygons_.at(i).size(); ++j) {
        {
          const auto & p = collision_polygons_.at(i).at(j);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
        if (j + 1 == collision_polygons_.at(i).size()) {
          const auto & p = collision_polygons_.at(i).at(0);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        } else {
          const auto & p = collision_polygons_.at(i).at(j + 1);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
      }
    }
    msg.markers.push_back(marker);
  }

  if (!slow_down_range_polygons_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "slow_down_detection_polygons", 0,
      visualization_msgs::msg::Marker::LINE_LIST, createMarkerScale(0.01, 0.0, 0.0),
      createMarkerColor(0.0, 1.0, 0.0, 0.999));

    for (size_t i = 0; i < slow_down_range_polygons_.size(); ++i) {
      for (size_t j = 0; j < slow_down_range_polygons_.at(i).size(); ++j) {
        {
          const auto & p = slow_down_range_polygons_.at(i).at(j);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
        if (j + 1 == slow_down_range_polygons_.at(i).size()) {
          const auto & p = slow_down_range_polygons_.at(i).at(0);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        } else {
          const auto & p = slow_down_range_polygons_.at(i).at(j + 1);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
      }
    }
    msg.markers.push_back(marker);
  }

  if (!slow_down_polygons_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "slow_down_polygons", 0, visualization_msgs::msg::Marker::LINE_LIST,
      createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(1.0, 1.0, 0.0, 0.999));

    for (size_t i = 0; i < slow_down_polygons_.size(); ++i) {
      for (size_t j = 0; j < slow_down_polygons_.at(i).size(); ++j) {
        {
          const auto & p = slow_down_polygons_.at(i).at(j);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
        if (j + 1 == slow_down_polygons_.at(i).size()) {
          const auto & p = slow_down_polygons_.at(i).at(0);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        } else {
          const auto & p = slow_down_polygons_.at(i).at(j + 1);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
      }
    }
    msg.markers.push_back(marker);
  }

  if (stop_pose_ptr_ != nullptr) {
    const auto p = calcOffsetPose(*stop_pose_ptr_, base_link2front_, 0.0, 0.0);
    const auto markers = createStopVirtualWallMarker(p, "obstacle on the path", current_time, 0);
    appendMarkerArray(markers, &msg);
  }

  if (slow_down_start_pose_ptr_ != nullptr && stop_pose_ptr_ == nullptr) {
    const auto p = calcOffsetPose(*slow_down_start_pose_ptr_, base_link2front_, 0.0, 0.0);

    {
      const auto markers =
        createSlowDownVirtualWallMarker(p, "obstacle beside the path", current_time, 0);
      appendMarkerArray(markers, &msg);
    }

    {
      auto markers = createSlowDownVirtualWallMarker(p, "slow down\nstart", current_time, 1);
      markers.markers.front().ns = "slow_down_start_virtual_wall";
      markers.markers.back().ns = "slow_down_start_factor_text";
      appendMarkerArray(markers, &msg);
    }
  }

  if (slow_down_end_pose_ptr_ != nullptr && stop_pose_ptr_ == nullptr) {
    const auto p = calcOffsetPose(*slow_down_end_pose_ptr_, base_link2front_, 0.0, 0.0);
    auto markers = createSlowDownVirtualWallMarker(p, "slow down\nend", current_time, 2);
    markers.markers.front().ns = "slow_down_end_virtual_wall";
    markers.markers.back().ns = "slow_down_end_factor_text";
    appendMarkerArray(markers, &msg);
  }

  if (stop_obstacle_point_ptr_ != nullptr) {
    auto marker = createDefaultMarker(
      "map", current_time, "stop_obstacle_point", 0, visualization_msgs::msg::Marker::SPHERE,
      createMarkerScale(0.25, 0.25, 0.25), createMarkerColor(1.0, 0.0, 0.0, 0.999));
    marker.pose.position = *stop_obstacle_point_ptr_;
    msg.markers.push_back(marker);
  }

  if (stop_obstacle_point_ptr_ != nullptr) {
    auto marker = createDefaultMarker(
      "map", current_time, "stop_obstacle_text", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, createMarkerScale(0.0, 0.0, 1.0),
      createMarkerColor(1.0, 1.0, 1.0, 0.999));
    marker.pose.position = *stop_obstacle_point_ptr_;
    marker.pose.position.z += 2.0;
    marker.text = "!";
    msg.markers.push_back(marker);
  }

  if (slow_down_obstacle_point_ptr_ != nullptr) {
    auto marker = createDefaultMarker(
      "map", current_time, "slow_down_obstacle_point", 0, visualization_msgs::msg::Marker::SPHERE,
      createMarkerScale(0.25, 0.25, 0.25), createMarkerColor(1.0, 0.0, 0.0, 0.999));
    marker.pose.position = *slow_down_obstacle_point_ptr_;
    msg.markers.push_back(marker);
  }

  if (slow_down_obstacle_point_ptr_ != nullptr) {
    auto marker = createDefaultMarker(
      "map", current_time, "slow_down_obstacle_text", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, createMarkerScale(0.0, 0.0, 1.0),
      createMarkerColor(1.0, 1.0, 1.0, 0.999));
    marker.pose.position = *slow_down_obstacle_point_ptr_;
    marker.pose.position.z += 2.0;
    marker.text = "!";
    msg.markers.push_back(marker);
  }

  return msg;
}

autoware_planning_msgs::msg::StopReasonArray ObstacleStopPlannerDebugNode::makeStopReasonArray()
{
  // create header
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = node_->now();

  // create stop reason stamped
  autoware_planning_msgs::msg::StopReason stop_reason_msg;
  stop_reason_msg.reason = autoware_planning_msgs::msg::StopReason::OBSTACLE_STOP;
  autoware_planning_msgs::msg::StopFactor stop_factor;

  if (stop_pose_ptr_ != nullptr) {
    stop_factor.stop_pose = *stop_pose_ptr_;
    if (stop_obstacle_point_ptr_ != nullptr) {
      stop_factor.stop_factor_points.emplace_back(*stop_obstacle_point_ptr_);
    }
    stop_reason_msg.stop_factors.emplace_back(stop_factor);
  }

  // create stop reason array
  autoware_planning_msgs::msg::StopReasonArray stop_reason_array;
  stop_reason_array.header = header;
  stop_reason_array.stop_reasons.emplace_back(stop_reason_msg);
  return stop_reason_array;
}

}  // namespace motion_planning
