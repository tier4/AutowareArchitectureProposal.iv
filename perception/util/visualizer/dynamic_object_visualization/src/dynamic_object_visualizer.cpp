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

#include <memory>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"
#include "Eigen/Eigen"

#include "dynamic_object_visualization/dynamic_object_visualizer.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::placeholders;

DynamicObjectVisualizer::DynamicObjectVisualizer(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("dynamic_object_visualizer", node_options)
{
  with_feature_ = declare_parameter("with_feature", true);
  only_known_objects_ = declare_parameter("only_known_objects", true);
  if (with_feature_) {
    sub_with_feature_ =
      create_subscription<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>(
      "input", rclcpp::QoS{1},
      std::bind(&DynamicObjectVisualizer::dynamicObjectWithFeatureCallback, this, _1));
  } else {
    sub_ = create_subscription<autoware_perception_msgs::msg::DynamicObjectArray>(
      "input", rclcpp::QoS{1},
      std::bind(&DynamicObjectVisualizer::dynamicObjectCallback, this, _1));
  }
  pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "output",
    rclcpp::QoS{1}.transient_local());
  initColorList(colors_);
}

void DynamicObjectVisualizer::dynamicObjectWithFeatureCallback(
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr input_msg)
{
  if (this->count_subscribers(pub_->get_topic_name()) < 1) {return;}
  auto converted_objects_ptr =
    std::make_shared<autoware_perception_msgs::msg::DynamicObjectArray>();
  converted_objects_ptr->header = input_msg->header;
  for (const auto & feature_object : input_msg->feature_objects) {
    converted_objects_ptr->objects.push_back(feature_object.object);
  }
  dynamicObjectCallback(converted_objects_ptr);
}

void DynamicObjectVisualizer::dynamicObjectCallback(
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg)
{
  if (this->count_subscribers(pub_->get_topic_name()) < 1) {return;}
  visualization_msgs::msg::MarkerArray output;
  constexpr double line_width = 0.03;
  // shape
  for (size_t i = 0; i < input_msg->objects.size(); ++i) {
    if (only_known_objects_) {
      if (input_msg->objects.at(i).semantic.type ==
        autoware_perception_msgs::msg::Semantic::UNKNOWN)
      {
        continue;
      }
    }
    visualization_msgs::msg::Marker marker;
    marker.header = input_msg->header;
    marker.id = i;
    marker.ns = std::string("shape");

    using Shape = autoware_perception_msgs::msg::Shape;
    if (input_msg->objects.at(i).shape.type == Shape::BOUNDING_BOX) {
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      if (!calcBoundingBoxLineList(input_msg->objects.at(i).shape, marker.points)) {continue;}
    } else if (input_msg->objects.at(i).shape.type == Shape::CYLINDER) {
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      if (!calcCylinderLineList(input_msg->objects.at(i).shape, marker.points)) {continue;}
    } else if (input_msg->objects.at(i).shape.type == Shape::POLYGON) {
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      if (!calcPolygonLineList(input_msg->objects.at(i).shape, marker.points)) {continue;}
    } else {
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      if (!calcPolygonLineList(input_msg->objects.at(i).shape, marker.points)) {continue;}
    }

    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.pose = input_msg->objects.at(i).state.pose_covariance.pose;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.scale.x = line_width;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    output.markers.push_back(marker);
  }

  // orientation
  for (size_t i = 0; i < input_msg->objects.size(); ++i) {
    if (!input_msg->objects.at(i).state.orientation_reliable) {
      continue;
    }
    if (only_known_objects_) {
      if (input_msg->objects.at(i).semantic.type ==
        autoware_perception_msgs::msg::Semantic::UNKNOWN)
      {
        continue;
      }
    }
    visualization_msgs::msg::Marker marker;
    marker.header = input_msg->header;
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.ns = std::string("orientation");
    marker.scale.x = line_width;
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.pose = input_msg->objects.at(i).state.pose_covariance.pose;
    geometry_msgs::msg::Point point;
    point.x = 0.0;
    point.y = 0;
    point.z = (input_msg->objects.at(i).shape.dimensions.z / 2.0);
    marker.points.push_back(point);
    point.x = (input_msg->objects.at(i).shape.dimensions.x / 2.0);
    point.y = 0;
    point.z = (input_msg->objects.at(i).shape.dimensions.z / 2.0);
    marker.points.push_back(point);
    point.x = 0.0;
    point.y = 0;
    point.z = -(input_msg->objects.at(i).shape.dimensions.z / 2.0);
    marker.points.push_back(point);
    point.x = (input_msg->objects.at(i).shape.dimensions.x / 2.0);
    point.y = 0;
    point.z = -(input_msg->objects.at(i).shape.dimensions.z / 2.0);
    marker.points.push_back(point);

    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    output.markers.push_back(marker);
  }

  // type label
  for (size_t i = 0; i < input_msg->objects.size(); ++i) {
    if (only_known_objects_) {
      if (input_msg->objects.at(i).semantic.type ==
        autoware_perception_msgs::msg::Semantic::UNKNOWN)
      {
        continue;
      }
    }
    visualization_msgs::msg::Marker marker;
    marker.header = input_msg->header;
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.ns = std::string("label");
    std::string label;
    if (!getLabel(input_msg->objects.at(i).semantic, label)) {continue;}
    marker.scale.x = 0.5;
    marker.scale.z = 0.5;

    std::string id_str = uuid_to_string(input_msg->objects.at(i).id);

    marker.text = label + ":" + id_str.substr(0, 4);
    if (input_msg->objects.at(i).state.twist_reliable) {
      double vel = std::sqrt(
        input_msg->objects.at(i).state.twist_covariance.twist.linear.x *
        input_msg->objects.at(i).state.twist_covariance.twist.linear.x +
        input_msg->objects.at(i).state.twist_covariance.twist.linear.y *
        input_msg->objects.at(i).state.twist_covariance.twist.linear.y +
        input_msg->objects.at(i).state.twist_covariance.twist.linear.z *
        input_msg->objects.at(i).state.twist_covariance.twist.linear.z);
      marker.text = marker.text + "\n" + std::to_string(static_cast<int>(vel * 3.6)) + std::string(
        "[km/h]");
    }
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.pose = input_msg->objects.at(i).state.pose_covariance.pose;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    output.markers.push_back(marker);
  }

  // position covariance
  for (size_t i = 0; i < input_msg->objects.size(); ++i) {
    if (
      input_msg->objects.at(i).state.pose_covariance.covariance[6 * 0 + 0] /*x-x*/ == 0.0 ||
      input_msg->objects.at(i).state.pose_covariance.covariance[6 * 1 + 1] /*y-y*/ == 0.0)
    {
      continue;
    }

    if (only_known_objects_) {
      if (input_msg->objects.at(i).semantic.type ==
        autoware_perception_msgs::msg::Semantic::UNKNOWN)
      {
        continue;
      }
    }
    visualization_msgs::msg::Marker marker;
    marker.header = input_msg->header;
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.ns = std::string("position covariance");
    marker.scale.x = line_width;
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.pose = input_msg->objects.at(i).state.pose_covariance.pose;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    geometry_msgs::msg::Point point;
    Eigen::Matrix2d pose_covariance;
    pose_covariance << input_msg->objects.at(i).state.pose_covariance.covariance[0],
      input_msg->objects.at(i).state.pose_covariance.covariance[1],
      input_msg->objects.at(i).state.pose_covariance.covariance[6],
      input_msg->objects.at(i).state.pose_covariance.covariance[7];
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(pose_covariance);
    double sigma1 = 2.448 * std::sqrt(solver.eigenvalues().x());  // 2.448 sigma is 95%
    double sigma2 = 2.448 * std::sqrt(solver.eigenvalues().y());  // 2.448 sigma is 95%
    Eigen::Vector2d e1 = solver.eigenvectors().col(0);
    Eigen::Vector2d e2 = solver.eigenvectors().col(1);
    point.x = -e1.x() * sigma1;
    point.y = -e1.y() * sigma1;
    point.z = 0;
    marker.points.push_back(point);
    point.x = e1.x() * sigma1;
    point.y = e1.y() * sigma1;
    point.z = 0;
    marker.points.push_back(point);
    point.x = -e2.x() * sigma2;
    point.y = -e2.y() * sigma2;
    point.z = 0;
    marker.points.push_back(point);
    point.x = e2.x() * sigma2;
    point.y = e2.y() * sigma2;
    point.z = 0;
    marker.points.push_back(point);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    output.markers.push_back(marker);
  }

  // twist
  for (size_t i = 0; i < input_msg->objects.size(); ++i) {
    if (!input_msg->objects.at(i).state.twist_reliable) {
      continue;
    }
    if (only_known_objects_) {
      if (input_msg->objects.at(i).semantic.type ==
        autoware_perception_msgs::msg::Semantic::UNKNOWN)
      {
        continue;
      }
    }
    visualization_msgs::msg::Marker marker;
    marker.header = input_msg->header;
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.ns = std::string("twist");
    marker.scale.x = line_width;
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.pose = input_msg->objects.at(i).state.pose_covariance.pose;
    geometry_msgs::msg::Point point;
    point.x = 0.0;
    point.y = 0;
    point.z = 0;
    marker.points.push_back(point);
    point.x = input_msg->objects.at(i).state.twist_covariance.twist.linear.x;
    point.y = input_msg->objects.at(i).state.twist_covariance.twist.linear.y;
    point.z = input_msg->objects.at(i).state.twist_covariance.twist.linear.z;
    marker.points.push_back(point);

    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    output.markers.push_back(marker);
  }

  // path
  {
    int id = 0;
    for (size_t i = 0; i < input_msg->objects.size(); ++i) {
      if (only_known_objects_) {
        if (input_msg->objects.at(i).semantic.type ==
          autoware_perception_msgs::msg::Semantic::UNKNOWN)
        {
          continue;
        }
      }
      visualization_msgs::msg::Marker marker;
      marker.header = input_msg->header;
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.ns = std::string("path");
      marker.action = visualization_msgs::msg::Marker::MODIFY;
      marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      initPose(marker.pose);
      getColor(input_msg->objects.at(i), marker.color);
      for (size_t j = 0; j < input_msg->objects.at(i).state.predicted_paths.size(); ++j) {
        marker.color.a = std::max(
          static_cast<double>(std::min(
            static_cast<double>(input_msg->objects.at(i).state.predicted_paths.at(j).confidence),
            0.999)),
          0.5);
        marker.scale.x = line_width * marker.color.a;
        marker.points.clear();
        if (!calcPathLineList(
            input_msg->objects.at(i).state.predicted_paths.at(j),
            marker.points))
        {
          continue;
        }
        for (size_t k = 0; k < marker.points.size(); ++k) {
          marker.points.at(k).z -= input_msg->objects.at(i).shape.dimensions.z / 2.0;
        }
        marker.id = ++id;
        output.markers.push_back(marker);
      }
    }
  }

  // path confidence label
  {
    int id = 0;
    for (size_t i = 0; i < input_msg->objects.size(); ++i) {
      if (only_known_objects_) {
        if (input_msg->objects.at(i).semantic.type ==
          autoware_perception_msgs::msg::Semantic::UNKNOWN)
        {
          continue;
        }
      }
      visualization_msgs::msg::Marker marker;
      marker.header = input_msg->header;
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.ns = std::string("path confidence");
      marker.action = visualization_msgs::msg::Marker::MODIFY;
      marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;
      initPose(marker.pose);
      getColor(input_msg->objects.at(i), marker.color);
      for (size_t j = 0; j < input_msg->objects.at(i).state.predicted_paths.size(); ++j) {
        if (!input_msg->objects.at(i).state.predicted_paths.at(j).path.empty()) {
          int path_final_index =
            static_cast<int>(input_msg->objects.at(i).state.predicted_paths.at(j).path.size()) - 1;
          marker.pose.position = input_msg->objects.at(i)
            .state.predicted_paths.at(j)
            .path.at(path_final_index)
            .pose.pose.position;
          marker.text =
            std::to_string(input_msg->objects.at(i).state.predicted_paths.at(j).confidence);
          marker.color.a = std::max(
            static_cast<double>(std::min(
              static_cast<double>(input_msg->objects.at(i).state.predicted_paths.at(j).confidence),
              1.0)),
            0.5);
          marker.id = ++id;
          output.markers.push_back(marker);
        }
      }
    }
  }
  pub_->publish(output);
}

bool DynamicObjectVisualizer::calcBoundingBoxLineList(
  const autoware_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  geometry_msgs::msg::Point point;
  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  // up surface
  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = shape.dimensions.z / 2.0;
  points.push_back(point);

  // down surface
  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = -shape.dimensions.x / 2.0;
  point.y = shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  point.x = shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);
  point.x = -shape.dimensions.x / 2.0;
  point.y = -shape.dimensions.y / 2.0;
  point.z = -shape.dimensions.z / 2.0;
  points.push_back(point);

  return true;
}

bool DynamicObjectVisualizer::calcCylinderLineList(
  const autoware_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const double radius = shape.dimensions.x * 0.5;
  {
    constexpr int n = 20;
    geometry_msgs::msg::Point center;
    center.x = 0.0;
    center.y = 0.0;
    center.z = shape.dimensions.z * 0.5;
    calcCircleLineList(center, radius, points, n);
    center.z = -shape.dimensions.z * 0.5;
    calcCircleLineList(center, radius, points, n);
  }
  {
    constexpr int n = 4;
    for (int i = 0; i < n; ++i) {
      geometry_msgs::msg::Point point;
      point.x = std::cos(
        (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI + M_PI /
        static_cast<double>(n)) * radius;
      point.y = std::sin(
        (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI + M_PI /
        static_cast<double>(n)) * radius;
      point.z = shape.dimensions.z * 0.5;
      points.push_back(point);
      point.x = std::cos(
        (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI + M_PI /
        static_cast<double>(n)) * radius;
      point.y = std::sin(
        (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI + M_PI /
        static_cast<double>(n)) * radius;
      point.z = -shape.dimensions.z * 0.5;
      points.push_back(point);
    }
  }
  return true;
}

bool DynamicObjectVisualizer::calcCircleLineList(
  const geometry_msgs::msg::Point center, const double radius,
  std::vector<geometry_msgs::msg::Point> & points, const int n)
{
  for (int i = 0; i < n; ++i) {
    geometry_msgs::msg::Point point;
    point.x = std::cos(
      (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI + M_PI /
      static_cast<double>(n)) * radius + center.x;
    point.y = std::sin(
      (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI + M_PI /
      static_cast<double>(n)) * radius + center.y;
    point.z = center.z;
    points.push_back(point);
    point.x =
      std::cos(
      (static_cast<double>(i + 1.0) / static_cast<double>(n)) * 2.0 * M_PI + M_PI /
      static_cast<double>(n)) * radius + center.x;
    point.y =
      std::sin(
      (static_cast<double>(i + 1.0) / static_cast<double>(n)) * 2.0 * M_PI + M_PI /
      static_cast<double>(n)) * radius + center.y;
    point.z = center.z;
    points.push_back(point);
  }
  return true;
}

bool DynamicObjectVisualizer::calcPolygonLineList(
  const autoware_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  if (shape.footprint.points.size() < 2) {return false;}
  for (size_t i = 0; i < shape.footprint.points.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = shape.footprint.points.at(i).x;
    point.y = shape.footprint.points.at(i).y;
    point.z = shape.dimensions.z / 2.0;
    points.push_back(point);
    point.x =
      shape.footprint.points.at(
      static_cast<int>(i + 1) %
      static_cast<int>(shape.footprint.points.size())).x;
    point.y =
      shape.footprint.points.at(
      static_cast<int>(i + 1) %
      static_cast<int>(shape.footprint.points.size())).y;
    point.z = shape.dimensions.z / 2.0;
    points.push_back(point);
  }
  for (size_t i = 0; i < shape.footprint.points.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = shape.footprint.points.at(i).x;
    point.y = shape.footprint.points.at(i).y;
    point.z = -shape.dimensions.z / 2.0;
    points.push_back(point);
    point.x =
      shape.footprint.points.at(
      static_cast<int>(i + 1) %
      static_cast<int>(shape.footprint.points.size())).x;
    point.y =
      shape.footprint.points.at(
      static_cast<int>(i + 1) %
      static_cast<int>(shape.footprint.points.size())).y;
    point.z = -shape.dimensions.z / 2.0;
    points.push_back(point);
  }
  for (size_t i = 0; i < shape.footprint.points.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = shape.footprint.points.at(i).x;
    point.y = shape.footprint.points.at(i).y;
    point.z = shape.dimensions.z / 2.0;
    points.push_back(point);
    point.x = shape.footprint.points.at(i).x;
    point.y = shape.footprint.points.at(i).y;
    point.z = -shape.dimensions.z / 2.0;
    points.push_back(point);
  }
  return true;
}

bool DynamicObjectVisualizer::calcPathLineList(
  const autoware_perception_msgs::msg::PredictedPath & paths,
  std::vector<geometry_msgs::msg::Point> & points)
{
  for (int i = 0; i < static_cast<int>(paths.path.size()) - 1; ++i) {
    geometry_msgs::msg::Point point;
    point.x = paths.path.at(i).pose.pose.position.x;
    point.y = paths.path.at(i).pose.pose.position.y;
    point.z = paths.path.at(i).pose.pose.position.z;
    points.push_back(point);
    point.x = paths.path.at(i + 1).pose.pose.position.x;
    point.y = paths.path.at(i + 1).pose.pose.position.y;
    point.z = paths.path.at(i + 1).pose.pose.position.z;
    points.push_back(point);
    calcCircleLineList(point, 0.25, points, 10);
  }
  return true;
}

void DynamicObjectVisualizer::initPose(geometry_msgs::msg::Pose & pose)
{
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
}

bool DynamicObjectVisualizer::getLabel(
  const autoware_perception_msgs::msg::Semantic & semantic, std::string & label)
{
  if (autoware_perception_msgs::msg::Semantic::UNKNOWN == semantic.type) {
    label = std::string("unknown");
  } else if (autoware_perception_msgs::msg::Semantic::CAR == semantic.type) {
    label = std::string("car");
  } else if (autoware_perception_msgs::msg::Semantic::TRUCK == semantic.type) {
    label = std::string("truck");
  } else if (autoware_perception_msgs::msg::Semantic::BUS == semantic.type) {
    label = std::string("bus");
  } else if (autoware_perception_msgs::msg::Semantic::BICYCLE == semantic.type) {
    label = std::string("bicycle");
  } else if (autoware_perception_msgs::msg::Semantic::MOTORBIKE == semantic.type) {
    label = std::string("motorbike");
  } else if (autoware_perception_msgs::msg::Semantic::PEDESTRIAN == semantic.type) {
    label = std::string("pedestrian");
  } else if (autoware_perception_msgs::msg::Semantic::ANIMAL == semantic.type) {
    label = std::string("animal");
  } else {
    label = std::string("other");
  }
  return true;
}

void DynamicObjectVisualizer::getColor(
  const autoware_perception_msgs::msg::DynamicObject & object, std_msgs::msg::ColorRGBA & color)
{
  std::string id_str = uuid_to_string(object.id);

  int i = (static_cast<int>(id_str.at(0)) * 4 + static_cast<int>(id_str.at(1))) %
    static_cast<int>(colors_.size());
  color.r = colors_.at(i).r;
  color.g = colors_.at(i).g;
  color.b = colors_.at(i).b;
}

void DynamicObjectVisualizer::initColorList(std::vector<std_msgs::msg::ColorRGBA> & colors)
{
  std_msgs::msg::ColorRGBA sample_color;
  sample_color.r = 1.0;
  sample_color.g = 0.0;
  sample_color.b = 1.0;
  colors.push_back(sample_color);  // magenta
  sample_color.r = 0.69;
  sample_color.g = 1.0;
  sample_color.b = 0.18;
  colors.push_back(sample_color);  // green yellow
  sample_color.r = 0.59;
  sample_color.g = 1.0;
  sample_color.b = 0.59;
  colors.push_back(sample_color);  // pale green
  sample_color.r = 0.5;
  sample_color.g = 1.0;
  sample_color.b = 0.0;
  colors.push_back(sample_color);  // chartreuse green
  sample_color.r = 0.12;
  sample_color.g = 0.56;
  sample_color.b = 1.0;
  colors.push_back(sample_color);  // dodger blue
  sample_color.r = 0.0;
  sample_color.g = 1.0;
  sample_color.b = 1.0;
  colors.push_back(sample_color);  // cyan
  sample_color.r = 0.54;
  sample_color.g = 0.168;
  sample_color.b = 0.886;
  colors.push_back(sample_color);  // blueviolet
  sample_color.r = 0.0;
  sample_color.g = 1.0;
  sample_color.b = 0.5;
  colors.push_back(sample_color);  // spring green
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(DynamicObjectVisualizer)
