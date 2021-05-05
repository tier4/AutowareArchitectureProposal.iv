// Copyright 2021 Apex.AI, Inc.
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
// limitations under the License..

#include <object_detection/object_polygon_detail.hpp>

#include <memory>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
namespace detail
{

using Marker = visualization_msgs::msg::Marker;

visualization_msgs::msg::Marker::SharedPtr get_2d_polygon_marker_ptr(
  const autoware_auto_msgs::msg::Shape & shape_msg,
  const std_msgs::msg::ColorRGBA & color_rgba)
{
  auto marker_ptr = std::make_shared<Marker>();

  if (shape_msg.polygon.points.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("ObjectPolygonDisplayBase"), "Empty polygon!");
    return marker_ptr;
  }

  marker_ptr->scale.x = 0.1;
  marker_ptr->type = Marker::LINE_STRIP;
  marker_ptr->action = Marker::ADD;
  marker_ptr->color = color_rgba;

  geometry_msgs::msg::Point first_pt;
  for (const auto & pt32 : shape_msg.polygon.points) {
    marker_ptr->points.push_back(to_point(pt32));
  }
  // Add the first vertex again to close the shape
  marker_ptr->points.push_back(to_point(shape_msg.polygon.points.front()));

  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_3d_polygon_marker_ptr(
  const autoware_auto_msgs::msg::Shape & shape_msg,
  const std_msgs::msg::ColorRGBA & color_rgba)
{
  auto marker_ptr = std::make_shared<Marker>();

  if (shape_msg.polygon.points.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("ObjectPolygonDisplayBase"), "Empty polygon!");
    return marker_ptr;
  }

  marker_ptr->scale.x = 0.1;
  marker_ptr->type = Marker::LINE_LIST;
  marker_ptr->action = Marker::ADD;
  marker_ptr->color = color_rgba;

  // To construct a 3d polygon using line list, we need to define all the edges with the two
  // end points. We will first draw lower part of the polygon by inserting all the vertices
  // twice (like, 1,2, 2,3, 3,4,). Then we will do the same to draw the upper part of the
  // polygon. Then we will connect the upper and lower part by inserting two points per vertex,
  // one with min_z and one with max_z.

  // Construct lower polygon
  geometry_msgs::msg::Point first_pt;
  first_pt = to_point(shape_msg.polygon.points.front());
  for (auto it = shape_msg.polygon.points.begin(); it != shape_msg.polygon.points.end(); ++it) {
    geometry_msgs::msg::Point pt = to_point(*it);
    marker_ptr->points.push_back(pt);
    if (it != shape_msg.polygon.points.begin()) {
      marker_ptr->points.push_back(pt);
    }
  }
  marker_ptr->points.push_back(first_pt);

  // Construct upper polygon
  first_pt.z += static_cast<double>(shape_msg.height);
  for (auto it = shape_msg.polygon.points.begin(); it != shape_msg.polygon.points.end(); ++it) {
    geometry_msgs::msg::Point pt = to_point(*it);
    pt.z += static_cast<double>(shape_msg.height);
    marker_ptr->points.push_back(pt);
    if (it != shape_msg.polygon.points.begin()) {
      marker_ptr->points.push_back(pt);
    }
  }
  marker_ptr->points.push_back(first_pt);

  // Construct connections between lower and upper polygon
  for (const auto & pt32 : shape_msg.polygon.points) {
    geometry_msgs::msg::Point pt = to_point(pt32);
    marker_ptr->points.push_back(pt);
    pt.z += static_cast<double>(shape_msg.height);
    marker_ptr->points.push_back(pt);
  }

  return marker_ptr;
}

}  // namespace detail
}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware
