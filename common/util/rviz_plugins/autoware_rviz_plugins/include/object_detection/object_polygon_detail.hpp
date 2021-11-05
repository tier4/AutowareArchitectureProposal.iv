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
// limitations under the License.
/// \brief This file defines some helper functions used by ObjectPolygonDisplayBase class
#ifndef OBJECT_DETECTION__OBJECT_POLYGON_DETAIL_HPP_
#define OBJECT_DETECTION__OBJECT_POLYGON_DETAIL_HPP_

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/shape.hpp>
#include <rclcpp/logging.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <visibility_control.hpp>

#include <map>
#include <string>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
namespace detail
{

// Struct to define all the configurable visual properties of an object of a particular
// classification type
struct ObjectPropertyValues
{
  // Classified type of the object
  std::string label;
  // Color for the type of the object
  std::array<int, 3> color;
  // Alpha values for the type of the object
  float alpha{0.7F};
};

// Map defining colors according to value of label field in ObjectClassification msg
const std::map<autoware_auto_perception_msgs::msg::ObjectClassification::_classification_type,
  ObjectPropertyValues>
// Color map is based on cityscapes color
kDefaultObjectPropertyValues = {
  {autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN, {"UNKNOWN", {255, 255, 255}}},
  {autoware_auto_perception_msgs::msg::ObjectClassification::CAR, {"CAR", {0, 0, 142}}},
  {autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN, {"PEDESTRIAN", {220, 20, 60}}},
  {autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE, {"CYCLIST", {119, 11, 32}}},
  {autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE, {"MOTORCYCLE", {0, 0, 230}}},
  {autoware_auto_perception_msgs::msg::ObjectClassification::TRAILER, {"TRAILER", {0, 80, 100}}},
  {autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK, {"TRUCK", {0, 0, 70}}}
};

/// \brief Convert the given polygon into a marker representing the shape in 2d
/// \param shape_msg Shape msg to be converted. Corners should be in object-local frame
/// \param centroid Centroid position of the shape in Object.header.frame_id frame
/// \param orientation Orientation of the shape in Object.header.frame_id frame
/// \param color_rgba Color and alpha values to use for the marker
/// \return Marker ptr. Id and header will have to be set by the caller
AUTOWARE_RVIZ_PLUGINS_PUBLIC visualization_msgs::msg::Marker::SharedPtr get_2d_polygon_marker_ptr(
  const autoware_auto_perception_msgs::msg::Shape & shape_msg,
  const geometry_msgs::msg::Point & centroid,
  const geometry_msgs::msg::Quaternion & orientation,
  const std_msgs::msg::ColorRGBA & color_rgba);

/// \brief Convert the given polygon into a marker representing the shape in 3d
/// \param shape_msg Shape msg to be converted. Corners should be in object-local frame
/// \param centroid Centroid position of the shape in Object.header.frame_id frame
/// \param orientation Orientation of the shape in Object.header.frame_id frame
/// \param color_rgba Color and alpha values to use for the marker
/// \return Marker ptr. Id and header will have to be set by the caller
AUTOWARE_RVIZ_PLUGINS_PUBLIC visualization_msgs::msg::Marker::SharedPtr get_3d_polygon_marker_ptr(
  const autoware_auto_perception_msgs::msg::Shape & shape_msg,
  const geometry_msgs::msg::Point & centroid,
  const geometry_msgs::msg::Quaternion & orientation,
  const std_msgs::msg::ColorRGBA & color_rgba);

/// \brief Convert Point32 to Point
/// \param val Point32 to be converted
/// \return Point type
inline AUTOWARE_RVIZ_PLUGINS_PUBLIC geometry_msgs::msg::Point to_point(
  const geometry_msgs::msg::Point32 & val)
{
  geometry_msgs::msg::Point ret;
  ret.x = static_cast<double>(val.x);
  ret.y = static_cast<double>(val.y);
  ret.z = static_cast<double>(val.z);
  return ret;
}

/// \brief Get the best classification from the list of classifications based on max probability
/// \tparam ClassificationContainerT List type with ObjectClassificationMsg
/// \param labels List of ObjectClassificationMsg objects
/// \param logger_name Name to use for logger in case of a warning (if labels is empty)
/// \return Id of the best classification, Unknown if there is no best label
template<typename ClassificationContainerT>
AUTOWARE_RVIZ_PLUGINS_PUBLIC autoware_auto_perception_msgs::msg::ObjectClassification::_classification_type
get_best_label(
  ClassificationContainerT labels, const std::string & logger_name)
{
  const auto best_class_label = std::max_element(
    labels.begin(), labels.end(),
    [](const auto & a, const auto & b) -> bool {
      return a.probability < b.probability;
    }
  );
  if (best_class_label == labels.end()) {
    RCLCPP_WARN(
      rclcpp::get_logger(logger_name), "Empty classification field. "
      "Treating as unknown");
    return autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
  }
  return best_class_label->classification;
}

}  // namespace detail
}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

#endif   // OBJECT_DETECTION__OBJECT_POLYGON_DETAIL_HPP_
