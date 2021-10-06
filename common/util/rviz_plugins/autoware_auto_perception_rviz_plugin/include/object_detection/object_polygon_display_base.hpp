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
#ifndef OBJECT_DETECTION__OBJECT_POLYGON_DISPLAY_BASE_HPP_
#define OBJECT_DETECTION__OBJECT_POLYGON_DISPLAY_BASE_HPP_

#include <autoware_auto_msgs/msg/object_classification.hpp>
#include <common/color_alpha_property.hpp>
#include <object_detection/object_polygon_detail.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <rviz_default_plugins/displays/marker_array/marker_array_display.hpp>
#include <visibility_control.hpp>

#include <list>
#include <memory>
#include <string>
#include <unordered_map>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
/// \brief Base rviz plugin class for all object msg types. The class defines common properties
///        for the plugin and also defines common helper functions that can be used by its derived
///        classes.
/// \tparam MsgT TrackedObjects or DetectedObjects type
template<typename MsgT>
class AUTOWARE_RVIZ_PLUGINS_PUBLIC ObjectPolygonDisplayBase
  : public rviz_common::RosTopicDisplay<MsgT>
{
public:
  using Color = std::array<float, 3U>;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerCommon = rviz_default_plugins::displays::MarkerCommon;
  using ObjectClassificationMsg = autoware_auto_msgs::msg::ObjectClassification;
  using RosTopicDisplay = rviz_common::RosTopicDisplay<MsgT>;

  using PolygonPropertyMap = std::unordered_map<ObjectClassificationMsg::_classification_type,
      common::ColorAlphaProperty>;

  explicit ObjectPolygonDisplayBase(const std::string & default_topic)
  : m_marker_common(this),
    m_display_3d_property{
      "Display 3d polygon",
      true,
      "Enable/disable height visualization of the polygon", this
    },
    m_default_topic{default_topic}
  {
    // iterate over default values to create and initialize the properties.
    for (const auto & map_property_it : detail::kDefaultObjectPropertyValues) {
      const auto & class_property_values = map_property_it.second;
      const auto & color = class_property_values.color;
      // This is just a parent property to contain the necessary properties for the given class:
      m_class_group_properties.emplace_back(
        class_property_values.label.c_str(), QVariant(),
        "Groups polygon properties for the given class", this);
      auto & parent_property = m_class_group_properties.back();
      // Associate a color and opacity property for the given class and attach them to the
      // parent property of the class so they can have a drop down view from the label property:
      m_polygon_properties.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(map_property_it.first),
        std::forward_as_tuple(
          QColor{color[0], color[1], color[2]}, class_property_values.alpha,
          &parent_property));
    }
  }

  void onInitialize() override
  {
    RosTopicDisplay::RTDClass::onInitialize();
    m_marker_common.initialize(this->context_, this->scene_node_);
    this->topic_property_->setValue(m_default_topic.c_str());
    this->topic_property_->setDescription("Topic to subscribe to.");
  }

  void load(const rviz_common::Config & config) override
  {
    RosTopicDisplay::Display::load(config);
    m_marker_common.load(config);
  }

  void update(float wall_dt, float ros_dt) override
  {
    m_marker_common.update(wall_dt, ros_dt);
  }

  void reset() override
  {
    RosTopicDisplay::reset();
    m_marker_common.clearMarkers();
  }

  void clear_markers()
  {
    m_marker_common.clearMarkers();
  }

  void add_marker(visualization_msgs::msg::Marker::ConstSharedPtr marker_ptr)
  {
    m_marker_common.addMessage(marker_ptr);
  }

protected:
  /// \brief Convert given shape msg into a Marker
  /// \tparam ClassificationContainerT List type with ObjectClassificationMsg
  /// \param shape_msg Shape msg to be converted
  /// \param centroid Centroid position of the shape in Object.header.frame_id frame
  /// \param orientation Orientation of the shape in Object.header.frame_id frame
  /// \param labels List of ObjectClassificationMsg objects
  /// \return Marker ptr. Id and header will have to be set by the caller
  template<typename ClassificationContainerT>
  visualization_msgs::msg::Marker::SharedPtr get_marker_ptr(
    const autoware_auto_msgs::msg::Shape & shape_msg,
    const geometry_msgs::msg::Point & centroid,
    const geometry_msgs::msg::Quaternion & orientation,
    const ClassificationContainerT & labels)
  const
  {
    const std_msgs::msg::ColorRGBA color_rgba = get_color_rgba(labels);

    if (m_display_3d_property.getBool()) {
      return detail::get_3d_polygon_marker_ptr(shape_msg, centroid, orientation, color_rgba);
    } else {
      return detail::get_2d_polygon_marker_ptr(shape_msg, centroid, orientation, color_rgba);
    }
  }

  /// \brief Get color and alpha values based on the given list of classification values
  /// \tparam ClassificationContainerT Container of ObjectClassification
  /// \param labels list of classifications
  /// \return Color and alpha for the best class in the given list. Unknown class is used in
  ///         degenerate cases
  template<typename ClassificationContainerT>
  std_msgs::msg::ColorRGBA get_color_rgba(const ClassificationContainerT & labels) const
  {
    static const std::string kLoggerName("ObjectPolygonDisplayBase");
    const auto label = detail::get_best_label(labels, kLoggerName);
    auto it = m_polygon_properties.find(label);

    if (it == m_polygon_properties.end()) {
      RCLCPP_WARN(
        rclcpp::get_logger(kLoggerName), "Color alpha property does not exist for "
        "label ", std::to_string(label), "Using property values from UNKNOWN");
      it = m_polygon_properties.find(ObjectClassificationMsg::UNKNOWN);
    }
    return it->second;
  }

private:
  // All rviz plugins should have this. Should be initialized with pointer to this class
  MarkerCommon m_marker_common;
  // List is used to store the properties for classification in case we need to access them:
  std::list<rviz_common::properties::Property> m_class_group_properties;
  // Map to store class labels and its corresponding properties
  PolygonPropertyMap m_polygon_properties;
  // Property to enable/disable height visualization of the polygon
  rviz_common::properties::BoolProperty m_display_3d_property;
  // Default topic name to be visualized
  std::string m_default_topic;
};
}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

#endif   // OBJECT_DETECTION__OBJECT_POLYGON_DISPLAY_BASE_HPP_
