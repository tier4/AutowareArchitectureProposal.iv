// Copyright 2019 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <object_detection/bounding_box_array_display.hpp>
#include <common/types.hpp>
#include <memory>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

namespace autoware
{
namespace rviz_plugins
{

BoundingBoxArrayDisplay::BoundingBoxArrayDisplay()
: rviz_common::RosTopicDisplay<autoware_auto_perception_msgs::msg::BoundingBoxArray>(),
  m_marker_common(std::make_unique<MarkerCommon>(this))
{
  no_label_color_property_ = new rviz_common::properties::ColorProperty(
    "No Label Color", QColor(255.0, 255.0, 255.0), "Color to draw unlabelled boundingboxes.",
    this, SLOT(updateProperty()));
  car_color_property_ = new rviz_common::properties::ColorProperty(
    "Car Color", QColor(255.0, 255.0, 0), "Color to draw car boundingboxes.",
    this, SLOT(updateProperty()));
  pedestrian_color_property_ = new rviz_common::properties::ColorProperty(
    "Pedestrian Color", QColor(0, 0, 255.0), "Color to draw pedestrian boundingboxes.",
    this, SLOT(updateProperty()));
  cyclist_color_property_ = new rviz_common::properties::ColorProperty(
    "Cyclist Color", QColor(255.0, 165.0, 0), "Color to draw cyclist boundingboxes.",
    this, SLOT(updateProperty()));
  motorcycle_color_property_ = new rviz_common::properties::ColorProperty(
    "Motorcycle Color", QColor(0, 255.0, 0), "Color to draw motorcycle boundingboxes.",
    this, SLOT(updateProperty()));
  other_color_property_ = new rviz_common::properties::ColorProperty(
    "Other Color", QColor(0, 0, 0), "Color to draw other boundingboxes.",
    this, SLOT(updateProperty()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0.7f, "Amount of transparency to apply to the boundingbox.",
    this, SLOT(updateProperty()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);
}

void BoundingBoxArrayDisplay::onInitialize()
{
  RTDClass::onInitialize();
  m_marker_common->initialize(context_, scene_node_);

  topic_property_->setValue("lidar_bounding_boxes");
  topic_property_->setDescription("BoundingBoxArray topic to subscribe to.");
}

void BoundingBoxArrayDisplay::load(const rviz_common::Config & config)
{
  Display::load(config);
  m_marker_common->load(config);
}

void BoundingBoxArrayDisplay::updateProperty()
{
  if (msg_cache != nullptr) {
    processMessage(msg_cache);
  }
}

void BoundingBoxArrayDisplay::processMessage(
  BoundingBoxArray::ConstSharedPtr msg)
{
  msg_cache = msg;
  m_marker_common->clearMarkers();
  for (auto idx = 0U; idx < msg->boxes.size(); idx++) {
    const auto marker_ptr = get_marker(msg->boxes[idx]);
    marker_ptr->ns = "bounding_box";
    marker_ptr->header = msg->header;
    marker_ptr->id = static_cast<int>(idx);
    m_marker_common->addMessage(marker_ptr);
  }
}

visualization_msgs::msg::Marker::SharedPtr BoundingBoxArrayDisplay::get_marker(
  const BoundingBox & box) const
{
  auto marker = std::make_shared<Marker>();

  marker->type = Marker::CUBE;
  marker->action = Marker::ADD;
  marker->color.a = alpha_property_->getFloat();

  QColor color;
  switch (box.vehicle_label) {
    case BoundingBox::NO_LABEL:     // white: non labeled
      color = no_label_color_property_->getColor();
      break;
    case BoundingBox::CAR:          // yellow: car
      color = car_color_property_->getColor();
      break;
    case BoundingBox::PEDESTRIAN:   // blue: pedestrian
      color = pedestrian_color_property_->getColor();
      break;
    case BoundingBox::CYCLIST:      // orange: cyclist
      color = cyclist_color_property_->getColor();
      break;
    case BoundingBox::MOTORCYCLE:   // green: motorcycle
      color = motorcycle_color_property_->getColor();
      break;
    default:                        // black: other labels
      color = other_color_property_->getColor();
      break;
  }

  marker->color.r = static_cast<float>(color.redF());
  marker->color.g = static_cast<float>(color.greenF());
  marker->color.b = static_cast<float>(color.blueF());
  marker->pose.position.x = static_cast<float64_t>(box.centroid.x);
  marker->pose.position.y = static_cast<float64_t>(box.centroid.y);
  marker->pose.position.z = static_cast<float64_t>(box.centroid.z);
  marker->pose.orientation.x = static_cast<float64_t>(box.orientation.x);
  marker->pose.orientation.y = static_cast<float64_t>(box.orientation.y);
  marker->pose.orientation.z = static_cast<float64_t>(box.orientation.z);
  marker->pose.orientation.w = static_cast<float64_t>(box.orientation.w);
  marker->scale.y = static_cast<float64_t>(box.size.x);
  marker->scale.x = static_cast<float64_t>(box.size.y);
  marker->scale.z = static_cast<float64_t>(box.size.z);

  return marker;
}


void BoundingBoxArrayDisplay::update(float32_t wall_dt, float32_t ros_dt)
{
  m_marker_common->update(wall_dt, ros_dt);
}

void BoundingBoxArrayDisplay::reset()
{
  RosTopicDisplay::reset();
  m_marker_common->clearMarkers();
}

}  // namespace rviz_plugins
}  // namespace autoware

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(autoware::rviz_plugins::BoundingBoxArrayDisplay, rviz_common::Display)
