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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <object_detection/detected_objects_display.hpp>

#include <memory>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
DetectedObjectsDisplay::DetectedObjectsDisplay()
: ObjectPolygonDisplayBase("detected_objects") {}

void DetectedObjectsDisplay::processMessage(DetectedObjects::ConstSharedPtr msg)
{
  clear_markers();
  int id = 0;
  for (const auto & object : msg->objects) {
    // Get marker for shape
    auto shape_marker_ptr = get_marker_ptr(
      object.shape, object.kinematics.centroid_position, object.kinematics.orientation,
      object.classification);
    shape_marker_ptr->header = msg->header;
    shape_marker_ptr->id = id++;
    add_marker(shape_marker_ptr);
  }
}

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  autoware::rviz_plugins::object_detection::DetectedObjectsDisplay,
  rviz_common::Display)
