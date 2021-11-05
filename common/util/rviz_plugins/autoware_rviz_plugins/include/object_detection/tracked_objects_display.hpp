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
#ifndef OBJECT_DETECTION__TRACKED_OBJECTS_DISPLAY_HPP_
#define OBJECT_DETECTION__TRACKED_OBJECTS_DISPLAY_HPP_

#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <object_detection/object_polygon_display_base.hpp>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
/// \brief Class defining rviz plugin to visualize TrackedObjects
class AUTOWARE_RVIZ_PLUGINS_PUBLIC TrackedObjectsDisplay
  : public ObjectPolygonDisplayBase<autoware_auto_perception_msgs::msg::TrackedObjects>
{
  Q_OBJECT

public:
  using TrackedObjects = autoware_auto_perception_msgs::msg::TrackedObjects;

  TrackedObjectsDisplay();

private:
  void processMessage(TrackedObjects::ConstSharedPtr msg) override;

  visualization_msgs::msg::Marker::SharedPtr get_marker_ptr_for_track_id(
    const autoware_auto_perception_msgs::msg::TrackedObject & track);
};

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

#endif  // OBJECT_DETECTION__TRACKED_OBJECTS_DISPLAY_HPP_
