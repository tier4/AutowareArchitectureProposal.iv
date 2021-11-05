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

#include <object_detection/tracked_objects_display.hpp>

#include <memory>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
TrackedObjectsDisplay::TrackedObjectsDisplay()
: ObjectPolygonDisplayBase("tracks") {}

void TrackedObjectsDisplay::processMessage(TrackedObjects::ConstSharedPtr msg)
{
  clear_markers();
  int id = 0;
  for (const auto & object : msg->objects) {
    // Get marker for shape
    auto shape_marker = get_shape_marker_ptr(
      object.shape,
      object.kinematics.pose_with_covariance.pose.position,
      object.kinematics.pose_with_covariance.pose.orientation,
      object.classification);
    if (shape_marker) {
      auto shape_marker_ptr = shape_marker.get();
      shape_marker_ptr->header = msg->header;
      shape_marker_ptr->id = id++;
      add_marker(shape_marker_ptr);
    }

    // Get marker for label
    auto label_marker = get_label_marker_ptr(
      object.kinematics.pose_with_covariance.pose.position,
      object.kinematics.pose_with_covariance.pose.orientation,
      object.classification);
    if (label_marker) {
      auto label_marker_ptr = label_marker.get();
      label_marker_ptr->header = msg->header;
      label_marker_ptr->id = id++;
      add_marker(label_marker_ptr);
    }

    // Get marker for id
    geometry_msgs::msg::Point uuid_vis_position;
    uuid_vis_position.x = object.kinematics.pose_with_covariance.pose.position.x - 0.5;
    uuid_vis_position.y = object.kinematics.pose_with_covariance.pose.position.y;
    uuid_vis_position.z = object.kinematics.pose_with_covariance.pose.position.z - 0.5;

    auto id_marker = get_uuid_marker_ptr(
      object.object_id,
      uuid_vis_position,
      object.classification);
    if (id_marker) {
      auto id_marker_ptr = id_marker.get();
      id_marker_ptr->header = msg->header;
      id_marker_ptr->id = id++;
      add_marker(id_marker_ptr);
    }

    // Get marker for pose with covariance
    auto pose_with_covariance_marker = get_pose_with_covariance_marker_ptr(
      object.kinematics.pose_with_covariance);
    if (pose_with_covariance_marker) {
      auto pose_with_covariance_marker_ptr = pose_with_covariance_marker.get();
      pose_with_covariance_marker_ptr->header = msg->header;
      pose_with_covariance_marker_ptr->id = id++;
      add_marker(pose_with_covariance_marker_ptr);
    }

    // Get marker for velocity text
    geometry_msgs::msg::Point vel_vis_position;
    vel_vis_position.x = uuid_vis_position.x - 0.5;
    vel_vis_position.y = uuid_vis_position.y;
    vel_vis_position.z = uuid_vis_position.z - 0.5;
    auto velocity_text_marker = get_velocity_text_marker_ptr(
      object.kinematics.twist_with_covariance.twist,
      vel_vis_position,
      object.classification);
    if (velocity_text_marker) {
      auto velocity_text_marker_ptr = velocity_text_marker.get();
      velocity_text_marker_ptr->header = msg->header;
      velocity_text_marker_ptr->id = id++;
      add_marker(velocity_text_marker_ptr);
    }

    // Get marker for twist
    auto twist_marker = get_twist_marker_ptr(
      object.kinematics.pose_with_covariance,
      object.kinematics.twist_with_covariance);
    if (twist_marker) {
      auto twist_marker_ptr = twist_marker.get();
      twist_marker_ptr->header = msg->header;
      twist_marker_ptr->id = id++;
      add_marker(twist_marker_ptr);
    }

  }
}

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  autoware::rviz_plugins::object_detection::TrackedObjectsDisplay,
  rviz_common::Display)
