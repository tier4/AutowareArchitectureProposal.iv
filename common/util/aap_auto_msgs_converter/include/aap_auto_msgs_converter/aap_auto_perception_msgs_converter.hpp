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

#ifndef AAP_AUTO_PERCEPTION_MSGS_CONVERTER__AAP_AUTO_PERCEPTION_MSGS_CONVERTER_HPP_
#define AAP_AUTO_PERCEPTION_MSGS_CONVERTER__AAP_AUTO_PERCEPTION_MSGS_CONVERTER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/dynamic_object_array.hpp>
#include <autoware_perception_msgs/msg/dynamic_object_with_feature_array.hpp>

namespace aap_auto_msgs_converter
{
class AAPAutoPerceptionMsgsConverter : public rclcpp::Node
{
public:
  explicit AAPAutoPerceptionMsgsConverter(const rclcpp::NodeOptions & node_options);

  template <class KinematicsT>
  using Kinematics = KinematicsT;

private:
  void onDetectedDynamicObjectArray(
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr input_msg);
  void onTrackedDynamicObjectArray(
    const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg);
  void onPredictedDynamicObjectArray(
    const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg);

  autoware_auto_perception_msgs::msg::DetectedObjects
  convertToDetectedObjects(
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr input_msg);

  autoware_auto_perception_msgs::msg::TrackedObjects
  convertToTrackedObjects(
    const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg);

  autoware_auto_perception_msgs::msg::PredictedObjects
  convertToPredictedObjects(
    const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg);

  rclcpp::Subscription<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>::SharedPtr
    detected_dynamic_object_array_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr
    tracked_dynamic_object_array_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr
    predicted_dynamic_object_array_sub_;

  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr
    detected_objects_pub_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr
    tracked_objects_pub_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr
    predicted_objects_pub_;

  uint8_t
  convertOrientationReliable(const autoware_perception_msgs::msg::DynamicObject & dynamic_obj)
  {
    using autoware_perception_msgs::msg::Semantic;
    if (dynamic_obj.state.orientation_reliable) {
      return autoware_auto_perception_msgs::msg::DetectedObjectKinematics::AVAILABLE;
    } else {
      if (dynamic_obj.semantic.type == Semantic::CAR) {
        return autoware_auto_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
      } else {
        return autoware_auto_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE;
      }
    }
  }

  autoware_auto_perception_msgs::msg::ObjectClassification::_label_type convertLabel(
    autoware_perception_msgs::msg::Semantic::_type_type type)
  {
    using autoware_auto_perception_msgs::msg::ObjectClassification;
    using autoware_perception_msgs::msg::Semantic;
    if (type == Semantic::UNKNOWN) {
      return ObjectClassification::UNKNOWN;
    } else if (type == Semantic::CAR) {
      return ObjectClassification::CAR;
    } else if (type == Semantic::TRUCK) {
      return ObjectClassification::TRUCK;
    } else if (type == Semantic::BUS) {
      return ObjectClassification::BUS;
    } else if (type == Semantic::BICYCLE) {
      return ObjectClassification::BICYCLE;
    } else if (type == Semantic::MOTORBIKE) {
      return ObjectClassification::MOTORCYCLE;
    } else if (type == Semantic::UNKNOWN) {
      return ObjectClassification::UNKNOWN;
    } else if (type == Semantic::PEDESTRIAN) {
      return ObjectClassification::PEDESTRIAN;
    } else if (type == Semantic::ANIMAL) {
      return ObjectClassification::UNKNOWN;
    } else {
      return ObjectClassification::UNKNOWN;
    }
  }
};

}  // namespace aap_auto_msgs_converter

#endif  // AAP_AUTO_PERCEPTION_MSGS_CONVERTER__AAP_AUTO_PERCEPTION_MSGS_CONVERTER_HPP_
