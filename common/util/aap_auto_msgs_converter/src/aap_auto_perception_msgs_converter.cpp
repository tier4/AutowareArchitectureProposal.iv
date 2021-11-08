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

#include "aap_auto_msgs_converter/aap_auto_perception_msgs_converter.hpp"

namespace aap_auto_msgs_converter
{
AAPAutoPerceptionMsgsConverter::AAPAutoPerceptionMsgsConverter(
  const rclcpp::NodeOptions & node_options)
: Node("aap_auto_perception_msgs_converter_node", node_options)
{
  detected_dynamic_object_array_sub_ =
    create_subscription<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>(
      "~/input/detected_dynamic_object_array", rclcpp::QoS{1},
      std::bind(
        &AAPAutoPerceptionMsgsConverter::onDetectedDynamicObjectArray, this,
        std::placeholders::_1));
  tracked_dynamic_object_array_sub_ =
    create_subscription<autoware_perception_msgs::msg::DynamicObjectArray>(
      "~/input/tracked_dynamic_object_array", rclcpp::QoS{1},
      std::bind(
        &AAPAutoPerceptionMsgsConverter::onTrackedDynamicObjectArray, this, std::placeholders::_1));
  predicted_dynamic_object_array_sub_ =
    create_subscription<autoware_perception_msgs::msg::DynamicObjectArray>(
      "~/input/predicted_dynamic_object_array", rclcpp::QoS{1},
      std::bind(
        &AAPAutoPerceptionMsgsConverter::onPredictedDynamicObjectArray, this,
        std::placeholders::_1));
  detected_objects_pub_ = create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "~/output/detected_objects", rclcpp::QoS{1});
  tracked_objects_pub_ = create_publisher<autoware_auto_perception_msgs::msg::TrackedObjects>(
    "~/output/tracked_objects", rclcpp::QoS{1});
  predicted_objects_pub_ = create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>(
    "~/output/predicted_objects", rclcpp::QoS{1});
}

void AAPAutoPerceptionMsgsConverter::onDetectedDynamicObjectArray(
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr input_msg)
{
  const auto detected_objects = convertToDetectedObjects(input_msg);
  detected_objects_pub_->publish(detected_objects);
}

void AAPAutoPerceptionMsgsConverter::onTrackedDynamicObjectArray(
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg)
{
  const auto tracked_objects = convertToTrackedObjects(input_msg);
  tracked_objects_pub_->publish(tracked_objects);
}

void AAPAutoPerceptionMsgsConverter::onPredictedDynamicObjectArray(
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg)
{
  const auto predicted_objects = convertToPredictedObjects(input_msg);
  predicted_objects_pub_->publish(predicted_objects);
}

autoware_auto_perception_msgs::msg::DetectedObjects
AAPAutoPerceptionMsgsConverter::convertToDetectedObjects(
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr input_msg)
{
  using autoware_auto_perception_msgs::msg::DetectedObject;
  using autoware_auto_perception_msgs::msg::DetectedObjects;
  DetectedObjects detected_objects;

  for (const auto & dynamic_obj_with_feature : input_msg->feature_objects) {
    const auto dynamic_obj = dynamic_obj_with_feature.object;
    DetectedObject detected_obj;
    // Semantic -> ObjectClassification
    autoware_auto_perception_msgs::msg::ObjectClassification object_cls;
    object_cls.label = convertLabel(dynamic_obj.semantic.type);
    object_cls.probability = dynamic_obj.semantic.confidence;
    detected_obj.classification.push_back(object_cls);

    // State -> DetectedObjectKinematics
    detected_obj.kinematics.pose_with_covariance = dynamic_obj.state.pose_covariance;
    detected_obj.kinematics.twist_with_covariance = dynamic_obj.state.twist_covariance;
    detected_obj.kinematics.orientation_availability = convertOrientationReliable(dynamic_obj);
    detected_obj.kinematics.has_position_covariance = false;
    detected_obj.kinematics.has_twist = false;
    detected_obj.kinematics.has_twist_covariance = false;

    // Shape -> Shape
    detected_obj.shape.type = dynamic_obj.shape.type;
    detected_obj.shape.footprint = dynamic_obj.shape.footprint;
    detected_obj.shape.dimensions = dynamic_obj.shape.dimensions;

    detected_objects.objects.push_back(detected_obj);
  }

  detected_objects.header = input_msg->header;
  return detected_objects;
}

autoware_auto_perception_msgs::msg::TrackedObjects
AAPAutoPerceptionMsgsConverter::convertToTrackedObjects(
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg)
{
  using autoware_auto_perception_msgs::msg::TrackedObject;
  using autoware_auto_perception_msgs::msg::TrackedObjects;
  TrackedObjects tracked_objects;

  for (const auto & dynamic_obj : input_msg->objects) {
    TrackedObject tracked_obj;
    // UUID -> UUID
    tracked_obj.object_id =dynamic_obj.id;

    // Semantic -> ObjectClassification
    autoware_auto_perception_msgs::msg::ObjectClassification object_cls;
    object_cls.label = convertLabel(dynamic_obj.semantic.type);
    object_cls.probability = dynamic_obj.semantic.confidence;
    tracked_obj.classification.push_back(object_cls);

    // State -> DetectedObjectKinematics
    tracked_obj.kinematics.pose_with_covariance = dynamic_obj.state.pose_covariance;
    tracked_obj.kinematics.twist_with_covariance = dynamic_obj.state.twist_covariance;
    tracked_obj.kinematics.orientation_availability = convertOrientationReliable(dynamic_obj);

    // Shape -> Shape
    autoware_auto_perception_msgs::msg::Shape shape;
    shape.type = dynamic_obj.shape.type;
    shape.footprint = dynamic_obj.shape.footprint;
    shape.dimensions = dynamic_obj.shape.dimensions;
    tracked_obj.shape.push_back(shape);

    tracked_objects.objects.push_back(tracked_obj);
  }

  tracked_objects.header = input_msg->header;
  return tracked_objects;
}

autoware_auto_perception_msgs::msg::PredictedObjects
AAPAutoPerceptionMsgsConverter::convertToPredictedObjects(
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg)
{
  using autoware_auto_perception_msgs::msg::PredictedObject;
  using autoware_auto_perception_msgs::msg::PredictedObjects;
  PredictedObjects predicted_objects;

  for (const auto & dynamic_obj : input_msg->objects) {
    PredictedObject predicted_obj;
    // UUID -> UUID
    predicted_obj.object_id =dynamic_obj.id;

    // Semantic -> ObjectClassification
    autoware_auto_perception_msgs::msg::ObjectClassification object_cls;
    object_cls.label = convertLabel(dynamic_obj.semantic.type);
    object_cls.probability = dynamic_obj.semantic.confidence;
    predicted_obj.classification.push_back(object_cls);

    // State -> DetectedObjectKinematics
    predicted_obj.kinematics.initial_pose_with_covariance = dynamic_obj.state.pose_covariance;
    predicted_obj.kinematics.initial_twist_with_covariance = dynamic_obj.state.twist_covariance;

    for (const auto & aap_path_combination : dynamic_obj.state.predicted_paths) {
      autoware_auto_perception_msgs::msg::PredictedPath predicted_path;
      for (const auto & aap_path : aap_path_combination.path) {
        geometry_msgs::msg::Pose pose = aap_path.pose.pose;
        predicted_path.path.push_back(pose);
      }
      // predicted_path.confidence = 1.0;
      predicted_obj.kinematics.predicted_paths.push_back(predicted_path);
    }

    // Shape -> Shape
    autoware_auto_perception_msgs::msg::Shape shape;
    shape.type = dynamic_obj.shape.type;
    shape.footprint = dynamic_obj.shape.footprint;
    shape.dimensions = dynamic_obj.shape.dimensions;
    predicted_obj.shape.push_back(shape);

    predicted_objects.objects.push_back(predicted_obj);
  }

  predicted_objects.header = input_msg->header;
  return predicted_objects;
}

}  // namespace aap_auto_msgs_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(aap_auto_msgs_converter::AAPAutoPerceptionMsgsConverter)
