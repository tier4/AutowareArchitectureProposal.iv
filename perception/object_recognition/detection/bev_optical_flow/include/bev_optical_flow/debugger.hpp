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

#ifndef BEV_OPTICAL_FLOW__DEBUGGER_HPP_
#define BEV_OPTICAL_FLOW__DEBUGGER_HPP_

#include <memory>

#include "cv_bridge/cv_bridge.h"

#include "autoware_perception_msgs/msg/dynamic_object_with_feature_array.hpp"
#include "bev_optical_flow/utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class Debugger
{
public:
  explicit Debugger(rclcpp::Node & node);
  bool publishDebugVisualizations(
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & optical_flow_array,
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & scene_flow_array,
    const cv::Mat & image, double topic_rate, const cv::Point2f & vehicle_vel);

private:
  void publishOpticalFlowImage(const cv::Point2f & vehicle_vel);
  void publishSceneFlowMarker(double topic_rate);
  bool createMarker(
    const autoware_perception_msgs::msg::DynamicObjectWithFeature & scene_flow,
    visualization_msgs::msg::Marker & debug_marker,
    visualization_msgs::msg::Marker & debug_text_marker, int idx, double topic_rate);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_array_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_text_marker_array_pub_;

  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray optical_flow_array_;
  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray scene_flow_array_;
  cv::Mat image_;
  std::shared_ptr<bev_optical_flow::Utils> utils_;
};

#endif  // BEV_OPTICAL_FLOW__DEBUGGER_HPP_
