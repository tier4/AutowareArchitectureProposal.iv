/*
 * Copyright 2020 TierIV. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <autoware_perception_msgs/DynamicObjectWithFeatureArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "utils.h"

class Debugger
{
public:
  Debugger();
  bool publishDebugVisualizations(
    const autoware_perception_msgs::DynamicObjectWithFeatureArray& optical_flow_array,
    const autoware_perception_msgs::DynamicObjectWithFeatureArray& scene_flow_array,
    const cv::Mat& image,
    double topic_rate,
    const cv::Point2f& vehicle_vel);

private:
  void publishOpticalFlowImage(const cv::Point2f& vehicle_vel);
  void publishSceneFlowMarker(double topic_rate);
  bool createMarker(
    const autoware_perception_msgs::DynamicObjectWithFeature& scene_flow,
    visualization_msgs::Marker& debug_marker,
    visualization_msgs::Marker& debug_text_marker,
    int idx,
    double topic_rate);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher debug_image_pub_;
  ros::Publisher debug_marker_array_pub_;
  ros::Publisher debug_text_marker_array_pub_;

  autoware_perception_msgs::DynamicObjectWithFeatureArray optical_flow_array_;
  autoware_perception_msgs::DynamicObjectWithFeatureArray scene_flow_array_;
  cv::Mat image_;
  std::shared_ptr<bev_optical_flow::Utils> utils_;
};
