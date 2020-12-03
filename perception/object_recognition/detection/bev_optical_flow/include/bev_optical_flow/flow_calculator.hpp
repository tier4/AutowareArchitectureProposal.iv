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

#include <iostream>
#include <opencv2/video/tracking.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <autoware_perception_msgs/DynamicObjectWithFeatureArray.h>
#include "utils.h"
#include "lidar_to_image.h"
#include "debugger.h"

namespace bev_optical_flow
{
class FlowCalculator
{
public:
  FlowCalculator();
  void setup(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
  bool isInitialized();
  void run(autoware_perception_msgs::DynamicObjectWithFeatureArray& scene_flow_array);

private:

  bool calcOpticalFlow(
    cv::Mat& current_image,
    cv::Mat& prev_image,
    std::vector<cv::Point2f>& prev_points,
    autoware_perception_msgs::DynamicObjectWithFeatureArray& flow_array_msg);

  bool getSceneFlowArray(
    const autoware_perception_msgs::DynamicObjectWithFeatureArray& optical_flow_array,
    autoware_perception_msgs::DynamicObjectWithFeatureArray& scene_flow_array);

  bool calcSceneFlow(
    const autoware_perception_msgs::DynamicObjectWithFeature& optical_flow,
    autoware_perception_msgs::DynamicObjectWithFeature& scene_flow);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  float quality_level_;
  int min_distance_;
  int block_size_;
  float harris_k_;
  int max_corners_;
  int sparce_size_;
  int num_split_;
  bool debug_;

  cv::Mat prev_image_;
  std::vector<cv::Point2f> prev_points_;

  cv::Mat image_;
  ros::Time current_stamp_;
  ros::Time prev_stamp_;
  cv::Point2f vehicle_vel_;
  double topic_rate_;
  bool setup_;

  std::shared_ptr<LidarToBEVImage> lidar_to_image_;
  std::shared_ptr<Utils> utils_;

  Debugger debugger_;
};
} // bev_optical_flow
