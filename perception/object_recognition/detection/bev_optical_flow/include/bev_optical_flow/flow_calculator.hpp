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

#ifndef BEV_OPTICAL_FLOW__FLOW_CALCULATOR_HPP_
#define BEV_OPTICAL_FLOW__FLOW_CALCULATOR_HPP_

#include <iostream>
#include <vector>
#include <memory>

#include "cv_bridge/cv_bridge.h"

#include "autoware_perception_msgs/msg/dynamic_object_with_feature_array.hpp"
#include "bev_optical_flow/debugger.hpp"
#include "bev_optical_flow/lidar_to_image.hpp"
#include "bev_optical_flow/utils.hpp"
#include "opencv2/video/tracking.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

namespace bev_optical_flow
{
class FlowCalculator
{
public:
  explicit FlowCalculator(rclcpp::Node & node);
  void setup(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
  bool isInitialized();
  void run(autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & scene_flow_array);

private:
  bool calcOpticalFlow(
    cv::Mat & current_image, cv::Mat & prev_image, std::vector<cv::Point2f> & prev_points,
    autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & flow_array_msg);

  bool getSceneFlowArray(
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & optical_flow_array,
    autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & scene_flow_array);

  bool calcSceneFlow(
    const autoware_perception_msgs::msg::DynamicObjectWithFeature & optical_flow,
    autoware_perception_msgs::msg::DynamicObjectWithFeature & scene_flow);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  float quality_level_;
  int min_distance_;
  int block_size_;
  float harris_k_;
  int max_corners_;
  int sparse_size_;
  int num_split_;
  bool debug_;

  cv::Mat prev_image_;
  std::vector<cv::Point2f> prev_points_;

  cv::Mat image_;
  rclcpp::Time current_stamp_;
  rclcpp::Time prev_stamp_;
  cv::Point2f vehicle_vel_;
  double topic_rate_;
  bool setup_;

  std::shared_ptr<LidarToBEVImage> lidar_to_image_;
  std::shared_ptr<Utils> utils_;

  Debugger debugger_;
};
}  // namespace bev_optical_flow

#endif  // BEV_OPTICAL_FLOW__FLOW_CALCULATOR_HPP_
