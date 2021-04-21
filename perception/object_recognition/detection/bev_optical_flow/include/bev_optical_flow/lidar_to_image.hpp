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

#ifndef BEV_OPTICAL_FLOW__LIDAR_TO_IMAGE_HPP_
#define BEV_OPTICAL_FLOW__LIDAR_TO_IMAGE_HPP_

#include <math.h>
#include <iostream>
#include <string>
#include <memory>

#include "cv_bridge/cv_bridge.h"

#include "bev_optical_flow/utils.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace bev_optical_flow
{
class LidarToBEVImage
{
public:
  explicit LidarToBEVImage(rclcpp::Node & node);
  void getBEVImage(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg, cv::Mat & bev_image);

private:
  double get_double_param(rclcpp::Node & node, std::string p, const double default_value);
  float pointToPixel(const pcl::PointXYZ & point, cv::Point2d & px, float map2base_angle);

  std::shared_ptr<bev_optical_flow::Utils> utils_;

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  int image_size_;
  float grid_size_;
  float point_radius_;
  float z_max_;
  float z_min_;
  int depth_max_;
  int depth_min_;
};
}  // namespace bev_optical_flow

#endif  // BEV_OPTICAL_FLOW__LIDAR_TO_IMAGE_HPP_
