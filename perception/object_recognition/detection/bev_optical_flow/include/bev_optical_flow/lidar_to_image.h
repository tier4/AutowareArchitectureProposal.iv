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
#include <math.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>

#include "utils.h"

namespace bev_optical_flow
{
class LidarToBEVImage
{
public:
  LidarToBEVImage();
  void getBEVImage(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    cv::Mat& bev_image);

private:
  float pointToPixel(const pcl::PointXYZ& point, cv::Point2d& px, float map2base_angle);

  std::shared_ptr<Utils> utils_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  int image_size_;
  float grid_size_;
  float point_radius_;
  float z_max_;
  float z_min_;
  int depth_max_;
  int depth_min_;
};
} // bev_optical_flow
