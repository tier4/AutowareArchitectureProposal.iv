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
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_perception_msgs/DynamicObjectWithFeatureArray.h>
#include "utils.h"
#include "lidar_to_image.h"
#include "flow_calculator.h"

namespace bev_optical_flow
{
class OpticalFlowNode
{
public:
  OpticalFlowNode();
  void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher flow_array_pub_;

  ros::Time prev_stamp_;

  std::shared_ptr<LidarToBEVImage> lidar_to_image_;
  std::shared_ptr<FlowCalculator> flow_calculator_;
  std::shared_ptr<Utils> utils_;
};
} // bev_optical_flow
