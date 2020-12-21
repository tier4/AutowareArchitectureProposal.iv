/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include <string>
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

namespace livox_tag_filter
{
class LivoxTagFilterNode
{
public:
  LivoxTagFilterNode();

private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  // Parameter
  std::vector<int> ignore_tags_;

  // Subscriber
  ros::Subscriber sub_pointcloud_;

  void onPointCloud(const sensor_msgs::PointCloud2::ConstPtr & msg);

  // Publisher
  ros::Publisher pub_pointcloud_;
};
}  // namespace livox_tag_filter
