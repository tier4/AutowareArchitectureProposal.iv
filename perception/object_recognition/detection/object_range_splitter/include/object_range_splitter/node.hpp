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
#include <ros/ros.h>
#include <memory>
#include "autoware_perception_msgs/DynamicObjectWithFeatureArray.h"

namespace object_range_splitter
{
class ObjectRangeSplitterNode
{
public:
  ObjectRangeSplitterNode();
  ~ObjectRangeSplitterNode() = default;

private:
  void objectCallback(
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_msg);

  ros::NodeHandle nh_, pnh_;
  ros::Publisher long_range_object_pub_;
  ros::Publisher short_range_object_pub_;
  ros::Subscriber sub_;

  // ROS Parameters
  float spilt_range_;
};

}  // namespace object_range_splitter
