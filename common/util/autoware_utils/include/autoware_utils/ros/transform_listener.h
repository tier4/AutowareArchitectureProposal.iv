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

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>

class TransformListener
{
public:
  geometry_msgs::TransformStamped::ConstPtr getTransform(
    const std::string & from, const std::string & to, const ros::Time & time,
    const ros::Duration & duration)
  {
    geometry_msgs::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(from, to, time, duration);
    } catch (tf2::TransformException & ex) {
      ROS_WARN("failed to get transform from %s to %s: %s", from.c_str(), to.c_str(), ex.what());
      return {};
    }

    geometry_msgs::TransformStamped::Ptr transform(new geometry_msgs::TransformStamped());
    *transform = tf;

    return transform;
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
};
