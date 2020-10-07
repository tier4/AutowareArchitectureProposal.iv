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

#include <autoware_utils/geometry/geometry.h>
#include <autoware_utils/ros/transform_listener.h>

namespace autoware_utils
{
class SelfPoseListener
{
public:
  void waitForFirstPose()
  {
    while (ros::ok()) {
      const auto pose = getPoseAt(ros::Time(0), ros::Duration(5.0));
      if (pose) {
        return;
      }
      ROS_INFO("waiting for self pose...");
    }
  }

  geometry_msgs::PoseStamped::ConstPtr getCurrentPose()
  {
    return getPoseAt(ros::Time(0), ros::Duration(0));
  }

  geometry_msgs::PoseStamped::ConstPtr getPoseAt(
    const ros::Time & time, const ros::Duration & duration)
  {
    const auto tf = transform_listener_.getTransform("map", "base_link", time, duration);

    if (!tf) {
      return {};
    }

    geometry_msgs::PoseStamped::Ptr pose(new geometry_msgs::PoseStamped());
    *pose = transform2pose(*tf);

    return geometry_msgs::PoseStamped::ConstPtr(pose);
  }

private:
  TransformListener transform_listener_;
};
}  // namespace autoware_utils
