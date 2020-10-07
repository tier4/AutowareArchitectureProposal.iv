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

#include <ros/ros.h>

template <class T>
T waitForParam(const ros::NodeHandle & nh, const std::string & key)
{
  T value;

  ros::Rate rate(1.0);
  while (ros::ok()) {
    const auto result = nh.getParam(key, value);
    if (result) {
      return value;
    }

    ROS_INFO("waiting for parameter `%s`...", key.c_str());
    rate.sleep();
  }

  return {};
}
