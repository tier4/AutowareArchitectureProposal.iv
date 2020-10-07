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
#include <unordered_map>

#include <ros/ros.h>

namespace debug_publisher
{
template <class T_msg, class T>
T_msg toMsg(const T & data)
{
  T_msg msg;
  msg.data = data;
  return msg;
}
}  // namespace debug_publisher

namespace autoware_utils
{
class DebugPublisher
{
public:
  DebugPublisher() : ns_("debug") {}
  explicit DebugPublisher(const char * ns) : ns_(ns) {}

  template <
    class T, std::enable_if_t<ros::message_traits::IsMessage<T>::value, std::nullptr_t> = nullptr>
  void publish(
    const std::string & name, const T & data, const uint32_t queue_size = 1,
    const bool latch = false)
  {
    if (pub_map_.count(name) == 0) {
      pub_map_[name] = private_nh_.advertise<T>(std::string(ns_) + "/" + name, queue_size, latch);
    }

    pub_map_.at(name).publish(data);
  }

  template <
    class T_msg, class T,
    std::enable_if_t<!ros::message_traits::IsMessage<T>::value, std::nullptr_t> = nullptr>
  void publish(
    const std::string & name, const T & data, const uint32_t queue_size = 1,
    const bool latch = false)
  {
    publish(name, debug_publisher::toMsg<T_msg>(data), queue_size, latch);
  }

private:
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  const char * ns_;
  std::unordered_map<std::string, ros::Publisher> pub_map_;
};
}  // namespace autoware_utils
