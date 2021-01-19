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

#include <map>
#include <sstream>
#include <string>

#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticStatus.h>

namespace autoware_utils
{
class ProcessingTimePublisher
{
public:
  ProcessingTimePublisher() : ProcessingTimePublisher("debug/processing_time_ms") {}
  explicit ProcessingTimePublisher(
    const char * name, const uint32_t queue_size = 1, const bool latch = false)
  {
    pub_processing_time_ =
      private_nh_.advertise<diagnostic_msgs::DiagnosticStatus>(name, queue_size, latch);
  }

  void publish(const std::map<std::string, double> & processing_time_map)
  {
    diagnostic_msgs::DiagnosticStatus status;

    for (const auto & m : processing_time_map) {
      diagnostic_msgs::KeyValue key_value;
      key_value.key = m.first;
      key_value.value = to_string_with_precision(m.second, 3);
      status.values.push_back(key_value);
    }

    pub_processing_time_.publish(status);
  }

private:
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  ros::Publisher pub_processing_time_;

  template <class T>
  std::string to_string_with_precision(const T & value, const int precision)
  {
    std::ostringstream oss;
    oss.precision(precision);
    oss << std::fixed << value;
    return oss.str();
  }
};
}  // namespace autoware_utils
