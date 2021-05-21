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

#ifndef OBJECT_FLOW_FUSION__UTILS_HPP_
#define OBJECT_FLOW_FUSION__UTILS_HPP_

#include "autoware_perception_msgs/msg/dynamic_object_with_feature_array.h"
#include "geometry_msgs/msg/twist_stamped.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_listener.h"

namespace object_flow_fusion
{
class Utils
{
public:
  Utils() {}
  geometry_msgs::msg::Vector3 mptopic2kph(
    const geometry_msgs::msg::Vector3 & twist, double topic_rate);
  geometry_msgs::msg::Vector3 kph2mptopic(
    const geometry_msgs::msg::Vector3 & twist, double topic_rate);
  geometry_msgs::msg::Vector3 kph2mps(const geometry_msgs::msg::Vector3 & twist);
  geometry_msgs::msg::Twist kph2mps(const geometry_msgs::msg::Twist & twist);
};
}  // namespace object_flow_fusion

#endif  // OBJECT_FLOW_FUSION__UTILS_HPP_
