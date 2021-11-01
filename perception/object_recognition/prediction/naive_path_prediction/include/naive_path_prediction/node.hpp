/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
 *
 * v1.0 Yukihiro Saito
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"

class NaivePathPredictionNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr pub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr sub_;

  void callback(const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg);

public:
  explicit NaivePathPredictionNode(const rclcpp::NodeOptions & node_options);
};
