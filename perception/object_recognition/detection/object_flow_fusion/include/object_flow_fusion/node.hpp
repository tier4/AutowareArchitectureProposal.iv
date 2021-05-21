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

#ifndef OBJECT_FLOW_FUSION__NODE_HPP_
#define OBJECT_FLOW_FUSION__NODE_HPP_

#include <iostream>
#include <memory>

#include "autoware_perception_msgs/msg/dynamic_object_with_feature_array.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "message_filters/time_synchronizer.h"
#include "object_flow_fusion.hpp"
#include "rclcpp/rclcpp.hpp"

namespace object_flow_fusion
{
class ObjectFlowFusionNode : public rclcpp::Node
{
public:
  ObjectFlowFusionNode();
  void callback(
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr object_msg,
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr flow_msg);

private:
  typedef message_filters::sync_policies::ApproximateTime<
      autoware_perception_msgs::msg::DynamicObjectWithFeatureArray,
      autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>
    ApproximateSync;

  typedef message_filters::sync_policies::ExactTime<
      autoware_perception_msgs::msg::DynamicObjectWithFeatureArray,
      autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>
    Sync;

  std::shared_ptr<message_filters::Synchronizer<ApproximateSync>> approximate_sync_;
  std::shared_ptr<message_filters::Synchronizer<Sync>> sync_;
  message_filters::Subscriber<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>
  object_sub_;
  message_filters::Subscriber<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>
  flow_sub_;

  rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>::SharedPtr pub_;
  bool is_approximate_sync_;
  bool use_flow_pose_;
  float flow_vel_thresh_;
  float fusion_box_offset_;
  std::shared_ptr<ObjectFlowFusion> object_flow_fusion_;
};
}  // namespace object_flow_fusion

#endif  // OBJECT_FLOW_FUSION__NODE_HPP_
