// Copyright 2020 TierIV
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

#pragma once

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "autoware_perception_msgs/msg/dynamic_object_with_feature_array.hpp"
#include "object_flow_fusion.hpp"

namespace object_flow_fusion
{
class ObjectFlowFusionNode : public rclcpp::Node
{
public:
  ObjectFlowFusionNode();
  void callback(
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstPtr& object_msg,
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstPtr& flow_msg);
private:
  typedef message_filters::sync_policies::ApproximateTime<
  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray,
  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray
  > ApproximateSync;

  typedef message_filters::sync_policies::ExactTime<
    autoware_perception_msgs::msg::DynamicObjectWithFeatureArray,
    autoware_perception_msgs::msg::DynamicObjectWithFeatureArray
    > Sync;

  std::shared_ptr<message_filters::Synchronizer<ApproximateSync> > approximate_sync_;
  std::shared_ptr<message_filters::Synchronizer<Sync> > sync_;
  message_filters::Subscriber<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray> object_sub_;
  message_filters::Subscriber<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray> flow_sub_;

  rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>::SharedPtr pub_;
  bool is_approximate_sync_;
  bool use_flow_pose_;
  float flow_vel_thresh_;
  std::shared_ptr<ObjectFlowFusion> object_flow_fusion_;

};
} // object_flow_fusion
