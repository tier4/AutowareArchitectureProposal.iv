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

#include <functional>

#include "object_flow_fusion/node.hpp"

namespace object_flow_fusion
{
ObjectFlowFusionNode::ObjectFlowFusionNode()
: Node("object_flow_fusion")
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  is_approximate_sync_ = declare_parameter("is_approximate_sync", true);
  use_flow_pose_ = declare_parameter("use_flow_pose", true);
  flow_vel_thresh_ = declare_parameter<float>("flow_vel_thresh", 10.0);

  object_sub_.subscribe(this, "input_object", rclcpp::QoS{1}.get_rmw_qos_profile());
  flow_sub_.subscribe(this, "input_flow", rclcpp::QoS{1}.get_rmw_qos_profile());

  if (is_approximate_sync_) {
    approximate_sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSync> >(1000);
    approximate_sync_->connectInput(object_sub_, flow_sub_);
    approximate_sync_->registerCallback(std::bind(&ObjectFlowFusionNode::callback, this, _1, _2));
  } else {
    sync_  = std::make_shared<message_filters::Synchronizer<Sync> >(1000);
    sync_->connectInput(object_sub_, flow_sub_);
    sync_->registerCallback(std::bind(&ObjectFlowFusionNode::callback, this, _1, _2));
  }

  pub_ = create_publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>("output", rclcpp::QoS{1});
  object_flow_fusion_ = std::make_shared<ObjectFlowFusion>();
}

void ObjectFlowFusionNode::callback(
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstPtr& object_msg,
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstPtr& flow_msg)
{
  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray fused_msg;
  fused_msg.header = object_msg->header;
  object_flow_fusion_->fusion(object_msg, flow_msg, use_flow_pose_, flow_vel_thresh_, fused_msg);
  pub_->publish(fused_msg);
}

} // object_flow_fusion_node
