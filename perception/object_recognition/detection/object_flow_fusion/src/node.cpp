/*
 * Copyright 2020 TierIV. All rights reserved.
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

#include "object_flow_fusion/node.h"

namespace object_flow_fusion
{
ObjectFlowFusionNode::ObjectFlowFusionNode() : nh_(""), pnh_("~")
{
  pnh_.param<bool>("is_approximate_sync", is_approximate_sync_, true);
  pnh_.param<bool>("use_flow_pose", use_flow_pose_, true);
  pnh_.param<float>("flow_vel_thresh", flow_vel_thresh_, 10.0);

  object_sub_.subscribe(pnh_, "input_object", 1);
  flow_sub_.subscribe(pnh_, "input_flow", 1);

  if (is_approximate_sync_) {
    approximate_sync_ = boost::make_shared<message_filters::Synchronizer<ApproximateSync> >(1000);
    approximate_sync_->connectInput(object_sub_, flow_sub_);
    approximate_sync_->registerCallback(boost::bind(&ObjectFlowFusionNode::callback, this, _1, _2));
  } else {
    sync_  = boost::make_shared<message_filters::Synchronizer<Sync> >(1000);
    sync_->connectInput(object_sub_, flow_sub_);
    sync_->registerCallback(boost::bind(&ObjectFlowFusionNode::callback, this, _1, _2));
  }

  pub_ = pnh_.advertise<autoware_perception_msgs::DynamicObjectWithFeatureArray>("output", 1);
  object_flow_fusion_ = std::make_shared<ObjectFlowFusion>();
}

void ObjectFlowFusionNode::callback(
  const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr& object_msg,
  const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr& flow_msg)
{
  autoware_perception_msgs::DynamicObjectWithFeatureArray fused_msg;
  fused_msg.header = object_msg->header;
  object_flow_fusion_->fusion(object_msg, flow_msg, use_flow_pose_, flow_vel_thresh_, fused_msg);
  pub_.publish(fused_msg);
}

} // object_flow_fusion_node
