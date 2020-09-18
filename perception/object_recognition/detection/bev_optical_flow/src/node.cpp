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

#include "bev_optical_flow/node.h"

namespace bev_optical_flow
{
OpticalFlowNode::OpticalFlowNode() : nh_(""), pnh_("~")
{
  cloud_sub_ = pnh_.subscribe("input_cloud", 1, &OpticalFlowNode::callback, this);
  flow_array_pub_ = pnh_.advertise<
    autoware_perception_msgs::DynamicObjectWithFeatureArray>("output/flows", 1);
  flow_calculator_ = std::make_shared<FlowCalculator>();
}

void OpticalFlowNode::callback(
  const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  flow_calculator_->setup(cloud_msg);
  if ( !flow_calculator_->isInitialized() )
    return;
  autoware_perception_msgs::DynamicObjectWithFeatureArray output_msg;
  output_msg.header = cloud_msg->header;
  flow_calculator_->run(output_msg);
  flow_array_pub_.publish(output_msg);
}
} // bev_optical_flow
