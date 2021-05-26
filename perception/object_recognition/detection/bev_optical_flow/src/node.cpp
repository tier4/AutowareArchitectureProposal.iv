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

#include <memory>

#include "bev_optical_flow/node.hpp"

namespace bev_optical_flow
{
OpticalFlowNode::OpticalFlowNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("bev_optical_flow", node_options)
{
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_cloud", 1, std::bind(&OpticalFlowNode::callback, this, std::placeholders::_1));
  flow_array_pub_ =
    this->create_publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>(
    "output/flows", 1);
  flow_calculator_ = std::make_shared<FlowCalculator>(*this);
}

void OpticalFlowNode::callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  flow_calculator_->setup(cloud_msg);
  if (!flow_calculator_->isInitialized()) {
    return;
  }
  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray output_msg;
  output_msg.header = cloud_msg->header;
  flow_calculator_->run(output_msg);
  flow_array_pub_->publish(output_msg);
}
}  // namespace bev_optical_flow

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bev_optical_flow::OpticalFlowNode)
