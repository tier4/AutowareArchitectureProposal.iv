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

#ifndef BEV_OPTICAL_FLOW__NODE_HPP_
#define BEV_OPTICAL_FLOW__NODE_HPP_

#include <iostream>
#include <memory>

#include "autoware_perception_msgs/msg/dynamic_object_with_feature_array.hpp"
#include "bev_optical_flow/flow_calculator.hpp"
#include "bev_optical_flow/lidar_to_image.hpp"
#include "bev_optical_flow/utils.hpp"
#include "geometry_msgs/msg/twist_stamped.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.h"

namespace bev_optical_flow
{
class OpticalFlowNode : public rclcpp::Node
{
public:
  explicit OpticalFlowNode(const rclcpp::NodeOptions & node_options);
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>::SharedPtr
    flow_array_pub_;

  rclcpp::Time prev_stamp_;

  std::shared_ptr<LidarToBEVImage> lidar_to_image_;
  std::shared_ptr<FlowCalculator> flow_calculator_;
  std::shared_ptr<Utils> utils_;
};
}  // namespace bev_optical_flow

#endif  // BEV_OPTICAL_FLOW__NODE_HPP_
