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
#pragma once
#include "pcl_conversions/pcl_conversions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "cluster_data_association/data_association.hpp"
#include <memory>
#include "autoware_perception_msgs/msg/dynamic_object_with_feature_array.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include <rclcpp/rclcpp.hpp>

namespace cluster_data_association
{
class ClusterDataAssociationNode : public rclcpp::Node
{
public:
  ClusterDataAssociationNode();
  ~ClusterDataAssociationNode() {}

private:
  void clusterCallback(
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr & input_cluster0_msg,
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr & input_cluster1_msg);

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>::SharedPtr
    associated_cluster_pub_;
  message_filters::Subscriber<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>
  cluster0_sub_;
  message_filters::Subscriber<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>
  cluster1_sub_;
  typedef message_filters::sync_policies::ApproximateTime<
      autoware_perception_msgs::msg::DynamicObjectWithFeatureArray,
      autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  Sync sync_;
  DataAssociation data_association_;

  // ROS Parameters
};

}  // namespace cluster_data_association
