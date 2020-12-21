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

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
// #include "tf2_sensor_msgs/msg/tf2_sensor_msgs.hpp"
#include <chrono>
#include "cluster_data_association/node.hpp"
#define EIGEN_MPL2_ONLY
#include "Eigen/Core"
#include "Eigen/Geometry"

namespace cluster_data_association
{
ClusterDataAssociationNode::ClusterDataAssociationNode()
: rclcpp::Node("cluster_data_association_node"),
  tf_listener_(tf_buffer_),
  cluster0_sub_(this, "input/clusters0", rclcpp::QoS{1}.get_rmw_qos_profile()),
  cluster1_sub_(this, "input/clusters1", rclcpp::QoS{1}.get_rmw_qos_profile()),
  sync_(SyncPolicy(10), cluster0_sub_, cluster1_sub_)
{
  sync_.registerCallback(boost::bind(&ClusterDataAssociationNode::clusterCallback, this, _1, _2));

  associated_cluster_pub_ =
    create_publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>(
    "output/clusters", rclcpp::QoS{10});
}

void ClusterDataAssociationNode::clusterCallback(
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr & input_cluster0_msg,
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr & input_cluster1_msg)
{
  // Guard
  if (associated_cluster_pub_->get_subscription_count() < 1) {return;}

  // build output msg
  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray output_msg;
  output_msg.header = input_cluster0_msg->header;

  /* global nearest neighboor */
  std::unordered_map<int, int> direct_assignment;
  std::unordered_map<int, int> reverse_assignment;
  Eigen::MatrixXd score_matrix =
    data_association_.calcScoreMatrix(*input_cluster1_msg, *input_cluster0_msg);
  data_association_.assign(score_matrix, direct_assignment, reverse_assignment);
  for (size_t cluster0_idx = 0; cluster0_idx < input_cluster0_msg->feature_objects.size();
    ++cluster0_idx)
  {
    if (direct_assignment.find(cluster0_idx) != direct_assignment.end()) { // found
      output_msg.feature_objects.push_back(input_cluster0_msg->feature_objects.at(cluster0_idx));
    } else { // not found
      output_msg.feature_objects.push_back(input_cluster0_msg->feature_objects.at(cluster0_idx));
    }
  }
  for (size_t cluster1_idx = 0; cluster1_idx < input_cluster1_msg->feature_objects.size();
    ++cluster1_idx)
  {
    if (reverse_assignment.find(cluster1_idx) != reverse_assignment.end()) { // found
    } else { // not found
      output_msg.feature_objects.push_back(input_cluster1_msg->feature_objects.at(cluster1_idx));
    }
  }

  // publish output msg
  associated_cluster_pub_->publish(output_msg);
}
}  // namespace cluster_data_association
