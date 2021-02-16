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
#include <utility>
#include "trajectory_test_publisher/node.hpp"

TrajectoryTestPublisherNode::TrajectoryTestPublisherNode()
: Node("trajectory_test_publisher_node")
{
  // register publisher
  traj_pub_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>("trajectory", 1);

  // register timer
  auto timer_callback = std::bind(&TrajectoryTestPublisherNode::timerCallback, this);
  const auto period = std::chrono::milliseconds(100);
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void TrajectoryTestPublisherNode::timerCallback()
{
  autoware_planning_msgs::msg::Trajectory trajectory_msg;
  trajectory_msg.header.frame_id = "map";
  trajectory_msg.header.stamp = this->get_clock()->now();

  size_t trajectory_size = 100;
  for (size_t i = 0; i < trajectory_size; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(i / 100.0);
    point.pose.position.y = static_cast<double>(0.0);
    point.pose.position.z = static_cast<double>(0.0);
    point.pose.orientation.x = static_cast<double>(0.0);
    point.pose.orientation.y = static_cast<double>(0.0);
    point.pose.orientation.z = static_cast<double>(0.0);
    point.pose.orientation.w = static_cast<double>(1.0);
    point.twist.linear.x = static_cast<double>(10.0) *
      ((static_cast<double>(trajectory_size) - 1.0) - static_cast<double>(i)) /
      (static_cast<double>(trajectory_size) - 1.0);
    point.twist.linear.y = static_cast<double>(0.0);
    point.twist.linear.z = static_cast<double>(0.0);

    trajectory_msg.points.push_back(point);
  }
  traj_pub_->publish(trajectory_msg);
}
