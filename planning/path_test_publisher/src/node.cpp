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
#include "path_test_publisher/node.hpp"

PathTestPublisherNode::PathTestPublisherNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("path_test_publisher", node_options)
{
  pub_ = create_publisher<autoware_planning_msgs::msg::Path>("path", rclcpp::QoS{1});

  auto timer_callback = std::bind(&PathTestPublisherNode::timerCallback, this);
  using namespace std::literals::chrono_literals;
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), 100ms, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void PathTestPublisherNode::timerCallback()
{
  autoware_planning_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = now();

  constexpr size_t path_size = 100;
  for (size_t i = 0; i < path_size; ++i) {
    double dmax = static_cast<double>(path_size - 1);
    double didx = static_cast<double>(i);
    autoware_planning_msgs::msg::PathPoint point;
    point.pose.position.x = didx;
    point.pose.position.y = 0.0;
    point.pose.position.z = 0.0;
    point.pose.orientation.x = 0.0;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = 0.0;
    point.pose.orientation.w = 1.0;
    point.twist.linear.x = 10.0 * ((dmax - didx) / dmax);
    point.twist.linear.y = 0.0;
    point.twist.linear.z = 0.0;
    path_msg.points.push_back(point);
  }
  pub_->publish(path_msg);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PathTestPublisherNode)
