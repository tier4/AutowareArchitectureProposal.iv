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

#ifndef AUTOWARE_UTILS__ROS__TRANSFORM_LISTENER_HPP_
#define AUTOWARE_UTILS__ROS__TRANSFORM_LISTENER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

class TransformListener
{
public:
  explicit TransformListener(rclcpp::Node * node)
  : clock_(node->get_clock()), logger_(node->get_logger())
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node->get_node_base_interface(), node->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
  geometry_msgs::msg::TransformStamped::ConstSharedPtr getTransform(
    const std::string & from, const std::string & to, const rclcpp::Time & time,
    const rclcpp::Duration & duration)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(from, to, time, duration);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(
        logger_, "failed to get transform from %s to %s: %s", from.c_str(), to.c_str(), ex.what());
      return {};
    }

    geometry_msgs::msg::TransformStamped::SharedPtr transform(
      new geometry_msgs::msg::TransformStamped());
    *transform = tf;

    return transform;
  }
  rclcpp::Logger getLogger() {return logger_;}

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif  // AUTOWARE_UTILS__ROS__TRANSFORM_LISTENER_HPP_
