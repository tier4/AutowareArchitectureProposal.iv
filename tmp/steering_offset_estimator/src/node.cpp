// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include "steering_offset_estimator/node.hpp"

#include <string>
#include <memory>

#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp/time.hpp"

SteeringOffsetEstimtaor::SteeringOffsetEstimtaor(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("steering_offset_estimator", node_options)
{
  steer_offset_pub_ =
    create_publisher<autoware_vehicle_msgs::msg::SteeringOffset>(
    "output/streering_offset",
    rclcpp::QoS{1}.transient_local());
  steer_offset_covariance_pub_ =
    create_publisher<autoware_vehicle_msgs::msg::SteeringOffsetCovariance>(
    "output/streering_offset_covariance",
    rclcpp::QoS{1}.transient_local());
  
  twist_sub_ =
    create_subscription<autoware_control_msgs::msg::TwistStamped>(
    "input/twist", rclcpp::QoS{1},
    std::bind(&SteeringOffsetEstimtaor::callbackTwist, this, std::placeholders::_1));
  lon_control_cmd_sub_ =
    create_subscription<autoware_control_msgs::msg::ControlCommandStamped>(
    "input/longitudinal/control_cmd", rclcpp::QoS{1},
    std::bind(&SteeringOffsetEstimtaor::callbackSteer, this, std::placeholders::_1));
  timeout_thr_sec_ = declare_parameter("timeout_thr_sec", 0.5);
}

bool SteeringOffsetEstimtaor::checkTimeout()
{
  const auto now = this->now();
  if ((now - lat_cmd_->header.stamp).seconds() > timeout_thr_sec_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "lat_cmd_ timeout failed.");
    return false;
  }
  if ((now - lon_cmd_->header.stamp).seconds() > timeout_thr_sec_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "lon_cmd_ timeout failed.");
    return false;
  }
  return true;
}

void SteeringOffsetEstimtaor::publishCmd()
{
  if (!lat_cmd_ || !lon_cmd_) {
    return;
  }
  if (!checkTimeout()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000 /*ms*/,
      "timeout failed. stop publish command.");
    return;
  }

  autoware_control_msgs::msg::ControlCommandStamped out;
  out.header.stamp = rclcpp::Node::now();
  out.header.frame_id = "base_link";
  out.control.steering_angle = lat_cmd_->control.steering_angle;
  out.control.steering_angle_velocity = lat_cmd_->control.steering_angle_velocity;
  out.control.velocity = lon_cmd_->control.velocity;
  out.control.acceleration = lon_cmd_->control.acceleration;

  control_cmd_pub_->publish(out);
}

void SteeringOffsetEstimtaor::latCtrlCmdCallback(
  const autoware_control_msgs::msg::ControlCommandStamped::SharedPtr input_msg)
{
  lat_cmd_ = std::make_shared<autoware_control_msgs::msg::ControlCommandStamped>(*input_msg);
  publishCmd();
}

void SteeringOffsetEstimtaor::lonCtrlCmdCallback(
  const autoware_control_msgs::msg::ControlCommandStamped::SharedPtr input_msg)
{
  lon_cmd_ = std::make_shared<autoware_control_msgs::msg::ControlCommandStamped>(*input_msg);
  publishCmd();
}

RCLCPP_COMPONENTS_REGISTER_NODE(SteeringOffsetEstimtaor)
