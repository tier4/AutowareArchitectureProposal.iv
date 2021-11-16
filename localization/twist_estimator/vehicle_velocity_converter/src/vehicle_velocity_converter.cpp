// Copyright 2021 TierIV
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

#include "vehicle_velocity_converter/vehicle_velocity_converter.hpp"

VehicleVelocityConverter::VehicleVelocityConverter() : Node("vehicle_velocity_converter")
{
  // set covariance value for twist with covariance msg
  std::vector<double> covariance;
  declare_parameter("twist_covariance", covariance);
  for (std::size_t i = 0; i < covariance.size(); ++i) {
    twist_covariance_[i] = covariance[i];
  }
  frame_id_ = declare_parameter("frame_id", "base_link");

  // velocity report to twist stamped and stamped with covariance
  vehicle_report_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "~/input/velocity_report", rclcpp::QoS{100},
    std::bind(&VehicleVelocityConverter::callbackVelocityReport, this, std::placeholders::_1));
  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("twist", rclcpp::QoS{10});
  twist_with_covariance_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance", rclcpp::QoS{10});

  // twist stamped to velocity report
  twist_stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/input/twist_stamped", rclcpp::QoS{100},
    std::bind(&VehicleVelocityConverter::callbackTwistStamped, this, std::placeholders::_1));
  velocity_report_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "~/output/velocity_report", rclcpp::QoS{10});
}

void VehicleVelocityConverter::callbackVelocityReport(
  const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
{
/*   if (msg->header.frame_id != frame_id_) {
    RCLCPP_WARN(get_logger(), "frame_id is not base_link.");
  }
  // set twist stamp msg from vehicle report msg
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header = msg->header;
  twist_msg.twist.linear.x = msg->longitudinal_velocity;
  twist_msg.twist.linear.y = msg->lateral_velocity;

  // set twist with covariance msg from vehicle report msg
  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance_msg;
  twist_with_covariance_msg.header = msg->header;
  twist_with_covariance_msg.twist.twist = twist_msg.twist;
  twist_with_covariance_msg.twist.covariance = twist_covariance_;

  twist_pub_->publish(twist_msg);
  twist_with_covariance_pub_->publish(twist_with_covariance_msg); */
}

void VehicleVelocityConverter::callbackTwistStamped(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (msg->header.frame_id != frame_id_) {
    RCLCPP_WARN(get_logger(), "frame_id is not base_link.");
  }

  autoware_auto_vehicle_msgs::msg::VelocityReport velocity_report_msg;
  velocity_report_msg.header = msg->header;
  velocity_report_msg.longitudinal_velocity = msg->twist.linear.x;
  velocity_report_msg.lateral_velocity = msg->twist.linear.y;
  velocity_report_msg.heading_rate = msg->twist.angular.z;
  velocity_report_pub_->publish(velocity_report_msg);
}
