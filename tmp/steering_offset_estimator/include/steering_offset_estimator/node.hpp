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

#pragma once
#include <memory>
#include <string>

#include <autoware_vehicle_msgs/msg/steering.hpp>
#include <autoware_vehicle_msgs/msg/steering_offset.hpp>
#include <autoware_vehicle_msgs/msg/steering_offset_covariance.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "autoware_control_msgs/msg/control_command_stamped.hpp"
#include "autoware_vehicle_msgs/msg/steering.h.hpp"
#include "rclcpp/rclcpp.hpp"

class SteeringOffsetEstimtaor : public rclcpp::Node
{
public:
  explicit SteeringOffsetEstimtaor(const rclcpp::NodeOptions & node_options);

private:

  std::shared_ptr<geometry_msgs::msg::twist_stamped> twist_ptr_;
  std::shared_ptr<autoware_vehicle_msgs::msg::steering> steer_ptr_;
  double wheel_base_;
  double update_hz_;
  double estimated_steer_offset_;
  double covariance_;
  double forgetting_factor_;
  double valid_min_velocity_;
  double valid_max_steer_;

  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringOffset>::SharedPtr steer_offset_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringOffsetCovariance>::SharedPtr steer_offset_cov_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Steering>::SharedPtr steer_sub_;

  ros::Timer timer_;

  void callbackTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr & msg);
  void callbackSteer(const autoware_vehicle_msgs::msg::Steering::ConstSharedPtr & msg);
  bool updateSteeringOffset();

  void latCtrlCmdCallback(const autoware_control_msgs::msg::ControlCommandStamped::SharedPtr msg);
  void lonCtrlCmdCallback(const autoware_control_msgs::msg::ControlCommandStamped::SharedPtr msg);
  void publishCmd();
  bool checkTimeout();

  rclcpp::Publisher<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr control_cmd_pub_;
  rclcpp::Subscription<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr
    lat_control_cmd_sub_;
  rclcpp::Subscription<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr
    lon_control_cmd_sub_;

  std::shared_ptr<autoware_control_msgs::msg::ControlCommandStamped> lat_cmd_;
  std::shared_ptr<autoware_control_msgs::msg::ControlCommandStamped> lon_cmd_;
  double timeout_thr_sec_;
};

