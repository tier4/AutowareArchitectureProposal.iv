/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef EMERGENCY_HANDLER_CORE_H_
#define EMERGENCY_HANDLER_CORE_H_

// Core
#include <string>

// ROS2 core
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

// Autoware
#include <autoware_control_msgs/msg/control_command_stamped.hpp>
#include <autoware_control_msgs/msg/gate_mode.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <autoware_system_msgs/msg/driving_capability.hpp>
#include <autoware_vehicle_msgs/msg/shift_stamped.hpp>
#include <autoware_vehicle_msgs/msg/turn_signal.hpp>
#include <autoware_vehicle_msgs/msg/vehicle_command.hpp>

class EmergencyHandler : public rclcpp::Node
{
public:
  EmergencyHandler(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  // NodeHandle
  // ros::NodeHandle nh_{""};
  // ros::NodeHandle private_nh_{"~"};

  // Subscribers
  // ros::Subscriber sub_autoware_state_;
  // ros::Subscriber sub_driving_capability_;
  // ros::Subscriber sub_prev_control_command_;
  // ros::Subscriber sub_current_gate_mode_;
  // ros::Subscriber sub_twist_;

  autoware_system_msgs::msg::AutowareState::ConstSharedPtr autoware_state_;
  autoware_system_msgs::msg::DrivingCapability::ConstSharedPtr driving_capability_;
  autoware_control_msgs::msg::ControlCommand::ConstSharedPtr prev_control_command_;
  autoware_control_msgs::msg::GateMode::ConstSharedPtr current_gate_mode_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_;

  void onAutowareState(const autoware_system_msgs::msg::AutowareState::ConstSharedPtr msg);
  void onDrivingCapability(const autoware_system_msgs::msg::DrivingCapability::ConstSharedPtr msg);
  // To be replaced by ControlCommand
  void onPrevControlCommand(const autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr msg);
  void onCurrentGateMode(const autoware_control_msgs::msg::GateMode::ConstSharedPtr msg);
  void onTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);

  // Publisher
  // ros::Publisher pub_control_command_;
  // ros::Publisher pub_shift_;
  // ros::Publisher pub_turn_signal_;
  // ros::Publisher pub_is_emergency_;

  // Timer
  // ros::Timer timer_;

  // Parameters
  double update_rate_;
  bool use_parking_after_stopped_;

  bool isDataReady();
  // void onTimer(const ros::TimerEvent & event);

  // Algorithm
  bool isStopped();
  bool isEmergency();
  autoware_control_msgs::msg::ControlCommand selectAlternativeControlCommand();
};

#endif
