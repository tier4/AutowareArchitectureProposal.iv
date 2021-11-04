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

#ifndef EMERGENCY_HANDLER__EMERGENCY_HANDLER_CORE_HPP_
#define EMERGENCY_HANDLER__EMERGENCY_HANDLER_CORE_HPP_

// Core
#include <memory>
#include <string>

// Autoware
#include <autoware_control_msgs/msg/control_command_stamped.hpp>
#include <autoware_system_msgs/msg/emergency_state_stamped.hpp>
#include <autoware_system_msgs/msg/hazard_status_stamped.hpp>
#include <autoware_vehicle_msgs/msg/control_mode.hpp>
#include <autoware_vehicle_msgs/msg/shift_stamped.hpp>
#include <autoware_vehicle_msgs/msg/turn_signal.hpp>
#include <autoware_vehicle_msgs/msg/vehicle_command.hpp>

// ROS2 core
#include <autoware_utils/system/heartbeat_checker.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

struct HazardLampPolicy
{
  bool emergency;
};

struct Param
{
  int update_rate;
  double timeout_hazard_status;
  double timeout_takeover_request;
  bool use_takeover_request;
  bool use_parking_after_stopped;
  HazardLampPolicy turning_hazard_on{};
};

class EmergencyHandler : public rclcpp::Node
{
public:
  EmergencyHandler();

private:
  // Subscribers
  rclcpp::Subscription<autoware_system_msgs::msg::HazardStatusStamped>::SharedPtr
    sub_hazard_status_stamped_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VehicleCommand>::SharedPtr
    sub_prev_control_command_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ControlMode>::SharedPtr sub_control_mode_;

  autoware_system_msgs::msg::HazardStatusStamped::ConstSharedPtr hazard_status_stamped_;
  autoware_control_msgs::msg::ControlCommand::ConstSharedPtr prev_control_command_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_;
  autoware_vehicle_msgs::msg::ControlMode::ConstSharedPtr control_mode_;

  void onHazardStatusStamped(
    const autoware_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg);
  // To be replaced by ControlCommand
  void onPrevControlCommand(const autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr msg);
  void onTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void onControlMode(const autoware_vehicle_msgs::msg::ControlMode::ConstSharedPtr msg);

  // Publisher
  rclcpp::Publisher<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr
    pub_control_command_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr pub_shift_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr pub_turn_signal_;
  rclcpp::Publisher<autoware_system_msgs::msg::EmergencyStateStamped>::SharedPtr
    pub_emergency_state_;

  autoware_vehicle_msgs::msg::TurnSignal createTurnSignalMsg();
  void publishControlCommands();

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  Param param_;

  bool isDataReady();
  void onTimer();

  // Heartbeat
  std::shared_ptr<HeaderlessHeartbeatChecker<autoware_system_msgs::msg::HazardStatusStamped>>
    heartbeat_hazard_status_;

  // Algorithm
  autoware_system_msgs::msg::EmergencyState::_state_type emergency_state_{
    autoware_system_msgs::msg::EmergencyState::NORMAL};
  rclcpp::Time takeover_requested_time_;

  void transitionTo(const int new_state);
  void updateEmergencyState();
  bool isStopped();
  bool isEmergency(const autoware_system_msgs::msg::HazardStatus & hazard_status);
  autoware_control_msgs::msg::ControlCommand selectAlternativeControlCommand();
};

#endif  // EMERGENCY_HANDLER__EMERGENCY_HANDLER_CORE_HPP_
