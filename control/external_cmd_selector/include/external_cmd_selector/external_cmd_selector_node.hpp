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

#ifndef EXTERNAL_CMD_SELECTOR__EXTERNAL_CMD_SELECTOR_NODE_HPP_
#define EXTERNAL_CMD_SELECTOR__EXTERNAL_CMD_SELECTOR_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "autoware_control_msgs/srv/external_command_select.hpp"
#include "autoware_control_msgs/msg/external_command_selector_mode.hpp"
#include "autoware_control_msgs/msg/emergency_mode.hpp"
#include "autoware_vehicle_msgs/msg/external_control_command_stamped.hpp"
#include "autoware_vehicle_msgs/msg/shift_stamped.hpp"
#include "autoware_vehicle_msgs/msg/turn_signal.hpp"

class ExternalCmdSelector : public rclcpp::Node
{
public:
  explicit ExternalCmdSelector(const rclcpp::NodeOptions & node_options);

private:
  using ExternalCommandSelect = autoware_control_msgs::srv::ExternalCommandSelect;
  using ExternalCommandSelectorMode = autoware_control_msgs::msg::ExternalCommandSelectorMode;
  using ExternalControlCommand = autoware_vehicle_msgs::msg::ExternalControlCommandStamped;
  using ShiftCommand = autoware_vehicle_msgs::msg::ShiftStamped;
  using TurnSignalCommand = autoware_vehicle_msgs::msg::TurnSignal;
  using EmergencyMode = autoware_control_msgs::msg::EmergencyMode;

  // CallbackGroups
  rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;
  rclcpp::CallbackGroup::SharedPtr callback_group_services_;

  // Publisher
  rclcpp::Publisher<ExternalCommandSelectorMode>::SharedPtr pub_current_selector_mode_;
  rclcpp::Publisher<ExternalControlCommand>::SharedPtr pub_control_cmd_;
  rclcpp::Publisher<ShiftCommand>::SharedPtr pub_shift_cmd_;
  rclcpp::Publisher<TurnSignalCommand>::SharedPtr pub_turn_signal_cmd_;
  rclcpp::Publisher<EmergencyMode>::SharedPtr pub_heartbeat_;

  // Subscriber
  rclcpp::Subscription<ExternalControlCommand>::SharedPtr sub_local_control_cmd_;
  rclcpp::Subscription<ShiftCommand>::SharedPtr sub_local_shift_cmd_;
  rclcpp::Subscription<TurnSignalCommand>::SharedPtr sub_local_turn_signal_cmd_;
  rclcpp::Subscription<EmergencyMode>::SharedPtr sub_local_heartbeat_;

  rclcpp::Subscription<ExternalControlCommand>::SharedPtr sub_remote_control_cmd_;
  rclcpp::Subscription<ShiftCommand>::SharedPtr sub_remote_shift_cmd_;
  rclcpp::Subscription<TurnSignalCommand>::SharedPtr sub_remote_turn_signal_cmd_;
  rclcpp::Subscription<EmergencyMode>::SharedPtr sub_remote_heartbeat_;

  void onLocalControlCmd(const ExternalControlCommand::ConstSharedPtr msg);
  void onLocalShiftCmd(const ShiftCommand::ConstSharedPtr msg);
  void onLocalTurnSignalCmd(const TurnSignalCommand::ConstSharedPtr msg);
  void onLocalHeartbeat(const EmergencyMode::ConstSharedPtr msg);

  void onRemoteControlCmd(const ExternalControlCommand::ConstSharedPtr msg);
  void onRemoteShiftCmd(const ShiftCommand::ConstSharedPtr msg);
  void onRemoteTurnSignalCmd(const TurnSignalCommand::ConstSharedPtr msg);
  void onRemoteHeartbeat(const EmergencyMode::ConstSharedPtr msg);

  // Service
  rclcpp::Service<ExternalCommandSelect>::SharedPtr srv_select_external_command_;
  ExternalCommandSelectorMode current_selector_mode_;

  bool onSelectExternalCommandService(
    const ExternalCommandSelect::Request::SharedPtr req,
    const ExternalCommandSelect::Response::SharedPtr res);

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimer();
};

#endif  // EXTERNAL_CMD_SELECTOR__EXTERNAL_CMD_SELECTOR_NODE_HPP_
