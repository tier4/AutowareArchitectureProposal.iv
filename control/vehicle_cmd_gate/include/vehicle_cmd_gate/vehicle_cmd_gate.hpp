// Copyright 2015-2019 Autoware Foundation
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

#ifndef VEHICLE_CMD_GATE__VEHICLE_CMD_GATE_HPP_
#define VEHICLE_CMD_GATE__VEHICLE_CMD_GATE_HPP_

#include <memory>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_control_msgs/msg/control_command_stamped.hpp"
#include "autoware_control_msgs/msg/emergency_mode.hpp"
#include "autoware_control_msgs/msg/gate_mode.hpp"
#include "autoware_vehicle_msgs/msg/engage.hpp"
#include "autoware_vehicle_msgs/msg/shift_stamped.hpp"
#include "autoware_vehicle_msgs/msg/steering.hpp"
#include "autoware_vehicle_msgs/msg/turn_signal.hpp"
#include "autoware_vehicle_msgs/msg/vehicle_command.hpp"

#include "vehicle_cmd_gate/vehicle_cmd_filter.hpp"
#include "std_srvs/srv/trigger.hpp"

struct Commands
{
  autoware_control_msgs::msg::ControlCommandStamped control;
  autoware_vehicle_msgs::msg::TurnSignal turn_signal;
  autoware_vehicle_msgs::msg::ShiftStamped shift;
};

class VehicleCmdGate : public rclcpp::Node
{
public:
  VehicleCmdGate();

private:
  // Publisher
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
  rclcpp::Publisher<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr control_cmd_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr shift_cmd_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr turn_signal_cmd_pub_;
  rclcpp::Publisher<autoware_control_msgs::msg::GateMode>::SharedPtr gate_mode_pub_;

  // Subscription
  rclcpp::Subscription<autoware_control_msgs::msg::EmergencyMode>::SharedPtr system_emergency_sub_;
  rclcpp::Subscription<autoware_control_msgs::msg::EmergencyMode>::SharedPtr
    external_emergency_stop_sub_;
  rclcpp::Subscription<autoware_control_msgs::msg::GateMode>::SharedPtr gate_mode_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Engage>::SharedPtr engage_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Steering>::SharedPtr steer_sub_;

  void onGateMode(autoware_control_msgs::msg::GateMode::ConstSharedPtr msg);
  void onSystemEmergency(autoware_control_msgs::msg::EmergencyMode::ConstSharedPtr msg);
  void onExternalEmergencyStop(autoware_control_msgs::msg::EmergencyMode::ConstSharedPtr msg);
  void onEngage(autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg);
  void onSteering(autoware_vehicle_msgs::msg::Steering::ConstSharedPtr msg);

  bool is_engaged_;
  bool is_system_emergency_ = false;
  bool is_external_emergency_stop_ = false;
  double current_steer_ = 0;
  autoware_control_msgs::msg::GateMode current_gate_mode_;

  // Heartbeat
  std::shared_ptr<rclcpp::Time> system_emergency_heartbeat_received_time_;
  bool is_system_emergency_heartbeat_timeout_ = false;
  std::shared_ptr<rclcpp::Time> external_emergency_stop_heartbeat_received_time_;
  bool is_external_emergency_stop_heartbeat_timeout_ = false;
  bool isHeartbeatTimeout(
    const std::shared_ptr<rclcpp::Time> & heartbeat_received_time, const double timeout);

  // Subscriber for auto
  Commands auto_commands_;
  rclcpp::Subscription<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr
    auto_control_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr auto_turn_signal_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr auto_shift_cmd_sub_;
  void onAutoCtrlCmd(autoware_control_msgs::msg::ControlCommandStamped::ConstSharedPtr msg);
  void onAutoTurnSignalCmd(autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg);
  void onAutoShiftCmd(autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg);

  // Subscription for remote
  Commands remote_commands_;
  rclcpp::Subscription<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr
    remote_control_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr
    remote_turn_signal_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr remote_shift_cmd_sub_;
  void onRemoteCtrlCmd(autoware_control_msgs::msg::ControlCommandStamped::ConstSharedPtr msg);
  void onRemoteTurnSignalCmd(autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg);
  void onRemoteShiftCmd(autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg);

  // Subscription for emergency
  Commands emergency_commands_;
  rclcpp::Subscription<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr
    emergency_control_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr
    emergency_turn_signal_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr
    emergency_shift_cmd_sub_;
  void onEmergencyCtrlCmd(autoware_control_msgs::msg::ControlCommandStamped::ConstSharedPtr msg);
  void onEmergencyTurnSignalCmd(autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg);
  void onEmergencyShiftCmd(autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg);

  // Parameter
  double update_period_;
  bool use_emergency_handling_;
  bool use_external_emergency_stop_;
  double system_emergency_heartbeat_timeout_;
  double external_emergency_stop_heartbeat_timeout_;

  // Service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_external_emergency_stop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_external_emergency_stop_;
  bool onExternalEmergencyStopService(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  bool onClearExternalEmergencyStopService(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // Timer / Event
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimer();
  void publishControlCommands(const Commands & input_msg);
  void publishEmergencyStopControlCommands();

  // Diagnostics Updater
  diagnostic_updater::Updater updater_;

  void checkExternalEmergencyStop(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Algorithm
  autoware_control_msgs::msg::ControlCommand prev_control_cmd_;
  autoware_control_msgs::msg::ControlCommand createStopControlCmd() const;
  autoware_control_msgs::msg::ControlCommand createEmergencyStopControlCmd() const;

  std::shared_ptr<rclcpp::Time> prev_time_;
  double getDt();

  VehicleCmdFilter filter_;
  autoware_control_msgs::msg::ControlCommand filterControlCommand(
    const autoware_control_msgs::msg::ControlCommand & msg);
};

#endif  // VEHICLE_CMD_GATE__VEHICLE_CMD_GATE_HPP_
