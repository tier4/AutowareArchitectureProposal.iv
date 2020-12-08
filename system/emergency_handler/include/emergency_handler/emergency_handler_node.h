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

#pragma once

#include <string>

#include <ros/ros.h>

#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_control_msgs/GateMode.h>
#include <autoware_system_msgs/AutowareState.h>
#include <autoware_system_msgs/DrivingCapability.h>
#include <autoware_system_msgs/HazardStatusStamped.h>
#include <autoware_vehicle_msgs/ShiftStamped.h>
#include <autoware_vehicle_msgs/TurnSignal.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include <emergency_handler/util/heartbeat_checker.h>

// tmp
#include <autoware_vehicle_msgs/VehicleCommand.h>

class EmergencyHandlerNode
{
public:
  EmergencyHandlerNode();

private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  // Parameter
  double update_rate_;
  double data_ready_timeout_;
  double timeout_driving_capability_;
  double timeout_is_state_timeout_;
  int emergency_hazard_level_;
  bool use_emergency_hold_;
  bool use_parking_after_stopped_;

  // Subscriber
  ros::Subscriber sub_autoware_state_;
  ros::Subscriber sub_driving_capability_;
  ros::Subscriber sub_prev_control_command_;
  ros::Subscriber sub_current_gate_mode_;
  ros::Subscriber sub_twist_;
  ros::Subscriber sub_is_state_timeout_;

  autoware_system_msgs::AutowareState::ConstPtr autoware_state_;
  autoware_system_msgs::DrivingCapability::ConstPtr driving_capability_;
  autoware_control_msgs::ControlCommand::ConstPtr prev_control_command_;
  autoware_control_msgs::GateMode::ConstPtr current_gate_mode_;
  geometry_msgs::TwistStamped::ConstPtr twist_;
  std_msgs::Bool::ConstPtr is_state_timeout_;

  void onAutowareState(const autoware_system_msgs::AutowareState::ConstPtr & msg);
  void onDrivingCapability(const autoware_system_msgs::DrivingCapability::ConstPtr & msg);
  // To be replaced by ControlCommand
  void onPrevControlCommand(const autoware_vehicle_msgs::VehicleCommand::ConstPtr & msg);
  void onCurrentGateMode(const autoware_control_msgs::GateMode::ConstPtr & msg);
  void onTwist(const geometry_msgs::TwistStamped::ConstPtr & msg);
  void onIsStateTimeout(const std_msgs::Bool::ConstPtr & msg);

  // Service
  ros::ServiceServer srv_clear_emergency_;

  bool onClearEmergencyService(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);

  // Publisher
  ros::Publisher pub_control_command_;
  ros::Publisher pub_shift_;
  ros::Publisher pub_turn_signal_;
  ros::Publisher pub_is_emergency_;
  ros::Publisher pub_hazard_status_;
  ros::Publisher pub_diagnostics_err_;

  void publishHazardStatus(const autoware_system_msgs::HazardStatus & hazard_status);
  void publishControlCommands();

  // Timer
  ros::Timer timer_;

  bool isDataReady();
  void onTimer(const ros::TimerEvent & event);

  // Heartbeat
  ros::Time initialized_time_;
  std::shared_ptr<HeaderlessHeartbeatChecker<autoware_system_msgs::DrivingCapability>>
    heartbeat_driving_capability_;
  std::shared_ptr<HeaderlessHeartbeatChecker<std_msgs::Bool>> heartbeat_is_state_timeout_;

  // Algorithm
  bool is_emergency_ = false;
  autoware_system_msgs::HazardStatus hazard_status_;

  bool isStopped();
  bool isEmergency(const autoware_system_msgs::HazardStatus & hazard_status);
  autoware_system_msgs::HazardStatus judgeHazardStatus();
  autoware_control_msgs::ControlCommand selectAlternativeControlCommand();
};
