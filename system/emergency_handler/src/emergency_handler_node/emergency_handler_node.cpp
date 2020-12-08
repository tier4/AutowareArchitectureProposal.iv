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

#include <emergency_handler/emergency_handler_node.h>

namespace
{
diagnostic_msgs::DiagnosticStatus createDiagnosticStatus(
  const int level, const std::string & name, const std::string & message)
{
  diagnostic_msgs::DiagnosticStatus diag;

  diag.level = level;
  diag.name = name;
  diag.message = message;
  diag.hardware_id = "emergency_handler";

  return diag;
}

diagnostic_msgs::DiagnosticArray convertHazardStatusToDiagnosticArray(
  const autoware_system_msgs::HazardStatus & hazard_status)
{
  using diagnostic_msgs::DiagnosticStatus;

  diagnostic_msgs::DiagnosticArray diag_array;
  diag_array.header.stamp = ros::Time::now();

  const auto decorateDiag = [](const auto & hazard_diag, const std::string & label) {
    auto diag = hazard_diag;

    diag.message = label + diag.message;

    return diag;
  };

  for (const auto & hazard_diag : hazard_status.diagnostics_nf) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[No Fault]"));
  }
  for (const auto & hazard_diag : hazard_status.diagnostics_sf) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[Safe Fault]"));
  }
  for (const auto & hazard_diag : hazard_status.diagnostics_lf) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[Latent Fault]"));
  }
  for (const auto & hazard_diag : hazard_status.diagnostics_spf) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[Single Point Fault]"));
  }

  return diag_array;
}
}  // namespace

EmergencyHandlerNode::EmergencyHandlerNode()
{
  // Parameter
  private_nh_.param("update_rate", update_rate_, 10.0);
  private_nh_.param("timeout_driving_capability", timeout_driving_capability_, 0.5);
  private_nh_.param("timeout_is_state_timeout", timeout_is_state_timeout_, 0.5);
  private_nh_.param("emergency_hazard_level", emergency_hazard_level_, 2);
  private_nh_.param("use_emergency_hold", use_emergency_hold_, false);
  private_nh_.param("data_ready_timeout", data_ready_timeout_, 30.0);
  private_nh_.param("use_parking_after_stopped", use_parking_after_stopped_, false);

  // Subscriber
  sub_autoware_state_ =
    private_nh_.subscribe("input/autoware_state", 1, &EmergencyHandlerNode::onAutowareState, this);
  sub_driving_capability_ = private_nh_.subscribe(
    "input/driving_capability", 1, &EmergencyHandlerNode::onDrivingCapability, this);
  sub_prev_control_command_ = private_nh_.subscribe(
    "input/prev_control_command", 1, &EmergencyHandlerNode::onPrevControlCommand, this);
  sub_current_gate_mode_ = private_nh_.subscribe(
    "input/current_gate_mode", 1, &EmergencyHandlerNode::onCurrentGateMode, this);
  sub_twist_ = private_nh_.subscribe("input/twist", 1, &EmergencyHandlerNode::onTwist, this);
  sub_is_state_timeout_ = private_nh_.subscribe(
    "input/is_state_timeout", 1, &EmergencyHandlerNode::onIsStateTimeout, this);

  // Heartbeat
  heartbeat_driving_capability_ =
    std::make_shared<HeaderlessHeartbeatChecker<autoware_system_msgs::DrivingCapability>>(
      "input/driving_capability", timeout_driving_capability_);
  heartbeat_is_state_timeout_ = std::make_shared<HeaderlessHeartbeatChecker<std_msgs::Bool>>(
    "input/is_state_timeout", timeout_is_state_timeout_);

  // Service
  srv_clear_emergency_ = private_nh_.advertiseService(
    "service/clear_emergency", &EmergencyHandlerNode::onClearEmergencyService, this);

  // Publisher
  pub_control_command_ = private_nh_.advertise<autoware_control_msgs::ControlCommandStamped>(
    "output/control_command", 1);
  pub_shift_ = private_nh_.advertise<autoware_vehicle_msgs::ShiftStamped>("output/shift", 1);
  pub_turn_signal_ =
    private_nh_.advertise<autoware_vehicle_msgs::TurnSignal>("output/turn_signal", 1);
  pub_is_emergency_ = private_nh_.advertise<std_msgs::Bool>("output/is_emergency", 1);
  pub_hazard_status_ =
    private_nh_.advertise<autoware_system_msgs::HazardStatusStamped>("output/hazard_status", 1);
  pub_diagnostics_err_ =
    private_nh_.advertise<diagnostic_msgs::DiagnosticArray>("output/diagnostics_err", 1);

  // Initialize
  twist_ = geometry_msgs::TwistStamped::ConstPtr(new geometry_msgs::TwistStamped);
  prev_control_command_ =
    autoware_control_msgs::ControlCommand::ConstPtr(new autoware_control_msgs::ControlCommand);

  // Timer
  initialized_time_ = ros::Time::now();
  timer_ = private_nh_.createTimer(ros::Rate(update_rate_), &EmergencyHandlerNode::onTimer, this);
}

void EmergencyHandlerNode::onAutowareState(
  const autoware_system_msgs::AutowareState::ConstPtr & msg)
{
  autoware_state_ = msg;
}

void EmergencyHandlerNode::onDrivingCapability(
  const autoware_system_msgs::DrivingCapability::ConstPtr & msg)
{
  driving_capability_ = msg;
}

// To be replaced by ControlCommand
void EmergencyHandlerNode::onPrevControlCommand(
  const autoware_vehicle_msgs::VehicleCommand::ConstPtr & msg)
{
  const auto control_command = new autoware_control_msgs::ControlCommand();
  *control_command = msg->control;
  prev_control_command_ = autoware_control_msgs::ControlCommand::ConstPtr(control_command);
}

void EmergencyHandlerNode::onCurrentGateMode(const autoware_control_msgs::GateMode::ConstPtr & msg)
{
  current_gate_mode_ = msg;
}

void EmergencyHandlerNode::onTwist(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
  twist_ = msg;
}

bool EmergencyHandlerNode::onClearEmergencyService(
  std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res)
{
  const auto hazard_status = judgeHazardStatus();
  if (!isEmergency(hazard_status)) {
    is_emergency_ = false;
    hazard_status_ = hazard_status;

    res.success = true;
    res.message = "Emergency state was cleared.";
  } else {
    res.success = false;
    res.message = "There are still errors, can't clear emergency state.";
  }

  return true;
}

void EmergencyHandlerNode::onIsStateTimeout(const std_msgs::Bool::ConstPtr & msg)
{
  is_state_timeout_ = msg;
}

void EmergencyHandlerNode::publishHazardStatus(
  const autoware_system_msgs::HazardStatus & hazard_status)
{
  // Create msg of is_emergency
  std_msgs::Bool is_emergency;
  is_emergency.data = isEmergency(hazard_status);

  // Create msg of hazard_status
  autoware_system_msgs::HazardStatusStamped hazard_status_stamped;
  hazard_status_stamped.header.stamp = ros::Time::now();
  hazard_status_stamped.status = hazard_status;

  // Publish data
  pub_is_emergency_.publish(is_emergency);
  pub_hazard_status_.publish(hazard_status_stamped);
  pub_diagnostics_err_.publish(convertHazardStatusToDiagnosticArray(hazard_status_stamped.status));
}

void EmergencyHandlerNode::publishControlCommands()
{
  // Create timestamp
  const auto stamp = ros::Time::now();

  // Publish ControlCommand
  {
    autoware_control_msgs::ControlCommandStamped msg;
    msg.header.stamp = stamp;
    msg.control = selectAlternativeControlCommand();
    pub_control_command_.publish(msg);
  }

  // Publish TurnSignal
  {
    autoware_vehicle_msgs::TurnSignal msg;
    msg.header.stamp = stamp;
    msg.data = autoware_vehicle_msgs::TurnSignal::HAZARD;
    pub_turn_signal_.publish(msg);
  }

  // Publish Shift
  if (use_parking_after_stopped_ && isStopped()) {
    autoware_vehicle_msgs::ShiftStamped msg;
    msg.header.stamp = stamp;
    msg.shift.data = autoware_vehicle_msgs::Shift::PARKING;
    pub_shift_.publish(msg);
  }
}

bool EmergencyHandlerNode::isDataReady()
{
  if (!autoware_state_) {
    ROS_INFO_THROTTLE(5.0, "waiting for autoware_state msg...");
    return false;
  }

  if (!driving_capability_) {
    ROS_INFO_THROTTLE(5.0, "waiting for driving_capability msg...");
    return false;
  }

  if (!current_gate_mode_) {
    ROS_INFO_THROTTLE(5.0, "waiting for current_gate_mode msg...");
    return false;
  }

  if (!is_state_timeout_) {
    ROS_INFO_THROTTLE(5.0, "waiting for is_state_timeout msg...");
    return false;
  }

  return true;
}

void EmergencyHandlerNode::onTimer(const ros::TimerEvent & event)
{
  // Wait for data ready
  if (!isDataReady()) {
    // Check timeout
    if ((ros::Time::now() - initialized_time_).toSec() > data_ready_timeout_) {
      ROS_WARN_THROTTLE(1.0, "input data is timeout");

      autoware_system_msgs::HazardStatus hazard_status;
      hazard_status.level = autoware_system_msgs::HazardStatus::SINGLE_POINT_FAULT;

      diagnostic_msgs::DiagnosticStatus diag;
      diag.name = "emergency_handler/input_data_timeout";
      diag.hardware_id = "emergency_handler";
      diag.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      hazard_status.diagnostics_spf.push_back(diag);

      publishHazardStatus(hazard_status);
    }

    return;
  }

  // Check if emergency
  if (use_emergency_hold_) {
    if (!is_emergency_) {
      // Update only when it is not emergency
      hazard_status_ = judgeHazardStatus();
      is_emergency_ = isEmergency(hazard_status_);
    }
  } else {
    // Update always
    hazard_status_ = judgeHazardStatus();
    is_emergency_ = isEmergency(hazard_status_);
  }

  // Publish data
  publishHazardStatus(hazard_status_);
  publishControlCommands();
}

bool EmergencyHandlerNode::isStopped()
{
  constexpr auto th_stopped_velocity = 0.001;
  if (twist_->twist.linear.x < th_stopped_velocity) {
    return true;
  }

  return false;
}

bool EmergencyHandlerNode::isEmergency(const autoware_system_msgs::HazardStatus & hazard_status)
{
  return hazard_status.level >= emergency_hazard_level_;
}

autoware_system_msgs::HazardStatus EmergencyHandlerNode::judgeHazardStatus()
{
  // Get hazard status
  auto hazard_status = current_gate_mode_->data == autoware_control_msgs::GateMode::AUTO
                         ? driving_capability_->autonomous_driving
                         : driving_capability_->remote_control;

  // Ignore initializing and finalizing state
  {
    using autoware_control_msgs::GateMode;
    using autoware_system_msgs::AutowareState;
    using autoware_system_msgs::HazardStatus;

    const auto is_in_auto_ignore_state =
      (autoware_state_->state == AutowareState::InitializingVehicle) ||
      (autoware_state_->state == AutowareState::WaitingForRoute) ||
      (autoware_state_->state == AutowareState::Planning) ||
      (autoware_state_->state == AutowareState::Finalizing);

    if (current_gate_mode_->data == GateMode::AUTO && is_in_auto_ignore_state) {
      hazard_status.level = HazardStatus::NO_FAULT;
    }

    const auto is_in_remote_ignore_state =
      (autoware_state_->state == AutowareState::InitializingVehicle) ||
      (autoware_state_->state == AutowareState::Finalizing);

    if (current_gate_mode_->data == GateMode::REMOTE && is_in_remote_ignore_state) {
      hazard_status.level = HazardStatus::NO_FAULT;
    }
  }

  // Check timeout
  {
    using autoware_system_msgs::HazardStatus;
    using diagnostic_msgs::DiagnosticStatus;

    if (heartbeat_driving_capability_->isTimeout()) {
      ROS_WARN_THROTTLE(1.0, "heartbeat_driving_capability is timeout");
      hazard_status.level = HazardStatus::SINGLE_POINT_FAULT;
      hazard_status.diagnostics_spf.push_back(createDiagnosticStatus(
        DiagnosticStatus::ERROR, "emergency_handler/heartbeat_timeout",
        "heartbeat_driving_capability is timeout"));
    }

    if (heartbeat_is_state_timeout_->isTimeout()) {
      ROS_WARN_THROTTLE(1.0, "heartbeat_is_state_timeout is timeout");
      hazard_status.level = HazardStatus::SINGLE_POINT_FAULT;
      hazard_status.diagnostics_spf.push_back(createDiagnosticStatus(
        DiagnosticStatus::ERROR, "emergency_handler/heartbeat_timeout",
        "heartbeat_is_state_timeout is timeout"));
    }

    if (is_state_timeout_->data) {
      ROS_WARN_THROTTLE(1.0, "state is timeout");
      hazard_status.level = HazardStatus::SINGLE_POINT_FAULT;
      hazard_status.diagnostics_spf.push_back(createDiagnosticStatus(
        DiagnosticStatus::ERROR, "emergency_handler/state_timeout", "state is timeout"));
    }
  }

  return hazard_status;
}

autoware_control_msgs::ControlCommand EmergencyHandlerNode::selectAlternativeControlCommand()
{
  // TODO: Add safe_stop planner

  // Emergency Stop
  {
    autoware_control_msgs::ControlCommand emergency_stop_cmd;
    emergency_stop_cmd.steering_angle = prev_control_command_->steering_angle;
    emergency_stop_cmd.steering_angle_velocity = prev_control_command_->steering_angle_velocity;
    emergency_stop_cmd.velocity = 0.0;
    emergency_stop_cmd.acceleration = -2.5;

    return emergency_stop_cmd;
  }
}
