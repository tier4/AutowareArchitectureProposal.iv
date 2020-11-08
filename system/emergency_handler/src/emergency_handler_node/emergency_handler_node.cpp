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
  if (!isEmergency()) {
    is_emergency_ = false;
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
  if (!isDataReady()) {
    if ((ros::Time::now() - initialized_time_).toSec() > data_ready_timeout_) {
      ROS_WARN_THROTTLE(1.0, "input data is timeout");

      std_msgs::Bool is_emergency;
      is_emergency.data = true;
      pub_is_emergency_.publish(is_emergency);
    }

    return;
  }

  // Create timestamp
  const auto stamp = ros::Time::now();

  // Check if emergency
  {
    if (use_emergency_hold_) {
      if (isEmergency()) {
        is_emergency_ = true;
      }
    } else {
      is_emergency_ = isEmergency();
    }

    std_msgs::Bool msg;
    msg.data = is_emergency_;
    pub_is_emergency_.publish(msg);
  }

  // Select ControlCommand
  autoware_control_msgs::ControlCommandStamped emergency_control_command;
  emergency_control_command.header.stamp = stamp;
  emergency_control_command.control = selectAlternativeControlCommand();
  pub_control_command_.publish(emergency_control_command);

  // TurnSignal
  {
    autoware_vehicle_msgs::TurnSignal turn_signal;
    turn_signal.header.stamp = stamp;
    turn_signal.data = autoware_vehicle_msgs::TurnSignal::HAZARD;
    pub_turn_signal_.publish(turn_signal);
  }

  // Shift
  if (use_parking_after_stopped_ && isStopped()) {
    autoware_vehicle_msgs::ShiftStamped shift;
    shift.header.stamp = stamp;
    shift.shift.data = autoware_vehicle_msgs::Shift::PARKING;
    pub_shift_.publish(shift);
  }
}

bool EmergencyHandlerNode::isStopped()
{
  constexpr auto th_stopped_velocity = 0.1;
  if (twist_->twist.linear.x < th_stopped_velocity) {
    return true;
  }

  return false;
}

bool EmergencyHandlerNode::isEmergency()
{
  // Check heartbeat timeout
  if (heartbeat_driving_capability_->isTimeout()) {
    ROS_WARN_THROTTLE(1.0, "heartbeat_driving_capability is timeout");
    return true;
  }

  if (heartbeat_is_state_timeout_->isTimeout()) {
    ROS_WARN_THROTTLE(1.0, "heartbeat_is_state_timeout is timeout");
    return true;
  }

  // Check state timeout
  if (is_state_timeout_->data) {
    ROS_WARN_THROTTLE(1.0, "state is timeout");
    return true;
  }

  using autoware_control_msgs::GateMode;
  using autoware_system_msgs::AutowareState;

  const auto is_in_target_state = (autoware_state_->state != AutowareState::InitializingVehicle) &&
                                  (autoware_state_->state != AutowareState::WaitingForRoute) &&
                                  (autoware_state_->state != AutowareState::Planning) &&
                                  (autoware_state_->state != AutowareState::Finalizing);

  if (!is_in_target_state) {
    return false;
  }

  if (current_gate_mode_->data == GateMode::AUTO) {
    if (!driving_capability_->autonomous_driving) {
      ROS_WARN_THROTTLE(1.0, "autonomous_driving is failed");
      return true;
    }
  }

  if (current_gate_mode_->data == GateMode::REMOTE) {
    if (!driving_capability_->remote_control) {
      ROS_WARN_THROTTLE(1.0, "remote_control is failed");
      return true;
    }
  }

  if (!driving_capability_->manual_driving) {
    ROS_WARN_THROTTLE(1.0, "manual_driving is failed");
    return true;
  }

  /* Currently not supported */
  // if (!driving_capability_->safe_stop) {
  //   ROS_WARN_THROTTLE(1.0, "safe_stop is failed");
  //   return true;
  // }
  /* Currently not supported */

  if (!driving_capability_->emergency_stop) {
    ROS_WARN_THROTTLE(1.0, "emergency_stop is failed");
    return true;
  }

  return false;
}

autoware_control_msgs::ControlCommand EmergencyHandlerNode::selectAlternativeControlCommand()
{
  /* Currently not supported */
  // Safe Stop
  // if (driving_capability_->safe_stop) {
  // TODO: Add safe_stop planner
  // }
  /* Currently not supported */

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

EmergencyHandlerNode::EmergencyHandlerNode()
{
  // Parameter
  private_nh_.param("update_rate", update_rate_, 10.0);
  private_nh_.param("timeout_driving_capability", timeout_driving_capability_, 0.5);
  private_nh_.param("timeout_is_state_timeout", timeout_is_state_timeout_, 0.5);
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

  // Initialize
  twist_ = geometry_msgs::TwistStamped::ConstPtr(new geometry_msgs::TwistStamped);
  prev_control_command_ =
    autoware_control_msgs::ControlCommand::ConstPtr(new autoware_control_msgs::ControlCommand);

  // Timer
  initialized_time_ = ros::Time::now();
  timer_ = private_nh_.createTimer(ros::Rate(update_rate_), &EmergencyHandlerNode::onTimer, this);
}
