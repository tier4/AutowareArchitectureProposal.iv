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

#include "remote_cmd_converter/node.hpp"

using std::placeholders::_1;

RemoteCmdConverter::RemoteCmdConverter()
: Node("remote_cmd_converter"),
  updater_(this),
  accel_map_(get_logger()),
  brake_map_(get_logger())
{
  pub_cmd_ = create_publisher<autoware_control_msgs::msg::ControlCommandStamped>(
    "out/control_cmd", rclcpp::QoS{1});
  pub_current_cmd_ = create_publisher<autoware_vehicle_msgs::msg::RawControlCommandStamped>(
    "out/latest_raw_control_cmd", rclcpp::QoS{1});

  sub_velocity_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "in/twist", 1, std::bind(&RemoteCmdConverter::onVelocity, this, _1));
  sub_control_cmd_ = create_subscription<autoware_vehicle_msgs::msg::RawControlCommandStamped>(
    "in/raw_control_cmd", 1, std::bind(&RemoteCmdConverter::onRemoteCmd, this, _1));
  sub_shift_cmd_ = create_subscription<autoware_vehicle_msgs::msg::ShiftStamped>(
    "in/shift_cmd", 1, std::bind(&RemoteCmdConverter::onShiftCmd, this, _1));
  sub_gate_mode_ = create_subscription<autoware_control_msgs::msg::GateMode>(
    "in/current_gate_mode", 1, std::bind(&RemoteCmdConverter::onGateMode, this, _1));
  sub_emergency_ = create_subscription<autoware_control_msgs::msg::EmergencyMode>(
    "in/emergency", 1, std::bind(&RemoteCmdConverter::onEmergency, this, _1));

  // Parameter
  ref_vel_gain_ = declare_parameter("ref_vel_gain", 3.0);

  // Parameter for Hz check
  time_threshold_ = declare_parameter("time_threshold", 3.0);
  const double timer_rate = declare_parameter("timer_rate", 10.0);

  auto timer_callback = std::bind(&RemoteCmdConverter::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / timer_rate));
  rate_check_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(rate_check_timer_, nullptr);

  // Parameter for accel/brake map
  const std::string csv_path_accel_map = declare_parameter("csv_path_accel_map").get<std::string>();
  const std::string csv_path_brake_map = declare_parameter("csv_path_brake_map").get<std::string>();
  acc_map_initialized_ = true;
  if (!accel_map_.readAccelMapFromCSV(csv_path_accel_map)) {
    RCLCPP_ERROR(
      get_logger(), "Cannot read accelmap. csv path = %s. stop calculation.",
      csv_path_accel_map.c_str());
    acc_map_initialized_ = false;
  }
  if (!brake_map_.readBrakeMapFromCSV(csv_path_brake_map)) {
    RCLCPP_ERROR(
      get_logger(), "Cannot read brakemap. csv path = %s. stop calculation.",
      csv_path_brake_map.c_str());
    acc_map_initialized_ = false;
  }

  // Diagnostics
  updater_.setHardwareID("remote_cmd_converter");
  updater_.add("remote_control_topic_status", this, &RemoteCmdConverter::checkTopicStatus);
  updater_.add("emergency_stop_operation", this, &RemoteCmdConverter::checkEmergency);

  // Set default values
  current_shift_cmd_ = std::make_shared<autoware_vehicle_msgs::msg::ShiftStamped>();
}

void RemoteCmdConverter::onTimer() { updater_.force_update(); }

void RemoteCmdConverter::onVelocity(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  current_velocity_ptr_ = std::make_shared<double>(msg->twist.linear.x);
}

void RemoteCmdConverter::onShiftCmd(
  const autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg)
{
  current_shift_cmd_ = msg;
}

void RemoteCmdConverter::onEmergency(
  const autoware_control_msgs::msg::EmergencyMode::ConstSharedPtr msg)
{
  current_emergency_cmd_ = msg->is_emergency;
  updater_.force_update();
}

void RemoteCmdConverter::onRemoteCmd(
  const autoware_vehicle_msgs::msg::RawControlCommandStamped::ConstSharedPtr raw_control_cmd_ptr)
{
  // Echo back received command
  {
    auto current_remote_cmd = *raw_control_cmd_ptr;
    current_remote_cmd.header.stamp = this->now();
    pub_current_cmd_->publish(current_remote_cmd);
  }

  // Save received time for rate check
  latest_cmd_received_time_ = std::make_shared<rclcpp::Time>(this->now());

  // Wait for input data
  if (!current_velocity_ptr_ || !acc_map_initialized_) {
    return;
  }

  // Calculate reference velocity and acceleration
  const double sign = getShiftVelocitySign(*current_shift_cmd_);
  const double ref_acceleration =
    calculateAcc(raw_control_cmd_ptr->control, std::fabs(*current_velocity_ptr_));

  double ref_velocity = *current_velocity_ptr_ + ref_acceleration * ref_vel_gain_ * sign;
  if (current_shift_cmd_->shift.data == autoware_vehicle_msgs::msg::Shift::REVERSE) {
    ref_velocity = std::min(0.0, ref_velocity);
  } else {
    ref_velocity = std::max(0.0, ref_velocity);
  }

  // Publish ControlCommand
  autoware_control_msgs::msg::ControlCommandStamped output;
  output.header = raw_control_cmd_ptr->header;
  output.control.steering_angle = raw_control_cmd_ptr->control.steering_angle;
  output.control.steering_angle_velocity = raw_control_cmd_ptr->control.steering_angle_velocity;
  output.control.velocity = ref_velocity;
  output.control.acceleration = ref_acceleration;

  pub_cmd_->publish(output);
}

double RemoteCmdConverter::calculateAcc(
  const autoware_vehicle_msgs::msg::RawControlCommand & cmd, const double vel)
{
  const double desired_throttle = cmd.throttle;
  const double desired_brake = cmd.brake;
  const double desired_pedal = desired_throttle - desired_brake;

  double ref_acceleration = 0.0;
  if (desired_pedal > 0.0) {
    accel_map_.getAcceleration(desired_pedal, vel, ref_acceleration);
  } else {
    brake_map_.getAcceleration(-desired_pedal, vel, ref_acceleration);
  }

  return ref_acceleration;
}

double RemoteCmdConverter::getShiftVelocitySign(
  const autoware_vehicle_msgs::msg::ShiftStamped & cmd)
{
  using autoware_vehicle_msgs::msg::Shift;

  if (cmd.shift.data == Shift::DRIVE) return 1.0;
  if (cmd.shift.data == Shift::LOW) return 1.0;
  if (cmd.shift.data == Shift::REVERSE) return -1.0;

  return 0.0;
}

void RemoteCmdConverter::checkTopicStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  DiagnosticStatus status;
  if (!checkRemoteTopicRate()) {
    status.level = DiagnosticStatus::ERROR;
    status.message = "low topic rate for remote vehicle_cmd";
  } else {
    status.level = DiagnosticStatus::OK;
  }

  stat.summary(status.level, status.message);
}

void RemoteCmdConverter::checkEmergency(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  DiagnosticStatus status;
  if (current_emergency_cmd_) {
    status.level = DiagnosticStatus::ERROR;
    status.message = "remote emergency requested";
  } else {
    status.level = DiagnosticStatus::OK;
  }

  stat.summary(status.level, status.message);
}

void RemoteCmdConverter::onGateMode(const autoware_control_msgs::msg::GateMode::ConstSharedPtr msg)
{
  current_gate_mode_ = msg;
}

bool RemoteCmdConverter::checkRemoteTopicRate()
{
  if (!latest_cmd_received_time_ || !current_gate_mode_) return true;

  if (current_gate_mode_->data == autoware_control_msgs::msg::GateMode::REMOTE) {
    const auto duration = (this->now() - *latest_cmd_received_time_);
    if (duration.seconds() > time_threshold_) return false;
  } else {
    latest_cmd_received_time_ = nullptr;  // reset;
  }

  return true;
}
