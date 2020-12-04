/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
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

#include "pacmod_interface/pacmod_interface.h"

namespace
{
template <class T>
T waitForParam(const ros::NodeHandle & nh, const std::string & key)
{
  T value;
  ros::Rate rate(1.0);

  while (ros::ok()) {
    const auto result = nh.getParam(key, value);
    if (result) {
      return value;
    }

    ROS_WARN("waiting for parameter `%s` ...", key.c_str());
    rate.sleep();
  }

  return {};
}
}  // namespace

PacmodInterface::PacmodInterface()
{
  /* setup parameters */
  private_nh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
  private_nh_.param<int>("command_timeout_ms", command_timeout_ms_, 1000);
  private_nh_.param<double>("loop_rate", loop_rate_, 30.0);

  /* parameters for vehicle specifications */
  tire_radius_ = waitForParam<double>(private_nh_, "/vehicle_info/wheel_radius");
  wheel_base_ = waitForParam<double>(private_nh_, "/vehicle_info/wheel_base");
  private_nh_.param<double>("steering_offset", steering_offset_, 0.0);
  private_nh_.param<bool>("enable_steering_rate_control", enable_steering_rate_control_, false);

  /* parameters for emergency stop */
  private_nh_.param<double>("emergency_brake", emergency_brake_, 0.7);
  private_nh_.param<double>("max_throttle", max_throttle_, 0.2);
  private_nh_.param<double>("max_brake", max_brake_, 0.8);

  private_nh_.param<double>("vgr_coef_a", vgr_coef_a_, 15.713);
  private_nh_.param<double>("vgr_coef_b", vgr_coef_b_, 0.053);
  private_nh_.param<double>("vgr_coef_c", vgr_coef_c_, 0.042);

  private_nh_.param<double>("accel_pedal_offset", accel_pedal_offset_, 0.0);
  private_nh_.param<double>("brake_pedal_offset", brake_pedal_offset_, 0.0);

  /* parameters for limitter */
  private_nh_.param<double>("max_steering_wheel", max_steering_wheel_, 2.7 * M_PI);
  private_nh_.param<double>("max_steering_wheel_rate", max_steering_wheel_rate_, 5.0);
  private_nh_.param<double>("min_steering_wheel_rate", min_steering_wheel_rate_, 0.5);
  private_nh_.param<double>("steering_wheel_rate_low_vel", steering_wheel_rate_low_vel_, 5.0);
  private_nh_.param<double>("steering_wheel_rate_stopped", steering_wheel_rate_stopped_, 5.0);
  private_nh_.param<double>("low_vel_thresh", low_vel_thresh_, 1.389);  // 5.0kmh

  /* parameters for turn signal recovery */
  private_nh_.param<double>("hazard_thresh_time", hazard_thresh_time_, 0.20);  //s
  /* initialize */
  prev_steer_cmd_.header.stamp = ros::Time::now();
  prev_steer_cmd_.command = 0.0;

  /* subscribers */
  // From autoware
  raw_vehicle_cmd_sub_ =
    nh_.subscribe("/vehicle/raw_vehicle_cmd", 1, &PacmodInterface::callbackVehicleCmd, this);
  turn_signal_cmd_sub_ =
    nh_.subscribe("/control/turn_signal_cmd", 1, &PacmodInterface::callbackTurnSignalCmd, this);
  engage_cmd_sub_ = nh_.subscribe("/vehicle/engage", 1, &PacmodInterface::callbackEngage, this);

  // From pacmod
  steer_wheel_rpt_sub_.subscribe(nh_, "/pacmod/parsed_tx/steer_rpt", 1);
  wheel_speed_rpt_sub_.subscribe(nh_, "/pacmod/parsed_tx/wheel_speed_rpt", 1);
  accel_rpt_sub_.subscribe(nh_, "/pacmod/parsed_tx/accel_rpt", 1);
  brake_rpt_sub_.subscribe(nh_, "/pacmod/parsed_tx/brake_rpt", 1);
  shift_rpt_sub_.subscribe(nh_, "/pacmod/parsed_tx/shift_rpt", 1);
  turn_rpt_sub_.subscribe(nh_, "/pacmod/parsed_tx/turn_rpt", 1);
  global_rpt_sub_.subscribe(nh_, "/pacmod/parsed_tx/global_rpt", 1);
  pacmod_feedbacks_sync_.connectInput(
    steer_wheel_rpt_sub_, wheel_speed_rpt_sub_, accel_rpt_sub_, brake_rpt_sub_, shift_rpt_sub_,
    turn_rpt_sub_, global_rpt_sub_);
  pacmod_feedbacks_sync_.registerCallback(
    boost::bind(&PacmodInterface::callbackPacmodRpt, this, _1, _2, _3, _4, _5, _6, _7));

  /* publisher */
  // To pacmod
  accel_cmd_pub_ = nh_.advertise<pacmod_msgs::SystemCmdFloat>("pacmod/as_rx/accel_cmd", 1);
  brake_cmd_pub_ = nh_.advertise<pacmod_msgs::SystemCmdFloat>("pacmod/as_rx/brake_cmd", 1);
  steer_cmd_pub_ = nh_.advertise<pacmod_msgs::SteerSystemCmd>("pacmod/as_rx/steer_cmd", 1);
  raw_steer_cmd_pub_ =
    nh_.advertise<pacmod_msgs::SteerSystemCmd>("pacmod/as_rx/raw_steer_cmd", 1);  //only for debug
  shift_cmd_pub_ = nh_.advertise<pacmod_msgs::SystemCmdInt>("pacmod/as_rx/shift_cmd", 1);
  turn_cmd_pub_ = nh_.advertise<pacmod_msgs::SystemCmdInt>("pacmod/as_rx/turn_cmd", 1);

  // To Autoware
  control_mode_pub_ =
    nh_.advertise<autoware_vehicle_msgs::ControlMode>("/vehicle/status/control_mode", 10);
  vehicle_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/vehicle/status/twist", 1);
  steering_status_pub_ =
    nh_.advertise<autoware_vehicle_msgs::Steering>("/vehicle/status/steering", 1);
  shift_status_pub_ =
    nh_.advertise<autoware_vehicle_msgs::ShiftStamped>("/vehicle/status/shift", 1);
  turn_signal_status_pub_ =
    nh_.advertise<autoware_vehicle_msgs::TurnSignal>("/vehicle/status/turn_signal", 1);
}

void PacmodInterface::run()
{
  ros::Rate rate(loop_rate_);

  while (ros::ok()) {
    ros::spinOnce();
    publishCommands();
    rate.sleep();
  }
}

void PacmodInterface::callbackVehicleCmd(
  const autoware_vehicle_msgs::RawVehicleCommand::ConstPtr & msg)
{
  vehicle_command_received_time_ = ros::Time::now();
  raw_vehicle_cmd_ptr_ = msg;
}

void PacmodInterface::callbackTurnSignalCmd(const autoware_vehicle_msgs::TurnSignal::ConstPtr & msg)
{
  turn_signal_cmd_ptr_ = msg;
}

void PacmodInterface::callbackEngage(const std_msgs::BoolConstPtr & msg)
{
  engage_cmd_ = msg->data;
  is_clear_override_needed_ = true;
}

void PacmodInterface::callbackPacmodRpt(
  const pacmod_msgs::SystemRptFloatConstPtr & steer_wheel_rpt,
  const pacmod_msgs::WheelSpeedRptConstPtr & wheel_speed_rpt,
  const pacmod_msgs::SystemRptFloatConstPtr & accel_rpt,
  const pacmod_msgs::SystemRptFloatConstPtr & brake_rpt,
  const pacmod_msgs::SystemRptIntConstPtr & shift_rpt,
  const pacmod_msgs::SystemRptIntConstPtr & turn_rpt,
  const pacmod_msgs::GlobalRptConstPtr & global_rpt)
{
  is_pacmod_rpt_received_ = true;

  steer_wheel_rpt_ptr_ = steer_wheel_rpt;
  wheel_speed_rpt_ptr_ = wheel_speed_rpt;
  accel_rpt_ptr_ = accel_rpt;
  brake_rpt_ptr_ = brake_rpt;
  shift_rpt_ptr_ = shift_rpt;
  global_rpt_ptr_ = global_rpt;
  turn_rpt_ptr_ = turn_rpt;

  is_pacmod_enabled_ =
    steer_wheel_rpt_ptr_->enabled && accel_rpt_ptr_->enabled && brake_rpt_ptr_->enabled;
  ROS_DEBUG(
    "[pacmod] enabled: is_pacmod_enabled_ %d, steer %d, accel %d, brake %d, shift %d, global %d",
    is_pacmod_enabled_, steer_wheel_rpt_ptr_->enabled, accel_rpt_ptr_->enabled,
    brake_rpt_ptr_->enabled, shift_rpt_ptr_->enabled, global_rpt_ptr_->enabled);

  const double current_velocity = calculateVehicleVelocity(
    *wheel_speed_rpt_ptr_, *shift_rpt_ptr_);  // current vehicle speed > 0 [m/s]
  const double current_steer_wheel =
    steer_wheel_rpt_ptr_->output;  // current vehicle steering wheel angle [rad]
  const double adaptive_gear_ratio =
    calculateVariableGearRatio(current_velocity, current_steer_wheel);
  const double current_steer = current_steer_wheel / adaptive_gear_ratio - steering_offset_;

  std_msgs::Header header;
  header.frame_id = base_frame_id_;
  header.stamp = ros::Time::now();

  /* publish vehicle status control_mode */
  {
    autoware_vehicle_msgs::ControlMode control_mode_msg;
    control_mode_msg.header = header;

    if (!global_rpt->enabled) {
      control_mode_msg.data = autoware_vehicle_msgs::ControlMode::MANUAL;
    } else if (is_pacmod_enabled_) {
      control_mode_msg.data = autoware_vehicle_msgs::ControlMode::AUTO;
    } else {
      bool is_pedal_enable = (accel_rpt_ptr_->enabled && brake_rpt_ptr_->enabled);
      bool is_steer_enable = steer_wheel_rpt_ptr_->enabled;
      if (is_pedal_enable) {
        control_mode_msg.data = autoware_vehicle_msgs::ControlMode::AUTO_PEDAL_ONLY;
      } else if (is_steer_enable) {
        control_mode_msg.data = autoware_vehicle_msgs::ControlMode::AUTO_STEER_ONLY;
      } else {
        ROS_ERROR(
          "[pacmod] global_rpt is enable, but steer & pedal is disabled. Set mode = MANUAL");
        control_mode_msg.data = autoware_vehicle_msgs::ControlMode::MANUAL;
      }
    }

    control_mode_pub_.publish(control_mode_msg);
  }

  /* publish vehicle status twist */
  {
    geometry_msgs::TwistStamped twist;
    twist.header = header;
    twist.twist.linear.x = current_velocity;                                           // [m/s]
    twist.twist.angular.z = current_velocity * std::tan(current_steer) / wheel_base_;  // [rad/s]
    vehicle_twist_pub_.publish(twist);
  }

  /* publish current shift */
  {
    autoware_vehicle_msgs::ShiftStamped shift_msg;
    shift_msg.header = header;
    shift_msg.shift.data = toAutowareShiftCmd(*shift_rpt_ptr_);
    shift_status_pub_.publish(shift_msg);
  }

  /* publish current steering angle */
  {
    autoware_vehicle_msgs::Steering steer_msg;
    steer_msg.header = header;
    steer_msg.data = current_steer;
    steering_status_pub_.publish(steer_msg);
  }

  /* publish current turn signal */
  {
    autoware_vehicle_msgs::TurnSignal turn_msg;
    turn_msg.header = header;
    turn_msg.data = toAutowareTurnSignal(*turn_rpt);
    turn_signal_status_pub_.publish(turn_msg);
  }
}

void PacmodInterface::publishCommands()
{
  /* guard */
  if (!raw_vehicle_cmd_ptr_ || !is_pacmod_rpt_received_) {
    ROS_INFO_DELAYED_THROTTLE(
      1.0, "[pacmod] vehicle_cmd = %d, pacmod_msgs = %d", raw_vehicle_cmd_ptr_ != nullptr,
      is_pacmod_rpt_received_);
    return;
  }

  const ros::Time current_time = ros::Time::now();

  double desired_throttle = raw_vehicle_cmd_ptr_->control.throttle + accel_pedal_offset_;
  double desired_brake = raw_vehicle_cmd_ptr_->control.brake + brake_pedal_offset_;
  if (raw_vehicle_cmd_ptr_->control.brake <= std::numeric_limits<double>::epsilon()) {
    desired_brake = 0.0;
  }

  /* check emergency and timeout */
  const bool emergency = (raw_vehicle_cmd_ptr_->emergency == 1);
  const double vehicle_cmd_delta_time_ms =
    (ros::Time::now() - vehicle_command_received_time_).toSec() * 1000.0;
  const bool timeouted =
    (command_timeout_ms_ >= 0.0) ? (vehicle_cmd_delta_time_ms > command_timeout_ms_) : false;
  if (emergency || timeouted) {
    ROS_ERROR("[pacmod] Emergency Stopping, emergency = %d, timeouted = %d", emergency, timeouted);
    desired_throttle = 0.0;
    desired_brake = emergency_brake_;
  }

  const double current_velocity = calculateVehicleVelocity(*wheel_speed_rpt_ptr_, *shift_rpt_ptr_);
  const double current_steer_wheel = steer_wheel_rpt_ptr_->output;

  /* calculate desired steering wheel */
  double adaptive_gear_ratio = calculateVariableGearRatio(current_velocity, current_steer_wheel);
  double desired_steer_wheel =
    (raw_vehicle_cmd_ptr_->control.steering_angle + steering_offset_) * adaptive_gear_ratio;
  desired_steer_wheel =
    std::min(std::max(desired_steer_wheel, -max_steering_wheel_), max_steering_wheel_);

  /* check clear flag */
  bool clear_override = false;
  if (is_pacmod_enabled_ == true) {
    is_clear_override_needed_ = false;
  } else if (is_clear_override_needed_ == true) {
    clear_override = true;
  }

  /* make engage cmd false when a driver overrides vehicle control */
  if (!prev_override_ && global_rpt_ptr_->override_active) {
    ROS_WARN_THROTTLE(1.0, "Pacmod is overrided, enable flag is back to false");
    engage_cmd_ = false;
  }
  prev_override_ = global_rpt_ptr_->override_active;

  /* make engage cmd false when vehicle report is timeouted, e.g. E-stop is depressed */
  const bool report_timeouted = ((ros::Time::now() - global_rpt_ptr_->header.stamp).toSec() > 1.0);
  if (report_timeouted) {
    ROS_WARN_THROTTLE(1.0, "Pacmod report is timeouted, enable flag is back to false");
    engage_cmd_ = false;
  }

  /* make engage cmd false when vehicle fault is active */
  if (global_rpt_ptr_->fault_active) {
    ROS_WARN_THROTTLE(1.0, "Pacmod fault is active, enable flag is back to false");
    engage_cmd_ = false;
  }

  ROS_DEBUG(
    "[pacmod] is_pacmod_enabled_ = %d, is_clear_override_needed_ = %d, clear_override = %d",
    is_pacmod_enabled_, is_clear_override_needed_, clear_override);

  /* check shift change */
  const double brake_for_shift_trans = 0.7;
  uint16_t desired_shift = shift_rpt_ptr_->output;
  if (std::fabs(current_velocity) < 0.1) {  // velocity is low -> the shift can be changed
    if (
      toPacmodShiftCmd(raw_vehicle_cmd_ptr_->shift) !=
      shift_rpt_ptr_->output) {  // need shift change.
      desired_throttle = 0.0;
      desired_brake = brake_for_shift_trans;  // set brake to change the shift
      desired_shift = toPacmodShiftCmd(raw_vehicle_cmd_ptr_->shift);
      ROS_DEBUG(
        "[pacmod] Doing shift change. current = %d, desired = %d. set brake_cmd to %f",
        shift_rpt_ptr_->output, toPacmodShiftCmd(raw_vehicle_cmd_ptr_->shift), desired_brake);
    }
  }

  /* publish accel cmd */
  {
    pacmod_msgs::SystemCmdFloat accel_cmd;
    accel_cmd.header.frame_id = base_frame_id_;
    accel_cmd.header.stamp = current_time;
    accel_cmd.enable = engage_cmd_;
    accel_cmd.ignore_overrides = false;
    accel_cmd.clear_override = clear_override;
    accel_cmd.clear_faults = false;
    accel_cmd.command = std::max(0.0, std::min(desired_throttle, max_throttle_));
    accel_cmd_pub_.publish(accel_cmd);
  }

  /* publish brake cmd */
  {
    pacmod_msgs::SystemCmdFloat brake_cmd;
    brake_cmd.header.frame_id = base_frame_id_;
    brake_cmd.header.stamp = current_time;
    brake_cmd.enable = engage_cmd_;
    brake_cmd.ignore_overrides = false;
    brake_cmd.clear_override = clear_override;
    brake_cmd.clear_faults = false;
    brake_cmd.command = std::max(0.0, std::min(desired_brake, max_brake_));
    brake_cmd_pub_.publish(brake_cmd);
  }

  /* publish steering cmd */
  {
    pacmod_msgs::SteerSystemCmd steer_cmd;
    double desired_rotation_rate;  // [rad/s]

    steer_cmd.header.frame_id = base_frame_id_;
    steer_cmd.header.stamp = current_time;
    steer_cmd.enable = engage_cmd_;
    steer_cmd.ignore_overrides = false;
    steer_cmd.clear_override = clear_override;
    steer_cmd.clear_faults = false;
    steer_cmd.rotation_rate = calcSteerWheelRateCmd(adaptive_gear_ratio);
    steer_cmd.command = steerWheelRateLimiter(
      desired_steer_wheel, prev_steer_cmd_.command, current_time, prev_steer_cmd_.header.stamp,
      steer_cmd.rotation_rate, current_steer_wheel, engage_cmd_);
    steer_cmd_pub_.publish(steer_cmd);
    prev_steer_cmd_ = steer_cmd;
  }

  /* publish raw steering cmd for debug */
  {
    pacmod_msgs::SteerSystemCmd raw_steer_cmd;

    raw_steer_cmd.header.frame_id = base_frame_id_;
    raw_steer_cmd.header.stamp = current_time;
    raw_steer_cmd.enable = engage_cmd_;
    raw_steer_cmd.ignore_overrides = false;
    raw_steer_cmd.clear_override = clear_override;
    raw_steer_cmd.clear_faults = false;
    raw_steer_cmd.command = desired_steer_wheel;
    raw_steer_cmd.rotation_rate =
      raw_vehicle_cmd_ptr_->control.steering_angle_velocity * adaptive_gear_ratio;
    raw_steer_cmd_pub_.publish(raw_steer_cmd);
  }

  /* publish shift cmd */
  {
    pacmod_msgs::SystemCmdInt shift_cmd;
    shift_cmd.header.frame_id = base_frame_id_;
    shift_cmd.header.stamp = current_time;
    shift_cmd.enable = engage_cmd_;
    shift_cmd.ignore_overrides = false;
    shift_cmd.clear_override = clear_override;
    shift_cmd.clear_faults = false;
    shift_cmd.command = desired_shift;
    shift_cmd_pub_.publish(shift_cmd);
  }

  if (turn_signal_cmd_ptr_) {
    /* publish shift cmd */
    pacmod_msgs::SystemCmdInt turn_cmd;
    turn_cmd.header.frame_id = base_frame_id_;
    turn_cmd.header.stamp = current_time;
    turn_cmd.enable = engage_cmd_;
    turn_cmd.ignore_overrides = false;
    turn_cmd.clear_override = clear_override;
    turn_cmd.clear_faults = false;
    turn_cmd.command = toPacmodTurnCmdWithHazardRecover(*turn_signal_cmd_ptr_);
    turn_cmd_pub_.publish(turn_cmd);
  }
}

double PacmodInterface::calcSteerWheelRateCmd(const double gear_ratio)
{
  const auto current_vel =
    std::fabs(calculateVehicleVelocity(*wheel_speed_rpt_ptr_, *shift_rpt_ptr_));

  // send low steer rate at low speed
  if (current_vel < std::numeric_limits<double>::epsilon()) {
    return steering_wheel_rate_stopped_;
  } else if (current_vel < low_vel_thresh_) {
    return steering_wheel_rate_low_vel_;
  }

  if (!enable_steering_rate_control_) {
    return max_steering_wheel_rate_;
  }

  constexpr double margin = 1.5;
  const double rate = margin * raw_vehicle_cmd_ptr_->control.steering_angle_velocity * gear_ratio;
  return std::min(std::max(std::fabs(rate), min_steering_wheel_rate_), max_steering_wheel_rate_);
}

double PacmodInterface::calculateVehicleVelocity(
  const pacmod_msgs::WheelSpeedRpt & wheel_speed_rpt, const pacmod_msgs::SystemRptInt & shift_rpt)
{
  const double sign = (shift_rpt.output == pacmod_msgs::SystemRptInt::SHIFT_REVERSE) ? -1 : 1;
  const double vel =
    (wheel_speed_rpt.rear_left_wheel_speed + wheel_speed_rpt.rear_right_wheel_speed) * 0.5 *
    tire_radius_;
  return sign * vel;
}

double PacmodInterface::calculateVariableGearRatio(const double vel, const double steer_wheel)
{
  return std::max(
    1e-5, vgr_coef_a_ + vgr_coef_b_ * vel * vel - vgr_coef_c_ * std::fabs(steer_wheel));
}

uint16_t PacmodInterface::toPacmodShiftCmd(const autoware_vehicle_msgs::Shift & shift)
{
  using autoware_vehicle_msgs::Shift;
  using pacmod_msgs::SystemCmdInt;

  if (shift.data == Shift::PARKING) return SystemCmdInt::SHIFT_PARK;
  if (shift.data == Shift::REVERSE) return SystemCmdInt::SHIFT_REVERSE;
  if (shift.data == Shift::NEUTRAL) return SystemCmdInt::SHIFT_NEUTRAL;
  if (shift.data == Shift::DRIVE) return SystemCmdInt::SHIFT_FORWARD;
  if (shift.data == Shift::LOW) return SystemCmdInt::SHIFT_LOW;

  return SystemCmdInt::SHIFT_NONE;
}

int32_t PacmodInterface::toAutowareShiftCmd(const pacmod_msgs::SystemRptInt & shift)
{
  using autoware_vehicle_msgs::Shift;
  using pacmod_msgs::SystemRptInt;

  if (shift.output == SystemRptInt::SHIFT_PARK) return Shift::PARKING;
  if (shift.output == SystemRptInt::SHIFT_REVERSE) return Shift::REVERSE;
  if (shift.output == SystemRptInt::SHIFT_NEUTRAL) return Shift::NEUTRAL;
  if (shift.output == SystemRptInt::SHIFT_FORWARD) return Shift::DRIVE;
  if (shift.output == SystemRptInt::SHIFT_LOW) return Shift::LOW;

  return Shift::NONE;
}

uint16_t PacmodInterface::toPacmodTurnCmd(const autoware_vehicle_msgs::TurnSignal & turn)
{
  using autoware_vehicle_msgs::TurnSignal;
  using pacmod_msgs::SystemCmdInt;

  if (turn.data == TurnSignal::LEFT) return SystemCmdInt::TURN_LEFT;
  if (turn.data == TurnSignal::RIGHT) return SystemCmdInt::TURN_RIGHT;
  if (turn.data == TurnSignal::HAZARD) return SystemCmdInt::TURN_HAZARDS;

  return SystemCmdInt::TURN_NONE;
}

uint16_t PacmodInterface::toPacmodTurnCmdWithHazardRecover(
  const autoware_vehicle_msgs::TurnSignal & turn)
{
  using pacmod_msgs::SystemRptInt;

  if (!engage_cmd_ || turn_rpt_ptr_->command == turn_rpt_ptr_->output) {
    last_shift_inout_matched_time_ = ros::Time::now();
    return toPacmodTurnCmd(turn);
  }

  if ((ros::Time::now() - last_shift_inout_matched_time_).toSec() < hazard_thresh_time_) {
    return toPacmodTurnCmd(turn);
  }

  // hazard recover mode
  if (hazard_recover_count_ > hazard_recover_cmd_num_) {
    last_shift_inout_matched_time_ = ros::Time::now();
    hazard_recover_count_ = 0;
  }
  hazard_recover_count_++;

  if (
    turn_rpt_ptr_->command != SystemRptInt::TURN_HAZARDS &&
    turn_rpt_ptr_->output == SystemRptInt::TURN_HAZARDS) {
    // publish hazard commands for turning off the hazard lights
    return SystemRptInt::TURN_HAZARDS;
  } else if (
    turn_rpt_ptr_->command == SystemRptInt::TURN_HAZARDS &&
    turn_rpt_ptr_->output != SystemRptInt::TURN_HAZARDS) {
    // publish none commands for turning on the hazard lights
    return SystemRptInt::TURN_NONE;
  } else {
    // something wrong
    ROS_ERROR_STREAM(
      "turn singnal command and output do not match. "
      << "COMMAND: " << turn_rpt_ptr_->command << "; OUTPUT: " << turn_rpt_ptr_->output);
    return toPacmodTurnCmd(turn);
  }
}

int32_t PacmodInterface::toAutowareTurnSignal(const pacmod_msgs::SystemRptInt & turn)
{
  using pacmod_msgs::SystemRptInt;
  if (turn.output == SystemRptInt::TURN_RIGHT) {
    return autoware_vehicle_msgs::TurnSignal::RIGHT;
  } else if (turn.output == SystemRptInt::TURN_LEFT) {
    return autoware_vehicle_msgs::TurnSignal::LEFT;
  } else if (turn.output == SystemRptInt::TURN_NONE) {
    return autoware_vehicle_msgs::TurnSignal::NONE;
  } else if (turn.output == SystemRptInt::TURN_HAZARDS) {
    return autoware_vehicle_msgs::TurnSignal::HAZARD;
  } else {
    return autoware_vehicle_msgs::TurnSignal::NONE;
  }
}

double PacmodInterface::steerWheelRateLimiter(
  const double current_steer_cmd, const double prev_steer_cmd, const ros::Time & current_steer_time,
  const ros::Time & prev_steer_time, const double steer_rate, const double current_steer_output,
  const bool engage)
{
  if (!engage) {
    // return current steer as steer command ( do not apply steer rate filter )
    return current_steer_output;
  }

  const double dsteer = current_steer_cmd - prev_steer_cmd;
  const double dt = std::max(0.0, (current_steer_time - prev_steer_time).toSec());
  const double max_dsteer = std::fabs(steer_rate) * dt;
  const double limitted_steer_cmd =
    prev_steer_cmd + std::min(std::max(-max_dsteer, dsteer), max_dsteer);
  return limitted_steer_cmd;
}
