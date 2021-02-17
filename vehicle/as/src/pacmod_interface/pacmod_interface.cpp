// Copyright 2017-2019 Autoware Foundation
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

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "pacmod_interface/pacmod_interface.hpp"

PacmodInterface::PacmodInterface()
: Node("pacmod_interface"),
  vehicle_info_(vehicle_info_util::VehicleInfo::create(*this))
{
  /* setup parameters */
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");
  command_timeout_ms_ = declare_parameter("command_timeout_ms", 1000);
  loop_rate_ = declare_parameter("loop_rate", 30.0);

  /* parameters for vehicle specifications */
  tire_radius_ = vehicle_info_.wheel_radius_m_;
  wheel_base_ = vehicle_info_.wheel_base_m_;


  steering_offset_ = declare_parameter("steering_offset", 0.0);
  enable_steering_rate_control_ = declare_parameter("enable_steering_rate_control", false);

  /* parameters for emergency stop */
  emergency_brake_ = declare_parameter("emergency_brake", 0.7);

  /* vehicle parameters */
  vgr_coef_a_ = declare_parameter("vgr_coef_a", 15.713);
  vgr_coef_b_ = declare_parameter("vgr_coef_b", 0.053);
  vgr_coef_c_ = declare_parameter("vgr_coef_c", 0.042);
  accel_pedal_offset_ = declare_parameter("accel_pedal_offset", 0.0);
  brake_pedal_offset_ = declare_parameter("brake_pedal_offset", 0.0);

  /* parameters for limitter */
  max_throttle_ = declare_parameter("max_throttle", 0.2);
  max_brake_ = declare_parameter("max_brake", 0.8);
  max_steering_wheel_ = declare_parameter("max_steering_wheel", 2.7 * M_PI);
  max_steering_wheel_rate_ = declare_parameter("max_steering_wheel_rate", 6.6);
  min_steering_wheel_rate_ = declare_parameter("min_steering_wheel_rate", 0.5);
  steering_wheel_rate_low_vel_ = declare_parameter("steering_wheel_rate_low_vel", 5.0);
  steering_wheel_rate_stopped_ = declare_parameter("steering_wheel_rate_stopped", 5.0);
  low_vel_thresh_ = declare_parameter("low_vel_thresh", 1.389);  // 5.0kmh

  /* parameters for turn signal recovery */
  hazard_thresh_time_ = declare_parameter("hazard_thresh_time", 0.20);  // s
  /* initialize */
  prev_steer_cmd_.header.stamp = this->now();
  prev_steer_cmd_.command = 0.0;

  /* subscribers */
  using std::placeholders::_1;

  // From autoware
  raw_vehicle_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::RawVehicleCommand>(
    "/vehicle/raw_vehicle_cmd", rclcpp::QoS{1},
    std::bind(&PacmodInterface::callbackVehicleCmd, this, _1));
  turn_signal_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::TurnSignal>(
    "/control/turn_signal_cmd", rclcpp::QoS{1},
    std::bind(&PacmodInterface::callbackTurnSignalCmd, this, _1));
  engage_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::Engage>(
    "/vehicle/engage", rclcpp::QoS{1}, std::bind(&PacmodInterface::callbackEngage, this, _1));

  // From pacmod

  steer_wheel_rpt_sub_ =
    std::make_unique<message_filters::Subscriber<pacmod_msgs::msg::SystemRptFloat>>(
    this, "/pacmod/parsed_tx/steer_rpt");
  wheel_speed_rpt_sub_ =
    std::make_unique<message_filters::Subscriber<pacmod_msgs::msg::WheelSpeedRpt>>(
    this, "/pacmod/parsed_tx/wheel_speed_rpt");
  accel_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod_msgs::msg::SystemRptFloat>>(
    this, "/pacmod/parsed_tx/accel_rpt");
  brake_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod_msgs::msg::SystemRptFloat>>(
    this, "/pacmod/parsed_tx/brake_rpt");
  shift_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod_msgs::msg::SystemRptInt>>(
    this, "/pacmod/parsed_tx/shift_rpt");
  turn_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod_msgs::msg::SystemRptInt>>(
    this, "/pacmod/parsed_tx/turn_rpt");
  global_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod_msgs::msg::GlobalRpt>>(
    this, "/pacmod/parsed_tx/global_rpt");

  pacmod_feedbacks_sync_ =
    std::make_unique<message_filters::Synchronizer<PacmodFeedbacksSyncPolicy>>(
    PacmodFeedbacksSyncPolicy(10), *steer_wheel_rpt_sub_, *wheel_speed_rpt_sub_, *accel_rpt_sub_,
    *brake_rpt_sub_, *shift_rpt_sub_, *turn_rpt_sub_, *global_rpt_sub_);

  pacmod_feedbacks_sync_->registerCallback(
    std::bind(
      &PacmodInterface::callbackPacmodRpt, this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
      std::placeholders::_7));

  /* publisher */
  // To pacmod
  accel_cmd_pub_ =
    create_publisher<pacmod_msgs::msg::SystemCmdFloat>("pacmod/as_rx/accel_cmd", rclcpp::QoS{1});
  brake_cmd_pub_ =
    create_publisher<pacmod_msgs::msg::SystemCmdFloat>("pacmod/as_rx/brake_cmd", rclcpp::QoS{1});
  steer_cmd_pub_ =
    create_publisher<pacmod_msgs::msg::SteerSystemCmd>("pacmod/as_rx/steer_cmd", rclcpp::QoS{1});
  shift_cmd_pub_ =
    create_publisher<pacmod_msgs::msg::SystemCmdInt>("pacmod/as_rx/shift_cmd", rclcpp::QoS{1});
  turn_cmd_pub_ =
    create_publisher<pacmod_msgs::msg::SystemCmdInt>("pacmod/as_rx/turn_cmd", rclcpp::QoS{1});
  raw_steer_cmd_pub_ = create_publisher<pacmod_msgs::msg::SteerSystemCmd>(
    "pacmod/as_rx/raw_steer_cmd", rclcpp::QoS{1});  // only for debug

  // To Autoware
  control_mode_pub_ = create_publisher<autoware_vehicle_msgs::msg::ControlMode>(
    "/vehicle/status/control_mode", rclcpp::QoS{1});
  vehicle_twist_pub_ =
    create_publisher<geometry_msgs::msg::TwistStamped>("/vehicle/status/twist", rclcpp::QoS{1});
  steering_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::Steering>(
    "/vehicle/status/steering", rclcpp::QoS{1});
  shift_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::ShiftStamped>(
    "/vehicle/status/shift", rclcpp::QoS{1});
  turn_signal_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::TurnSignal>(
    "/vehicle/status/turn_signal", rclcpp::QoS{1});

  // Timer
  auto timer_callback = std::bind(&PacmodInterface::publishCommands, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / loop_rate_));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void PacmodInterface::callbackVehicleCmd(
  const autoware_vehicle_msgs::msg::RawVehicleCommand::ConstSharedPtr msg)
{
  vehicle_command_received_time_ = this->get_clock()->now();
  raw_vehicle_cmd_ptr_ = msg;
}
void PacmodInterface::callbackTurnSignalCmd(
  const autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg)
{
  turn_signal_cmd_ptr_ = msg;
}
void PacmodInterface::callbackEngage(const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg)
{
  engage_cmd_ = msg->engage;
  is_clear_override_needed_ = true;
}

void PacmodInterface::callbackPacmodRpt(
  const pacmod_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt,
  const pacmod_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt,
  const pacmod_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt,
  const pacmod_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt,
  const pacmod_msgs::msg::SystemRptInt::ConstSharedPtr shift_rpt,
  const pacmod_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt,
  const pacmod_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt)
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
  RCLCPP_DEBUG(
    get_logger(),
    "enabled: is_pacmod_enabled_ %d, steer %d, accel %d, brake %d, shift %d, "
    "global %d",
    is_pacmod_enabled_, steer_wheel_rpt_ptr_->enabled, accel_rpt_ptr_->enabled,
    brake_rpt_ptr_->enabled, shift_rpt_ptr_->enabled, global_rpt_ptr_->enabled);

  const double current_velocity = calculateVehicleVelocity(
    *wheel_speed_rpt_ptr_, *shift_rpt_ptr_);  // current vehicle speed > 0 [m/s]
  const double current_steer_wheel =
    steer_wheel_rpt_ptr_->output;  // current vehicle steering wheel angle [rad]
  const double adaptive_gear_ratio =
    calculateVariableGearRatio(current_velocity, current_steer_wheel);
  const double current_steer = current_steer_wheel / adaptive_gear_ratio - steering_offset_;

  std_msgs::msg::Header header;
  header.frame_id = base_frame_id_;
  header.stamp = get_clock()->now();

  /* publish vehicle status control_mode */
  {
    autoware_vehicle_msgs::msg::ControlMode control_mode_msg;
    control_mode_msg.header = header;

    if (!global_rpt->enabled) {
      control_mode_msg.data = autoware_vehicle_msgs::msg::ControlMode::MANUAL;
    } else if (is_pacmod_enabled_) {
      control_mode_msg.data = autoware_vehicle_msgs::msg::ControlMode::AUTO;
    } else {
      bool is_pedal_enable = (accel_rpt_ptr_->enabled && brake_rpt_ptr_->enabled);
      bool is_steer_enable = steer_wheel_rpt_ptr_->enabled;
      if (is_pedal_enable) {
        control_mode_msg.data = autoware_vehicle_msgs::msg::ControlMode::AUTO_PEDAL_ONLY;
      } else if (is_steer_enable) {
        control_mode_msg.data = autoware_vehicle_msgs::msg::ControlMode::AUTO_STEER_ONLY;
      } else {
        RCLCPP_ERROR(
          get_logger(), "global_rpt is enable, but steer & pedal is disabled. Set mode = MANUAL");
        control_mode_msg.data = autoware_vehicle_msgs::msg::ControlMode::MANUAL;
      }
    }

    control_mode_pub_->publish(control_mode_msg);
  }

  /* publish vehicle status twist */
  {
    geometry_msgs::msg::TwistStamped twist;
    twist.header = header;
    twist.twist.linear.x = current_velocity;                                        // [m/s]
    twist.twist.angular.z = current_velocity * std::tan(current_steer) / wheel_base_;  // [rad/s]
    vehicle_twist_pub_->publish(twist);
  }

  /* publish current shift */
  {
    autoware_vehicle_msgs::msg::ShiftStamped shift_msg;
    shift_msg.header = header;
    shift_msg.shift.data = toAutowareShiftCmd(*shift_rpt_ptr_);
    shift_status_pub_->publish(shift_msg);
  }

  /* publish current steernig angle */
  {
    autoware_vehicle_msgs::msg::Steering steer_msg;
    steer_msg.header = header;
    steer_msg.data = current_steer;
    steering_status_pub_->publish(steer_msg);
  }

  {
    autoware_vehicle_msgs::msg::TurnSignal turn_msg;
    turn_msg.header = header;
    turn_msg.data = toAutowareTurnSignal(*turn_rpt);
    turn_signal_status_pub_->publish(turn_msg);
  }
}

void PacmodInterface::publishCommands()
{
  /* guard */
  if (!raw_vehicle_cmd_ptr_ || !is_pacmod_rpt_received_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "vehicle_cmd = %d, pacmod_msgs = %d", raw_vehicle_cmd_ptr_ != nullptr,
      is_pacmod_rpt_received_);
    return;
  }

  const rclcpp::Time current_time = get_clock()->now();

  double desired_throttle = raw_vehicle_cmd_ptr_->control.throttle + accel_pedal_offset_;
  double desired_brake = raw_vehicle_cmd_ptr_->control.brake + brake_pedal_offset_;
  if (raw_vehicle_cmd_ptr_->control.brake <= std::numeric_limits<double>::epsilon()) {
    desired_brake = 0.0;
  }

  /* check emergency and timeout */
  const bool emergency = (raw_vehicle_cmd_ptr_->emergency == 1);
  const double vehicle_cmd_delta_time_ms =
    (get_clock()->now() - vehicle_command_received_time_).seconds() * 1000.0;
  const bool timeouted =
    (command_timeout_ms_ >= 0.0) ? (vehicle_cmd_delta_time_ms > command_timeout_ms_) : false;
  if (emergency || timeouted) {
    RCLCPP_ERROR(
      get_logger(), "Emergency Stopping, emergency = %d, timeouted = %d",
      emergency, timeouted);
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
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Pacmod is overrided, enable flag is back to false");
    engage_cmd_ = false;
  }
  prev_override_ = global_rpt_ptr_->override_active;

  /* make engage cmd false when vehicle report is timeouted, e.g. E-stop is depressed */
  const bool report_timeouted = ((this->now() - global_rpt_ptr_->header.stamp).seconds() > 1.0);
  if (report_timeouted) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Pacmod report is timeouted, enable flag is back to false");
    engage_cmd_ = false;
  }

  /* make engage cmd false when vehicle fault is active */
  if (global_rpt_ptr_->fault_active) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Pacmod fault is active, enable flag is back to false");
    engage_cmd_ = false;
  }
  RCLCPP_DEBUG(
    get_logger(),
    "is_pacmod_enabled_ = %d, is_clear_override_needed_ = %d, clear_override = "
    "%d",
    is_pacmod_enabled_, is_clear_override_needed_, clear_override);

  /* check shift change */
  const double brake_for_shift_trans = 0.7;
  uint16_t desired_shift = shift_rpt_ptr_->output;
  if (std::fabs(current_velocity) < 0.1) {  // velocity is low -> the shift can be changed
    if (
      toPacmodShiftCmd(raw_vehicle_cmd_ptr_->shift) !=
      shift_rpt_ptr_->output)    // need shift change.
    {
      desired_throttle = 0.0;
      desired_brake = brake_for_shift_trans;  // set brake to change the shift
      desired_shift = toPacmodShiftCmd(raw_vehicle_cmd_ptr_->shift);
      RCLCPP_DEBUG(
        get_logger(),
        "Doing shift change. current = %d, desired = %d. set brake_cmd to %f",
        shift_rpt_ptr_->output, toPacmodShiftCmd(raw_vehicle_cmd_ptr_->shift), desired_brake);
    }
  }

  /* publish accel cmd */
  {
    pacmod_msgs::msg::SystemCmdFloat accel_cmd;
    accel_cmd.header.frame_id = base_frame_id_;
    accel_cmd.header.stamp = current_time;
    accel_cmd.enable = engage_cmd_;
    accel_cmd.ignore_overrides = false;
    accel_cmd.clear_override = clear_override;
    accel_cmd.clear_faults = false;
    accel_cmd.command = std::max(0.0, std::min(desired_throttle, max_throttle_));
    accel_cmd_pub_->publish(accel_cmd);
  }

  /* publish brake cmd */
  {
    pacmod_msgs::msg::SystemCmdFloat brake_cmd;
    brake_cmd.header.frame_id = base_frame_id_;
    brake_cmd.header.stamp = current_time;
    brake_cmd.enable = engage_cmd_;
    brake_cmd.ignore_overrides = false;
    brake_cmd.clear_override = clear_override;
    brake_cmd.clear_faults = false;
    brake_cmd.command = std::max(0.0, std::min(desired_brake, max_brake_));
    brake_cmd_pub_->publish(brake_cmd);
  }

  /* publish steering cmd */
  {
    pacmod_msgs::msg::SteerSystemCmd steer_cmd;
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
    steer_cmd_pub_->publish(steer_cmd);
    prev_steer_cmd_ = steer_cmd;
  }

  /* publish raw steering cmd for debug */
  {
    pacmod_msgs::msg::SteerSystemCmd raw_steer_cmd;
    raw_steer_cmd.header.frame_id = base_frame_id_;
    raw_steer_cmd.header.stamp = current_time;
    raw_steer_cmd.enable = engage_cmd_;
    raw_steer_cmd.ignore_overrides = false;
    raw_steer_cmd.clear_override = clear_override;
    raw_steer_cmd.clear_faults = false;
    raw_steer_cmd.command = desired_steer_wheel;
    raw_steer_cmd.rotation_rate =
      raw_vehicle_cmd_ptr_->control.steering_angle_velocity * adaptive_gear_ratio;
    raw_steer_cmd_pub_->publish(raw_steer_cmd);
  }

  /* publish shift cmd */
  {
    pacmod_msgs::msg::SystemCmdInt shift_cmd;
    shift_cmd.header.frame_id = base_frame_id_;
    shift_cmd.header.stamp = current_time;
    shift_cmd.enable = engage_cmd_;
    shift_cmd.ignore_overrides = false;
    shift_cmd.clear_override = clear_override;
    shift_cmd.clear_faults = false;
    shift_cmd.command = desired_shift;
    shift_cmd_pub_->publish(shift_cmd);
  }

  if (turn_signal_cmd_ptr_) {
    /* publish shift cmd */
    pacmod_msgs::msg::SystemCmdInt turn_cmd;
    turn_cmd.header.frame_id = base_frame_id_;
    turn_cmd.header.stamp = current_time;
    turn_cmd.enable = engage_cmd_;
    turn_cmd.ignore_overrides = false;
    turn_cmd.clear_override = clear_override;
    turn_cmd.clear_faults = false;
    turn_cmd.command = toPacmodTurnCmdWithHazardRecover(*turn_signal_cmd_ptr_);
    turn_cmd_pub_->publish(turn_cmd);
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
  const pacmod_msgs::msg::WheelSpeedRpt & wheel_speed_rpt,
  const pacmod_msgs::msg::SystemRptInt & shift_rpt)
{
  const double sign = (shift_rpt.output == pacmod_msgs::msg::SystemRptInt::SHIFT_REVERSE) ? -1 : 1;
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

uint16_t PacmodInterface::toPacmodShiftCmd(const autoware_vehicle_msgs::msg::Shift & shift)
{
  using autoware_vehicle_msgs::msg::Shift;
  using pacmod_msgs::msg::SystemCmdInt;

  if (shift.data == Shift::PARKING) {return SystemCmdInt::SHIFT_PARK;}
  if (shift.data == Shift::REVERSE) {return SystemCmdInt::SHIFT_REVERSE;}
  if (shift.data == Shift::NEUTRAL) {return SystemCmdInt::SHIFT_NEUTRAL;}
  if (shift.data == Shift::DRIVE) {return SystemCmdInt::SHIFT_FORWARD;}
  if (shift.data == Shift::LOW) {return SystemCmdInt::SHIFT_LOW;}

  return SystemCmdInt::SHIFT_NONE;
}

int32_t PacmodInterface::toAutowareShiftCmd(const pacmod_msgs::msg::SystemRptInt & shift)
{
  using autoware_vehicle_msgs::msg::Shift;
  using pacmod_msgs::msg::SystemRptInt;

  if (shift.output == SystemRptInt::SHIFT_PARK) {return Shift::PARKING;}
  if (shift.output == SystemRptInt::SHIFT_REVERSE) {return Shift::REVERSE;}
  if (shift.output == SystemRptInt::SHIFT_NEUTRAL) {return Shift::NEUTRAL;}
  if (shift.output == SystemRptInt::SHIFT_FORWARD) {return Shift::DRIVE;}
  if (shift.output == SystemRptInt::SHIFT_LOW) {return Shift::LOW;}

  return Shift::NONE;
}

uint16_t PacmodInterface::toPacmodTurnCmd(const autoware_vehicle_msgs::msg::TurnSignal & turn)
{
  using autoware_vehicle_msgs::msg::TurnSignal;
  using pacmod_msgs::msg::SystemCmdInt;

  if (turn.data == TurnSignal::LEFT) {return SystemCmdInt::TURN_LEFT;}
  if (turn.data == TurnSignal::RIGHT) {return SystemCmdInt::TURN_RIGHT;}
  if (turn.data == TurnSignal::HAZARD) {return SystemCmdInt::TURN_HAZARDS;}

  return SystemCmdInt::TURN_NONE;
}

uint16_t PacmodInterface::toPacmodTurnCmdWithHazardRecover(
  const autoware_vehicle_msgs::msg::TurnSignal & turn)
{
  using pacmod_msgs::msg::SystemRptInt;

  if (!engage_cmd_ || turn_rpt_ptr_->command == turn_rpt_ptr_->output) {
    last_shift_inout_matched_time_ = this->now();
    return toPacmodTurnCmd(turn);
  }

  if ((this->now() - last_shift_inout_matched_time_).seconds() < hazard_thresh_time_) {
    return toPacmodTurnCmd(turn);
  }

  // hazard recover mode
  if (hazard_recover_count_ > hazard_recover_cmd_num_) {
    last_shift_inout_matched_time_ = this->now();
    hazard_recover_count_ = 0;
  }
  hazard_recover_count_++;

  if (
    turn_rpt_ptr_->command != SystemRptInt::TURN_HAZARDS &&
    turn_rpt_ptr_->output == SystemRptInt::TURN_HAZARDS)
  {
    // publish hazard commands for turning off the hazard lights
    return SystemRptInt::TURN_HAZARDS;
  } else if (  // NOLINT
    turn_rpt_ptr_->command == SystemRptInt::TURN_HAZARDS &&
    turn_rpt_ptr_->output != SystemRptInt::TURN_HAZARDS)
  {
    // publish none commands for turning on the hazard lights
    return SystemRptInt::TURN_NONE;
  } else {
    // something wrong
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "turn singnal command and output do not match. " <<
        "COMMAND: " << turn_rpt_ptr_->command << "; OUTPUT: " << turn_rpt_ptr_->output);
    return toPacmodTurnCmd(turn);
  }
}

int32_t PacmodInterface::toAutowareTurnSignal(const pacmod_msgs::msg::SystemRptInt & turn)
{
  using pacmod_msgs::msg::SystemRptInt;
  if (turn.output == SystemRptInt::TURN_RIGHT) {
    return autoware_vehicle_msgs::msg::TurnSignal::RIGHT;
  } else if (turn.output == SystemRptInt::TURN_LEFT) {
    return autoware_vehicle_msgs::msg::TurnSignal::LEFT;
  } else if (turn.output == SystemRptInt::TURN_NONE) {
    return autoware_vehicle_msgs::msg::TurnSignal::NONE;
  } else if (turn.output == SystemRptInt::TURN_HAZARDS) {
    return autoware_vehicle_msgs::msg::TurnSignal::HAZARD;
  } else {
    return autoware_vehicle_msgs::msg::TurnSignal::NONE;
  }
}

double PacmodInterface::steerWheelRateLimiter(
  const double current_steer_cmd, const double prev_steer_cmd,
  const rclcpp::Time & current_steer_time, const rclcpp::Time & prev_steer_time,
  const double steer_rate, const double current_steer_output,
  const bool engage)
{
  if (!engage) {
    // return current steer as steer command ( do not apply steer rate filter )
    return current_steer_output;
  }

  const double dsteer = current_steer_cmd - prev_steer_cmd;
  const double dt = std::max(0.0, (current_steer_time - prev_steer_time).seconds());
  const double max_dsteer = std::fabs(steer_rate) * dt;
  const double limitted_steer_cmd =
    prev_steer_cmd + std::min(std::max(-max_dsteer, dsteer), max_dsteer);
  return limitted_steer_cmd;
}
