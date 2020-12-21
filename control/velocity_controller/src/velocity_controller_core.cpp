/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not enable this file except in compliance with the License.
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

#include "velocity_controller/velocity_controller.hpp"

#include "tf2_ros/create_timer_ros.h"

namespace
{
double lowpass_filter(const double current_value, const double prev_value, const double gain)
{
  return gain * prev_value + (1.0 - gain) * current_value;
}
}  // namespace

VelocityController::VelocityController()
: Node("velocity_controller")
{
  using std::placeholders::_1;

  // parameters timer
  control_rate_ = declare_parameter("control_rate", 30.0);

  // parameters to enable functions
  enable_smooth_stop_ = declare_parameter("enable_smooth_stop", true);
  enable_overshoot_emergency_ = declare_parameter("enable_overshoot_emergency", true);
  enable_slope_compensation_ = declare_parameter("enable_slope_compensation", false);

  // parameters to find a closest waypoint
  closest_dist_thr_ = declare_parameter("closest_waypoint_distance_threshold", 3.0);
  closest_angle_thr_ = declare_parameter("closest_waypoint_angle_threshold", M_PI_4);

  // parameters for stop state
  stop_state_vel_ = declare_parameter("stop_state_vel", 0.0);                          // [m/s]
  stop_state_acc_ = declare_parameter("stop_state_acc", -2.0);                         // [m/s^2]
  stop_state_entry_ego_speed_ = declare_parameter("stop_state_entry_ego_speed", 0.2);  // [m/s]
  stop_state_entry_target_speed_ =
    declare_parameter("stop_state_entry_target_speed", 0.1);  // [m/s]
  stop_state_keep_stopping_dist_ =
    declare_parameter("stop_state_keep_stopping_dist", 0.5);  // [m/s]

  // parameters for delay compensation
  delay_compensation_time_ = declare_parameter("delay_compensation_time", 0.17);  // [sec]

  // parameters for emergency stop by this controller
  emergency_stop_acc_ = declare_parameter("emergency_stop_acc", -2.0);             // [m/s^2]
  emergency_stop_jerk_ = declare_parameter("emergency_stop_jerk", -1.5);           // [m/s^3]
  emergency_overshoot_dist_ = declare_parameter("emergency_overshoot_dist", 1.5);  // [m]

  // parameters for smooth stop
  {
    auto & p = smooth_stop_param_;
    p.stop_dist_ = declare_parameter("smooth_stop/stop_dist", 3.0);                   // [m/s^2]
    p.exit_ego_speed = declare_parameter("smooth_stop/exit_ego_speed", 2.0);          // [m/s]
    p.exit_target_speed = declare_parameter("smooth_stop/exit_target_speed", 2.0);    // [m/s]
    p.entry_ego_speed = declare_parameter("smooth_stop/entry_ego_speed", 1.0);        // [m/s]
    p.entry_target_speed = declare_parameter("smooth_stop/entry_target_speed", 1.0);  // [m/s]
    p.weak_brake_time = declare_parameter("smooth_stop/weak_brake_time", 3.0);        // [sec]
    p.weak_brake_acc = declare_parameter("smooth_stop/weak_brake_acc", -0.4);         // [m/s^2]
    p.increasing_brake_time = declare_parameter("smooth_stop/increasing_brake_time", 3.0);  // [sec]
    p.increasing_brake_gradient =
      declare_parameter("smooth_stop/increasing_brake_gradient", -0.05);        // [m/s^3]
    p.stop_brake_time = declare_parameter("smooth_stop/stop_brake_time", 2.0);  // [sec]
    p.stop_brake_acc = declare_parameter("smooth_stop/stop_brake_acc", -1.7);   // [m/s^2]
  }

  // parameters for acceleration limit
  max_acc_ = declare_parameter("max_acc", 2.0);   // [m/s^2]
  min_acc_ = declare_parameter("min_acc", -5.0);  // [m/s^2]

  // parameters for jerk limit
  max_jerk_ = declare_parameter("max_jerk", 2.0);   // [m/s^3]
  min_jerk_ = declare_parameter("min_jerk", -5.0);  // [m/s^3]

  // parameters for slope compensation
  max_pitch_rad_ = declare_parameter("max_pitch_rad", 0.1);   // [rad]
  min_pitch_rad_ = declare_parameter("min_pitch_rad", -0.1);  // [rad]

  current_vel_threshold_pid_integrate_ = declare_parameter(
    "pid_controller/current_vel_threshold_pid_integration",
    0.5);  // [m/s]

  // subscriber, publisher
  sub_current_vel_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "current_velocity", 1, std::bind(&VelocityController::callbackCurrentVelocity, this, _1));
  sub_trajectory_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "current_trajectory", 1, std::bind(&VelocityController::callbackTrajectory, this, _1));
  pub_control_cmd_ = create_publisher<autoware_control_msgs::msg::ControlCommandStamped>(
    "control_cmd", rclcpp::QoS{1});
  pub_debug_ = create_publisher<std_msgs::msg::Float32MultiArray>("debug_values", rclcpp::QoS{1});

  // Timer
  {
    auto timer_callback = std::bind(&VelocityController::callbackTimerControl, this);
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_));
    timer_control_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
      this->get_clock(), period, std::move(timer_callback),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_control_, nullptr);
  }

  // tf setting
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto cti = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(cti);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // initialize PID gain
  {
    double kp = declare_parameter("pid_controller/kp", 0.0);
    double ki = declare_parameter("pid_controller/ki", 0.0);
    double kd = declare_parameter("pid_controller/kd", 0.0);
    pid_vel_.setGains(kp, ki, kd);
  }

  // initialize PID limits
  {
    double max_pid = declare_parameter("pid_controller/max_out", 0.0);     // [m/s^2]
    double min_pid = declare_parameter("pid_controller/min_out", 0.0);     // [m/s^2]
    double max_p = declare_parameter("pid_controller/max_p_effort", 0.0);  // [m/s^2]
    double min_p = declare_parameter("pid_controller/min_p_effort", 0.0);  // [m/s^2]
    double max_i = declare_parameter("pid_controller/max_i_effort", 0.0);  // [m/s^2]
    double min_i = declare_parameter("pid_controller/min_i_effort", 0.0);  // [m/s^2]
    double max_d = declare_parameter("pid_controller/max_d_effort", 0.0);  // [m/s^2]
    double min_d = declare_parameter("pid_controller/min_d_effort", 0.0);  // [m/s^2]
    pid_vel_.setLimits(max_pid, min_pid, max_p, min_p, max_i, min_i, max_d, min_d);
  }

  // set lowpass filter
  {
    double lpf_vel_error_gain = declare_parameter("pid_controller/lpf_vel_error_gain", 0.9);
    lpf_vel_error_.init(lpf_vel_error_gain);

    double lpf_pitch_gain = declare_parameter("lpf_pitch_gain", 0.95);
    lpf_pitch_.init(lpf_pitch_gain);
  }

  debug_values_.data.clear();
  debug_values_.data.resize(num_debug_values_, 0.0);

  // set parameter callback
  set_param_res_ =
    this->add_on_set_parameters_callback(std::bind(&VelocityController::paramCallback, this, _1));

  // wait at end of constructor so other members are properly initialized first
  blockUntilVehiclePositionAvailable(tf2::durationFromSec(1.0));
}

void VelocityController::callbackCurrentVelocity(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  current_vel_ptr_ = msg;
}

void VelocityController::callbackTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  trajectory_ptr_ = msg;
}

void VelocityController::blockUntilVehiclePositionAvailable(const tf2::Duration & duration)
{
  static constexpr auto input = "map", output = "base_link";
  while (!tf_buffer_->canTransform(input, output, tf2::TimePointZero, tf2::durationFromSec(0.0)) &&
    rclcpp::ok())
  {
    RCLCPP_INFO(
      get_logger(), "waiting %d ms for %s->%s transform to become available",
      std::chrono::duration_cast<std::chrono::milliseconds>(duration).count(), input, output);
    rclcpp::sleep_for(duration);
  }
  RCLCPP_INFO(get_logger(), "transform available");
}

bool VelocityController::updateCurrentPose()
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
      "cannot get map to base_link transform. %s", ex.what());
    return false;
  }
  geometry_msgs::msg::PoseStamped ps;
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  current_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(ps);
  return true;
}

void VelocityController::callbackTimerControl()
{
  const bool is_pose_updated = updateCurrentPose();

  /* gurad */
  if (!is_pose_updated || !current_pose_ptr_ || !current_vel_ptr_ || !trajectory_ptr_) {
    RCLCPP_DEBUG(
      get_logger(),
      "Waiting topics, Publish stop command. pose_update: %d, pose: %d, vel: %d, trajectory: %d",
      is_pose_updated, current_pose_ptr_ != nullptr, current_vel_ptr_ != nullptr,
      trajectory_ptr_ != nullptr);
    control_mode_ = ControlMode::INIT;
    publishCtrlCmd(stop_state_vel_, stop_state_acc_);
    return;
  }

  /* calculate command velocity & acceleration */
  const CtrlCmd ctrl_cmd = calcCtrlCmd();

  /* publish control command */
  publishCtrlCmd(ctrl_cmd.vel, ctrl_cmd.acc);
  debug_values_.data.at(DBGVAL::FLAG_SMOOTH_STOP) = is_smooth_stop_;
  debug_values_.data.at(DBGVAL::FLAG_EMERGENCY_STOP) = is_emergency_stop_;

  /* reset parameters depending on the current control mode */
  resetHandling(control_mode_);
}

rcl_interfaces::msg::SetParametersResult VelocityController::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto update_param = [&](const std::string & name, double & v) {
      auto it = std::find_if(
        parameters.cbegin(), parameters.cend(),
        [&name](const rclcpp::Parameter & parameter) {return parameter.get_name() == name;});
      if (it != parameters.cend()) {
        v = it->as_double();
        return true;
      }
      return false;
    };

  // closest waypoint threshold
  update_param("closest_waypoint_distance_threshold_", closest_dist_thr_);
  update_param("closest_waypoint_angle_threshold_", closest_angle_thr_);

  // stop state
  update_param("stop_state_velocity", stop_state_vel_);
  update_param("stop_state_acc", stop_state_acc_);
  update_param("stop_state_entry_ego_speed", stop_state_entry_ego_speed_);
  update_param("stop_state_entry_target_speed", stop_state_entry_target_speed_);
  update_param("stop_state_keep_stopping_dist", stop_state_keep_stopping_dist_);

  // delay compensation
  update_param("delay_compensation_time", delay_compensation_time_);

  // emergency stop by this controller
  update_param("emergency_stop_acc", emergency_stop_acc_);
  update_param("emergency_stop_jerk", emergency_stop_jerk_);

  // smooth stop
  {
    auto & p = smooth_stop_param_;
    update_param("exit_ego_speed", p.exit_ego_speed);
    update_param("entry_ego_speed", p.entry_ego_speed);
    update_param("exit_target_speed", p.exit_target_speed);
    update_param("entry_target_speed", p.entry_target_speed);
    update_param("weak_brake_time", p.weak_brake_time);
    update_param("weak_brake_acc", p.weak_brake_acc);
    update_param("increasing_brake_time", p.increasing_brake_time);
    update_param("increasing_brake_gradient", p.increasing_brake_gradient);
    update_param("stop_brake_time", p.stop_brake_time);
    update_param("stop_brake_acc", p.stop_brake_acc);
  }

  // acceleration limit
  update_param("max_acc", max_acc_);
  update_param("min_acc", min_acc_);

  // jerk limit
  update_param("max_jerk", max_jerk_);
  update_param("min_jerk", min_jerk_);

  // slope compensation
  update_param("max_pitch_rad", max_pitch_rad_);
  update_param("min_pitch_rad", min_pitch_rad_);
  update_param("current_velocity_threshold_pid_integration", current_vel_threshold_pid_integrate_);

  {
    double lpf_pitch_gain = get_parameter("lpf_pitch_gain").as_double();
    update_param("lpf_pitch_gain", lpf_pitch_gain);
    lpf_pitch_.init(lpf_pitch_gain);
  }

  // velocity feedback
  // PID gain
  {
    // TODO(Takamasa Horibe) Combine the getter and setter into one function,
    // or having them in the class member.
    double kp = get_parameter("pid_controller/kp").as_double();
    double ki = get_parameter("pid_controller/ki").as_double();
    double kd = get_parameter("pid_controller/kd").as_double();
    update_param("pid_controller/kp", kp);
    update_param("pid_controller/ki", ki);
    update_param("pid_controller/kd", kd);
    pid_vel_.setGains(kp, ki, kd);
  }

  // PID limits
  {
    double max_pid = get_parameter("pid_controller/max_out").as_double();
    double min_pid = get_parameter("pid_controller/min_out").as_double();
    double max_p = get_parameter("pid_controller/max_p_effort").as_double();
    double min_p = get_parameter("pid_controller/min_p_effort").as_double();
    double max_i = get_parameter("pid_controller/max_i_effort").as_double();
    double min_i = get_parameter("pid_controller/min_i_effort").as_double();
    double max_d = get_parameter("pid_controller/max_d_effort").as_double();
    double min_d = get_parameter("pid_controller/min_d_effort").as_double();
    update_param("pid_controller/max_out", max_pid);
    update_param("pid_controller/min_out", min_pid);
    update_param("pid_controller/max_p_effort", max_p);
    update_param("pid_controller/min_p_effort", min_p);
    update_param("pid_controller/max_i_effort", max_i);
    update_param("pid_controller/min_i_effort", min_i);
    update_param("pid_controller/max_d_effort", max_d);
    update_param("pid_controller/min_d_effort", min_d);
    pid_vel_.setLimits(max_pid, min_pid, max_p, min_p, max_i, min_i, max_d, min_d);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

CtrlCmd VelocityController::calcCtrlCmd()
{
  /* initialize parameters */
  const double dt = getDt();
  const auto & current_pose = current_pose_ptr_->pose;

  /* -- find a closest waypoint index --
   *
   * If the closest is not found (when the threshold is exceeded), it is treated as an emergency stop.
   *
   * Outout velocity : "0" with maximum acceleration constraint
   * Output acceleration : "emergency_stop_acc_" with maximum jerk constraint
   *
   */
  int closest_idx;
  if (!vcutils::calcClosestWithThr(
      *trajectory_ptr_, current_pose, closest_angle_thr_, closest_dist_thr_, closest_idx))
  {
    double vel_cmd = applyRateFilter(0.0, prev_vel_cmd_, dt, emergency_stop_acc_);
    double acc_cmd = applyRateFilter(emergency_stop_acc_, prev_acc_cmd_, dt, emergency_stop_jerk_);
    control_mode_ = ControlMode::ERROR;
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
      "closest not found. Emergency Stop! (dist_thr: %.3f [m], angle_thr = %.3f [rad])",
      closest_dist_thr_, closest_angle_thr_);
    RCLCPP_DEBUG(get_logger(), "[closest error]. vel: %3.3f, acc: %3.3f", vel_cmd, acc_cmd);
    return CtrlCmd{vel_cmd, acc_cmd};
  }

  const double current_vel = current_vel_ptr_->twist.linear.x;
  const double closest_vel = calcInterpolatedTargetValue(
    *trajectory_ptr_, *current_pose_ptr_, current_vel, closest_idx, "twist");
  int target_idx = DelayCompensator::getTrajectoryPointIndexAfterTimeDelay(
    *trajectory_ptr_, closest_idx, delay_compensation_time_, current_vel);
  auto target_pose = DelayCompensator::calcPoseAfterTimeDelay(
    *current_pose_ptr_, delay_compensation_time_, current_vel);
  double target_vel =
    calcInterpolatedTargetValue(*trajectory_ptr_, target_pose, closest_vel, target_idx, "twist");
  double target_acc =
    calcInterpolatedTargetValue(*trajectory_ptr_, target_pose, closest_vel, target_idx, "accel");
  const double pred_vel_in_target =
    predictedVelocityInTargetPoint(current_vel, prev_accel_, delay_compensation_time_);

  /* shift check */
  const Shift shift = getCurrentShift(target_vel);
  if (shift != prev_shift_) {pid_vel_.reset();}
  prev_shift_ = shift;

  const double pitch_filtered = lpf_pitch_.filter(getPitch(current_pose.orientation));
  writeDebugValues(
    dt, current_vel, pred_vel_in_target, target_vel, target_acc, shift, pitch_filtered,
    closest_idx);

  /* ===== STOPPED =====
   *
   * If the current velocity and target velocity is almost zero,
   * and the smooth stop is not working, enter the stop state.
   *
   * Outout velocity : "stop_state_vel_" (assumed to be zero, depending on the vehicle interface)
   * Output acceleration : "stop_state_acc_" with max_jerk limit. (depending on the vehicle interface)
   *
   */
  if (checkIsStopped(current_vel, target_vel, closest_idx)) {
    double acc_cmd = calcFilteredAcc(stop_state_acc_, pitch_filtered, dt, shift);
    control_mode_ = ControlMode::STOPPED;
    RCLCPP_DEBUG(get_logger(), "[Stopped]. vel: %3.3f, acc: %3.3f", stop_state_vel_, acc_cmd);
    return CtrlCmd{stop_state_vel_, acc_cmd};
  }

  /* ===== EMERGENCY STOP =====
   *
   * If the emergency flag is true, enter the emergency state.
   * The condition of the emergency is checked in checkEmergency() function.
   * The flag is reset when the vehicle is stopped.
   *
   * Outout velocity : "0" with maximum acceleration constraint
   * Output acceleration : "emergency_stop_acc_" with max_jerk limit.
   *
   */
  is_emergency_stop_ = checkEmergency(closest_idx);
  if (is_emergency_stop_) {
    double vel_cmd = applyRateFilter(0.0, prev_vel_cmd_, dt, emergency_stop_acc_);
    double acc_cmd = applyRateFilter(emergency_stop_acc_, prev_acc_cmd_, dt, emergency_stop_jerk_);
    control_mode_ = ControlMode::EMERGENCY_STOP;
    RCLCPP_ERROR(get_logger(), "[Emergency stop] vel: %3.3f, acc: %3.3f", 0.0, acc_cmd);
    return CtrlCmd{vel_cmd, acc_cmd};
  }

  /* ===== SMOOTH STOP =====
   *
   * If the vehicle veloicity & target velocity is low ehough, and there is a stop point nearby the ego vehicle,
   * enter the smooth stop state.
   *
   * Outout velocity : "target_vel" from the reference trajectory
   * Output acceleration : "emergency_stop_acc_" with max_jerk limit.
   *
   */
  is_smooth_stop_ = checkSmoothStop(closest_idx, target_vel);
  if (is_smooth_stop_) {
    if (!start_time_smooth_stop_) {
      start_time_smooth_stop_ = std::make_shared<rclcpp::Time>(get_clock()->now());
    }
    double smooth_stop_acc_cmd = calcSmoothStopAcc();
    double vel_cmd = 0.0;
    double acc_cmd = calcFilteredAcc(smooth_stop_acc_cmd, pitch_filtered, dt, shift);
    control_mode_ = ControlMode::SMOOTH_STOP;
    RCLCPP_DEBUG(
      get_logger(), "[smooth stop]: Smooth stopping. vel: %3.3f, acc: %3.3f", vel_cmd, acc_cmd);
    return CtrlCmd{vel_cmd, acc_cmd};
  }

  /* ===== FEEDBACK CONTROL =====
   *
   * Execute PID feedback control.
   *
   * Outout velocity : "target_vel" from the reference trajectory
   * Output acceleration : calculated by PID controller with max_acceleration & max_jerk limit.
   *
   */
  double feedback_acc_cmd = applyVelocityFeedback(target_acc, target_vel, dt, pred_vel_in_target);
  double acc_cmd = calcFilteredAcc(feedback_acc_cmd, pitch_filtered, dt, shift);
  control_mode_ = ControlMode::PID_CONTROL;
  RCLCPP_DEBUG(
    get_logger(),
    "[feedback control]  vel: %3.3f, acc: %3.3f, dt: %3.3f, vcurr: %3.3f, vref: %3.3f "
    "feedback_acc_cmd: %3.3f, shift: %d",
    target_vel, acc_cmd, dt, current_vel, target_vel, feedback_acc_cmd, shift);
  return CtrlCmd{target_vel, acc_cmd};
}

void VelocityController::resetHandling(const ControlMode control_mode)
{
  if (control_mode == ControlMode::EMERGENCY_STOP) {
    pid_vel_.reset();
    lpf_vel_error_.reset();
    resetSmoothStop();
    if (std::fabs(current_vel_ptr_->twist.linear.x) < stop_state_entry_ego_speed_) {
      is_emergency_stop_ = false;
    }
  } else if (control_mode == ControlMode::ERROR) {
    pid_vel_.reset();
    resetSmoothStop();
  } else if (control_mode == ControlMode::STOPPED) {
    pid_vel_.reset();
    lpf_vel_error_.reset();
    resetSmoothStop();
    resetEmergencyStop();
  } else if (control_mode == ControlMode::SMOOTH_STOP) {
    pid_vel_.reset();
    lpf_vel_error_.reset();
  } else if (control_mode == ControlMode::PID_CONTROL) {
    resetSmoothStop();
  }
}

void VelocityController::storeAccelCmd(const double accel)
{
  if (control_mode_ == ControlMode::PID_CONTROL) {
    // convert format
    autoware_control_msgs::msg::ControlCommandStamped cmd;
    cmd.header.stamp = get_clock()->now();
    cmd.control.acceleration = accel;

    // store published ctrl cmd
    ctrl_cmd_vec_.emplace_back(cmd);
  } else {
    // reset command
    ctrl_cmd_vec_.clear();
  }

  // remove unused ctrl cmd
  if (ctrl_cmd_vec_.size() <= 2) {
    return;
  }
  if (
    (get_clock()->now() - ctrl_cmd_vec_.at(1).header.stamp).seconds() > delay_compensation_time_)
  {
    ctrl_cmd_vec_.erase(ctrl_cmd_vec_.begin());
  }
}

void VelocityController::publishCtrlCmd(const double vel, const double acc)
{
  prev_acc_cmd_ = acc;
  prev_vel_cmd_ = vel;

  autoware_control_msgs::msg::ControlCommandStamped cmd;
  cmd.header.stamp = get_clock()->now();
  cmd.header.frame_id = "base_link";
  cmd.control.velocity = vel;
  cmd.control.acceleration = acc;
  pub_control_cmd_->publish(cmd);

  // calculate accleration from velocity
  if (prev_vel_ptr_) {
    const double dv = current_vel_ptr_->twist.linear.x - prev_vel_ptr_->twist.linear.x;
    const double dt = std::max(
      (rclcpp::Time(current_vel_ptr_->header.stamp) - rclcpp::Time(prev_vel_ptr_->header.stamp))
      .seconds(),
      1e-03);
    const double accel = dv / dt;
    // apply lowpass filter
    const double lowpass_accel = lowpass_filter(accel, prev_accel_, accel_lowpass_gain_);
    prev_accel_ = lowpass_accel;
    debug_values_.data.at(DBGVAL::CALCULATED_ACC) = lowpass_accel;
  }
  prev_vel_ptr_ = current_vel_ptr_;

  // debug
  debug_values_.data.at(DBGVAL::CTRL_MODE) = static_cast<double>(control_mode_);
  debug_values_.data.at(DBGVAL::ACCCMD_PUBLISHED) = acc;
  pub_debug_->publish(debug_values_);
  debug_values_.data.clear();
  debug_values_.data.resize(num_debug_values_, 0.0);
}

bool VelocityController::checkSmoothStop(const int closest, const double target_vel) const
{
  if (!enable_smooth_stop_) {
    return false;
  }

  bool is_large_target_speed = std::fabs(target_vel) > smooth_stop_param_.entry_target_speed;
  bool is_large_ego_speed =
    std::fabs(current_vel_ptr_->twist.linear.x) > smooth_stop_param_.entry_ego_speed;
  if (control_mode_ != ControlMode::SMOOTH_STOP && (is_large_target_speed || is_large_ego_speed)) {
    return false;
  }

  const double stop_dist = calcStopDistance(*trajectory_ptr_, closest);
  if (std::fabs(stop_dist) < smooth_stop_param_.stop_dist_) {
    return true;  // stop point is found around ego position.
  }
  return false;
}

bool VelocityController::checkIsStopped(double current_vel, double target_vel, int closest) const
{
  if (is_smooth_stop_) {
    return false;                     // stopping.

  }
  // Prevent a direct transition from PID_CONTROL to STOPPED without going through SMOOTH_STOP.
  if (control_mode_ == ControlMode::PID_CONTROL) {return false;}

  if (control_mode_ == ControlMode::STOPPED) {
    double dist = calcStopDistance(*trajectory_ptr_, closest);
    if (dist < stop_state_keep_stopping_dist_) {
      RCLCPP_DEBUG(
        get_logger(), "stop_dist = %f < %f : stop_state_keep_stopping_dist_. keep stopping.", dist,
        stop_state_keep_stopping_dist_);
      return true;
    }
    RCLCPP_DEBUG(get_logger(), "stop_dist = %f release stopping.", dist);
  }

  if (
    std::fabs(current_vel) < stop_state_entry_ego_speed_ &&
    std::fabs(target_vel) < stop_state_entry_target_speed_)
  {
    return true;
  } else {
    return false;
  }
}

bool VelocityController::checkEmergency(int closest) const
{
  // already in emergency.
  if (is_emergency_stop_) {
    return true;
  }

  // velocity is getting high when smoth stopping.
  bool has_smooth_exit_vel =
    std::fabs(current_vel_ptr_->twist.linear.x) > smooth_stop_param_.exit_ego_speed;
  if (is_smooth_stop_ && has_smooth_exit_vel) {
    return true;
  }

  // overshoot stop line.
  if (enable_overshoot_emergency_) {
    double stop_dist = calcStopDistance(*trajectory_ptr_, closest);
    if (stop_dist < -emergency_overshoot_dist_) {
      return true;
    }
  }
  return false;
}

double VelocityController::calcFilteredAcc(
  const double raw_acc, const double pitch, const double dt, const Shift shift)
{
  double acc_max_filtered = applyLimitFilter(raw_acc, max_acc_, min_acc_);
  debug_values_.data.at(DBGVAL::ACCCMD_ACC_LIMITED) = acc_max_filtered;

  // store ctrl cmd without slope filter
  storeAccelCmd(acc_max_filtered);

  double acc_slope_filtered = applySlopeCompensation(acc_max_filtered, pitch, shift);
  debug_values_.data.at(DBGVAL::ACCCMD_SLOPE_APPLIED) = acc_slope_filtered;

  double acc_jerk_filtered =
    applyRateFilter(acc_slope_filtered, prev_acc_cmd_, dt, max_jerk_, min_jerk_);
  debug_values_.data.at(DBGVAL::ACCCMD_JERK_LIMITED) = acc_jerk_filtered;

  return acc_jerk_filtered;
}

double VelocityController::getDt()
{
  double dt;
  if (!prev_control_time_) {
    dt = 1.0 / control_rate_;
    prev_control_time_ = std::make_shared<rclcpp::Time>(get_clock()->now());
  } else {
    dt = (get_clock()->now() - *prev_control_time_).seconds();
    *prev_control_time_ = get_clock()->now();
  }
  const double max_dt = 1.0 / control_rate_ * 2.0;
  const double min_dt = 1.0 / control_rate_ * 0.5;
  return std::max(std::min(dt, max_dt), min_dt);
}

enum VelocityController::Shift VelocityController::getCurrentShift(const double target_vel) const
{
  const double ep = 1.0e-5;
  return target_vel > ep ? Shift::Forward : (target_vel < -ep ? Shift::Reverse : prev_shift_);
}

double VelocityController::getPitch(const geometry_msgs::msg::Quaternion & quaternion) const
{
  Eigen::Quaterniond q(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
  Eigen::Vector3d v = q.toRotationMatrix() * Eigen::Vector3d::UnitX();
  double den = std::max(std::sqrt(v.x() * v.x() + v.y() * v.y()), 1.0E-8 /* avoid 0 divide */);
  double pitch = (-1.0) * std::atan2(v.z(), den);
  return pitch;
}

double VelocityController::calcSmoothStopAcc()
{
  const double elapsed_time = (get_clock()->now() - *start_time_smooth_stop_).seconds();

  double acc_cmd;
  const double t0 = smooth_stop_param_.weak_brake_time;
  const double t1 = t0 + smooth_stop_param_.increasing_brake_time;
  const double t2 = t1 + smooth_stop_param_.stop_brake_time;
  if (elapsed_time < t0) {
    acc_cmd = smooth_stop_param_.weak_brake_acc;
    RCLCPP_DEBUG(get_logger(), "[VC Smooth Stop] weak breaking! acc = %f", acc_cmd);
  } else if (elapsed_time < t1) {
    const double dt = elapsed_time - t0;
    acc_cmd = smooth_stop_param_.weak_brake_acc + smooth_stop_param_.increasing_brake_gradient * dt;
    RCLCPP_DEBUG(get_logger(), "[VC Smooth Stop] break increasing! acc = %f", acc_cmd);
  } else if (elapsed_time < t2) {
    acc_cmd = smooth_stop_param_.stop_brake_acc;
    RCLCPP_DEBUG(get_logger(), "[VC Smooth Stop] stop breaking! acc = %f", acc_cmd);
  } else {
    acc_cmd = smooth_stop_param_.stop_brake_acc;
    RCLCPP_DEBUG(get_logger(), "[VC Smooth Stop] finish smooth stopping! acc = %f", acc_cmd);
    resetSmoothStop();
    control_mode_ = ControlMode::STOPPED;  // set to STOPPED when smooth stop finished.
  }

  return acc_cmd;
}

double VelocityController::calcStopDistance(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const int origin) const
{
  constexpr double zero_velocity = std::numeric_limits<double>::epsilon();
  const double origin_velocity = trajectory.points.at(origin).twist.linear.x;
  double stop_dist = 0.0;

  // search forward
  if (std::fabs(origin_velocity) > zero_velocity) {
    for (int i = origin + 1; i < static_cast<int>(trajectory.points.size()) - 1; ++i) {
      const auto & p0 = trajectory.points.at(i);
      const auto & p1 = trajectory.points.at(i - 1);
      stop_dist += vcutils::calcDistance2D(p0.pose, p1.pose);
      if (std::fabs(p0.twist.linear.x) < zero_velocity) {
        break;
      }
    }
    return stop_dist;
  }

  // search backward
  for (int i = origin - 1; 0 < i; --i) {
    const auto & p0 = trajectory.points.at(i);
    const auto & p1 = trajectory.points.at(i + 1);
    if (std::fabs(p0.twist.linear.x) > zero_velocity) {
      break;
    }
    stop_dist -= vcutils::calcDistance2D(p0.pose, p1.pose);
  }
  return stop_dist;
}

double VelocityController::predictedVelocityInTargetPoint(
  const double current_vel, const double current_acc, const double delay_compensation_time)
{
  if (std::fabs(current_vel) < 1e-01) {
    // when velocity is low, no prediction
    return current_vel;
  }

  const double current_vel_abs = std::fabs(current_vel);
  if (ctrl_cmd_vec_.size() == 0) {
    const double pred_vel = current_vel + current_acc * delay_compensation_time;
    // avoid to change sign of current_vel and pred_vel
    return pred_vel > 0 ? std::copysign(pred_vel, current_vel) : 0.0;
  }

  double pred_vel = current_vel_abs;

  const auto past_delay_time =
    get_clock()->now() - rclcpp::Duration::from_seconds(delay_compensation_time);
  for (std::size_t i = 0; i < ctrl_cmd_vec_.size(); i++) {
    if (
      (get_clock()->now() - ctrl_cmd_vec_.at(i).header.stamp).seconds() <
      delay_compensation_time_)
    {
      if (i == 0) {
        // lack of data
        pred_vel =
          current_vel_abs + ctrl_cmd_vec_.at(i).control.acceleration * delay_compensation_time;
        return pred_vel > 0 ? std::copysign(pred_vel, current_vel) : 0.0;
      }
      // add velocity to accel * dt
      const double acc = ctrl_cmd_vec_.at(i - 1).control.acceleration;
      const auto curr_time_i = rclcpp::Time(ctrl_cmd_vec_.at(i).header.stamp);
      const double time_to_next_acc = std::min(
        (curr_time_i - rclcpp::Time(ctrl_cmd_vec_.at(i - 1).header.stamp)).seconds(),
        (curr_time_i - past_delay_time).seconds());
      pred_vel += acc * time_to_next_acc;
    }
  }

  const double last_acc = ctrl_cmd_vec_.at(ctrl_cmd_vec_.size() - 1).control.acceleration;
  const double time_to_current =
    (get_clock()->now() - ctrl_cmd_vec_.at(ctrl_cmd_vec_.size() - 1).header.stamp).seconds();
  pred_vel += last_acc * time_to_current;

  // avoid to change sign of current_vel and pred_vel
  return pred_vel > 0 ? std::copysign(pred_vel, current_vel) : 0.0;
}

double VelocityController::getPointValue(
  const autoware_planning_msgs::msg::TrajectoryPoint & point, const std::string & value_type)
{
  if (value_type == "twist") {
    return point.twist.linear.x;
  } else if (value_type == "accel") {
    return point.accel.linear.x;
  }

  RCLCPP_WARN(get_logger(), "value_type in VelocityController::getPointValue is invalid.");
  return 0.0;
}

double VelocityController::calcInterpolatedTargetValue(
  const autoware_planning_msgs::msg::Trajectory & traj,
  const geometry_msgs::msg::PoseStamped & curr_pose, const double current_vel, const int closest,
  const std::string & value_type)
{
  const double closest_value = getPointValue(traj.points.at(closest), value_type);

  if (traj.points.size() < 2) {
    return closest_value;
  }

  /* If the current position is at the edge of the reference trajectory, enable the edge value(vel/acc).
   * Else, calc secondary closest index for interpolation */
  int closest_second;
  const auto & closest_pos = traj.points.at(closest).pose;
  geometry_msgs::msg::Point rel_pos =
    vcutils::transformToRelativeCoordinate2D(curr_pose.pose.position, closest_pos);
  if (closest == 0) {
    if (rel_pos.x * current_vel <= 0.0) {
      return closest_value;
    }
    closest_second = 1;
  } else if (closest == static_cast<int>(traj.points.size()) - 1) {
    if (rel_pos.x * current_vel >= 0.0) {
      return closest_value;
    }
    closest_second = traj.points.size() - 2;
  } else {
    const double dist1 = vcutils::calcDistSquared2D(closest_pos, traj.points.at(closest - 1).pose);
    const double dist2 = vcutils::calcDistSquared2D(closest_pos, traj.points.at(closest + 1).pose);
    closest_second = dist1 < dist2 ? closest - 1 : closest + 1;
  }

  /* apply linear interpolation */
  const double dist_c1 = vcutils::calcDistance2D(curr_pose.pose, closest_pos);
  const double dist_c2 =
    vcutils::calcDistance2D(curr_pose.pose, traj.points.at(closest_second).pose);
  const double v1 = getPointValue(traj.points.at(closest), value_type);
  const double v2 = getPointValue(traj.points.at(closest_second), value_type);
  const double value_interp = (dist_c1 * v2 + dist_c2 * v1) / (dist_c1 + dist_c2);

  return value_interp;
}

double VelocityController::applyLimitFilter(
  const double input_val, const double max_val, const double min_val) const
{
  const double limitted_val = std::min(std::max(input_val, min_val), max_val);
  return limitted_val;
}

double VelocityController::applyRateFilter(
  const double input_val, const double prev_val, const double dt, const double max_val,
  const double min_val) const
{
  const double diff_raw = (input_val - prev_val) / dt;
  const double diff = std::min(std::max(diff_raw, min_val), max_val);
  const double filtered_val = prev_val + (diff * dt);
  return filtered_val;
}

double VelocityController::applyRateFilter(
  const double input_val, const double prev_val, const double dt, const double lim_val) const
{
  const double max_val = std::fabs(lim_val);
  const double min_val = -max_val;
  return applyRateFilter(input_val, prev_val, dt, max_val, min_val);
}
double VelocityController::applySlopeCompensation(
  const double input_acc, const double pitch, const Shift shift) const
{
  if (!enable_slope_compensation_) {
    return input_acc;
  }
  constexpr double gravity = 9.80665;
  const double pitch_limited = std::min(std::max(pitch, min_pitch_rad_), max_pitch_rad_);
  double sign = (shift == Shift::Forward) ? -1 : (shift == Shift::Reverse ? 1 : 0);
  double compensated_acc = input_acc + sign * gravity * std::sin(pitch_limited);
  return compensated_acc;
}

double VelocityController::applyVelocityFeedback(
  const double target_acc, const double target_vel, const double dt, const double current_vel)
{
  const double current_vel_abs = std::fabs(current_vel);
  const double target_vel_abs = std::fabs(target_vel);
  const bool enable_integration = (current_vel_abs > current_vel_threshold_pid_integrate_);
  const double error_vel_filtered = lpf_vel_error_.filter(target_vel_abs - current_vel_abs);

  std::vector<double> pid_contributions(3);
  const double pid_acc =
    pid_vel_.calculate(error_vel_filtered, dt, enable_integration, pid_contributions);
  const double feedbacked_acc = target_acc + pid_acc;

  debug_values_.data.at(DBGVAL::ACCCMD_PID_APPLIED) = feedbacked_acc;
  debug_values_.data.at(DBGVAL::ERROR_V_FILTERED) = error_vel_filtered;
  debug_values_.data.at(DBGVAL::ACCCMD_FB_P_CONTRIBUTION) = pid_contributions.at(0);  // P
  debug_values_.data.at(DBGVAL::ACCCMD_FB_I_CONTRIBUTION) = pid_contributions.at(1);  // I
  debug_values_.data.at(DBGVAL::ACCCMD_FB_D_CONTRIBUTION) = pid_contributions.at(2);  // D

  return feedbacked_acc;
}

void VelocityController::resetSmoothStop()
{
  is_smooth_stop_ = false;
  start_time_smooth_stop_ = nullptr;
}

void VelocityController::resetEmergencyStop() {is_emergency_stop_ = false;}

void VelocityController::writeDebugValues(
  const double dt, const double current_vel, const double predicted_velocity,
  const double target_vel, const double target_acc, const Shift shift, const double pitch,
  const int32_t closest)
{
  constexpr double rad2deg = 180.0 / 3.141592;
  const double raw_pitch = getPitch(current_pose_ptr_->pose.orientation);
  debug_values_.data.at(DBGVAL::DT) = dt;
  debug_values_.data.at(DBGVAL::CURR_V) = current_vel;
  debug_values_.data.at(DBGVAL::TARGET_V) = target_vel;
  debug_values_.data.at(DBGVAL::TARGET_ACC) = target_acc;
  debug_values_.data.at(DBGVAL::CLOSEST_V) = calcInterpolatedTargetValue(
    *trajectory_ptr_, *current_pose_ptr_, current_vel, closest, "twist");
  debug_values_.data.at(DBGVAL::PREDICTED_V) = predicted_velocity;
  debug_values_.data.at(DBGVAL::CLOSEST_ACC) = calcInterpolatedTargetValue(
    *trajectory_ptr_, *current_pose_ptr_, current_vel, closest, "accel");
  debug_values_.data.at(DBGVAL::SHIFT) = static_cast<double>(shift);
  debug_values_.data.at(DBGVAL::PITCH_LPFED_RAD) = pitch;
  debug_values_.data.at(DBGVAL::PITCH_LPFED_DEG) = pitch * rad2deg;
  debug_values_.data.at(DBGVAL::PITCH_RAW_RAD) = raw_pitch;
  debug_values_.data.at(DBGVAL::PITCH_RAW_DEG) = raw_pitch * rad2deg;
  debug_values_.data.at(DBGVAL::ERROR_V) = target_vel - current_vel;
}
