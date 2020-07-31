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

#include <velocity_controller/velocity_controller.h>

VelocityController::VelocityController()
: nh_(""),
  pnh_("~"),
  tf_listener_(tf_buffer_),
  is_smooth_stop_(false),
  is_emergency_stop_(false),
  prev_acc_cmd_(0.0)
{
  // parameters timer
  pnh_.param("control_rate", control_rate_, 30.0);

  // parameters to enable functions
  pnh_.param("enable_smooth_stop", enable_smooth_stop_, true);
  pnh_.param("enable_overshoot_emergency", enable_overshoot_emergency_, true);
  pnh_.param("enable_slope_compensation", enable_slope_compensation_, true);

  // parameters to find a closest waypoint
  pnh_.param("closest_waypoint_distance_threshold", closest_dist_thr_, 3.0);
  pnh_.param("closest_waypoint_angle_threshold", closest_angle_thr_, M_PI_4);

  // parameters for stop state
  pnh_.param("stop_state_vel", stop_state_vel_, 0.0);                                // [m/s]
  pnh_.param("stop_state_acc", stop_state_acc_, -2.0);                               // [m/s^2]
  pnh_.param("stop_state_entry_ego_speed", stop_state_entry_ego_speed_, 0.2);        // [m/s]
  pnh_.param("stop_state_entry_target_speed", stop_state_entry_target_speed_, 0.1);  // [m/s]

  // parameters for delay compensation
  pnh_.param("delay_compensation_time", delay_compensation_time_, 0.17);  // [sec]

  // parameters for emergency stop by this controller
  pnh_.param("emergency_stop_acc", emergency_stop_acc_, -2.0);             // [m/s^2]
  pnh_.param("emergency_stop_jerk", emergency_stop_jerk_, -1.5);           // [m/s^3]
  pnh_.param("emergency_overshoot_dist", emergency_overshoot_dist_, 1.5);  // [m]

  // parameters for smooth stop
  {
    auto & p = smooth_stop_param_;
    pnh_.param("smooth_stop/exit_ego_speed", p.exit_ego_speed, 2.0);                // [m/s]
    pnh_.param("smooth_stop/exit_target_speed", p.exit_target_speed, 2.0);          // [m/s]
    pnh_.param("smooth_stop/entry_ego_speed", p.entry_ego_speed, 1.0);              // [m/s]
    pnh_.param("smooth_stop/entry_target_speed", p.entry_target_speed, 1.0);        // [m/s]
    pnh_.param("smooth_stop/weak_brake_time", p.weak_brake_time, 3.0);              // [sec]
    pnh_.param("smooth_stop/weak_brake_acc", p.weak_brake_acc, -0.4);               // [m/s^2]
    pnh_.param("smooth_stop/increasing_brake_time", p.increasing_brake_time, 3.0);  // [sec]
    pnh_.param(
      "smooth_stop/increasing_brake_gradient", p.increasing_brake_gradient, -0.05);  // [m/s^3]
    pnh_.param("smooth_stop/stop_brake_time", p.stop_brake_time, 2.0);               // [sec]
    pnh_.param("smooth_stop/stop_brake_acc", p.stop_brake_acc, -1.7);                // [m/s^2]
    pnh_.param("smooth_stop/stop_dist_", p.stop_dist_, 3.0);                         // [m/s^2]
  }

  // parameters for acceleration limit
  pnh_.param("max_acc", max_acc_, 2.0);   // [m/s^2]
  pnh_.param("min_acc", min_acc_, -5.0);  // [m/s^2]

  // parameters for jerk limit
  pnh_.param("max_jerk", max_jerk_, 2.0);   // [m/s^3]
  pnh_.param("min_jerk", min_jerk_, -5.0);  // [m/s^3]

  // parameters for slope compensation
  pnh_.param("max_pitch_rad", max_pitch_rad_, 0.1);   // [rad]
  pnh_.param("min_pitch_rad", min_pitch_rad_, -0.1);  // [rad]

  pnh_.param(
    "pid_controller/current_vel_threshold_pid_integration", current_vel_threshold_pid_integrate_,
    0.5);  // [m/s]

  // subscriber, publisher and timer
  sub_current_vel_ =
    pnh_.subscribe("current_velocity", 1, &VelocityController::callbackCurrentVelocity, this);
  sub_trajectory_ =
    pnh_.subscribe("current_trajectory", 1, &VelocityController::callbackTrajectory, this);
  pub_control_cmd_ = pnh_.advertise<autoware_control_msgs::ControlCommandStamped>("control_cmd", 1);
  pub_debug_ = pnh_.advertise<std_msgs::Float32MultiArray>("debug_values", 1);
  timer_control_ = nh_.createTimer(
    ros::Duration(1.0 / control_rate_), &VelocityController::callbackTimerControl, this);

  // dynamic reconfigure
  dynamic_reconfigure_srv_.setCallback(
    boost::bind(&VelocityController::callbackConfig, this, _1, _2));

  // initialize PID gain
  {
    double kp, ki, kd;
    pnh_.param("pid_controller/kp", kp, 0.0);
    pnh_.param("pid_controller/ki", ki, 0.0);
    pnh_.param("pid_controller/kd", kd, 0.0);
    pid_vel_.setGains(kp, ki, kd);
  }

  // initialize PID limits
  {
    double max_pid, min_pid, max_p, min_p, max_i, min_i, max_d, min_d;
    pnh_.param("pid_controller/max_out", max_pid, 0.0);     // [m/s^2]
    pnh_.param("pid_controller/min_out", min_pid, 0.0);     // [m/s^2]
    pnh_.param("pid_controller/max_p_effort", max_p, 0.0);  // [m/s^2]
    pnh_.param("pid_controller/min_p_effort", min_p, 0.0);  // [m/s^2]
    pnh_.param("pid_controller/max_i_effort", max_i, 0.0);  // [m/s^2]
    pnh_.param("pid_controller/min_i_effort", min_i, 0.0);  // [m/s^2]
    pnh_.param("pid_controller/max_d_effort", max_d, 0.0);  // [m/s^2]
    pnh_.param("pid_controller/min_d_effort", min_d, 0.0);  // [m/s^2]
    pid_vel_.setLimits(max_pid, min_pid, max_p, min_p, max_i, min_i, max_d, min_d);
  }

  // set lowpass filter
  {
    double lpf_vel_error_gain;
    pnh_.param("pid_controller/lpf_vel_error_gain", lpf_vel_error_gain, 0.9);
    lpf_vel_error_.init(lpf_vel_error_gain);

    double lpf_pitch_gain;
    pnh_.param("lpf_pitch_gain", lpf_pitch_gain, 0.95);
    lpf_pitch_.init(lpf_pitch_gain);
  }

  debug_values_.data.clear();
  debug_values_.data.resize(num_debug_values_, 0.0);

  /* wait to get vehicle position */
  while (ros::ok()) {
    if (!updateCurrentPose(5.0)) {
      ROS_INFO("waiting map to base_link at initialize.");
    } else {
      break;
    }
  }
}

void VelocityController::callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
  current_vel_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(*msg);
}

void VelocityController::callbackTrajectory(const autoware_planning_msgs::TrajectoryConstPtr & msg)
{
  trajectory_ptr_ = std::make_shared<autoware_planning_msgs::Trajectory>(*msg);
}

bool VelocityController::getCurretPoseFromTF(
  const double timeout_sec, geometry_msgs::PoseStamped & ps)
{
  geometry_msgs::TransformStamped transform;
  try {
    transform =
      tf_buffer_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(timeout_sec));
  } catch (tf2::TransformException & ex) {
    ROS_WARN_DELAYED_THROTTLE(3.0, "cannot get map to base_link transform. %s", ex.what());
    return false;
  }
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  return true;
}

bool VelocityController::updateCurrentPose(const double timeout_sec)
{
  geometry_msgs::PoseStamped ps;
  if (!getCurretPoseFromTF(timeout_sec, ps)) {
    return false;
  }
  current_pose_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(ps);
  return true;
}

void VelocityController::callbackTimerControl(const ros::TimerEvent & event)
{
  const bool is_pose_updated = updateCurrentPose(0.0);

  /* gurad */
  if (!is_pose_updated || !current_pose_ptr_ || !current_vel_ptr_ || !trajectory_ptr_) {
    ROS_DEBUG(
      "Waiting topics, Publish stop command. pose_update: %d, pose: %d, vel: %d, trajectory: %d",
      is_pose_updated, current_pose_ptr_ != nullptr, current_vel_ptr_ != nullptr,
      trajectory_ptr_ != nullptr);
    controller_mode_ = ControlMode::INIT;
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
  resetHandling(controller_mode_);
}

void VelocityController::callbackConfig(
  const velocity_controller::VelocityControllerConfig & config, const uint32_t level)
{
  // closest waypoint threshold
  closest_dist_thr_ = config.closest_waypoint_distance_threshold;
  closest_angle_thr_ = config.closest_waypoint_angle_threshold;

  // stop state
  stop_state_vel_ = config.stop_state_velocity;
  stop_state_acc_ = config.stop_state_acc;
  stop_state_entry_ego_speed_ = config.stop_state_entry_ego_speed;
  stop_state_entry_target_speed_ = config.stop_state_entry_target_speed;

  // delay compensation
  delay_compensation_time_ = config.delay_compensation_time;

  // emergency stop by this controller
  emergency_stop_acc_ = config.emergency_stop_acc;
  emergency_stop_jerk_ = config.emergency_stop_jerk;

  // smooth stop
  smooth_stop_param_.exit_ego_speed = config.exit_ego_speed;
  smooth_stop_param_.entry_ego_speed = config.entry_ego_speed;
  smooth_stop_param_.exit_target_speed = config.exit_target_speed;
  smooth_stop_param_.entry_target_speed = config.entry_target_speed;
  smooth_stop_param_.weak_brake_time = config.weak_brake_time;
  smooth_stop_param_.weak_brake_acc = config.weak_brake_acc;
  smooth_stop_param_.increasing_brake_time = config.increasing_brake_time;
  smooth_stop_param_.increasing_brake_gradient = config.increasing_brake_gradient;
  smooth_stop_param_.stop_brake_time = config.stop_brake_time;
  smooth_stop_param_.stop_brake_acc = config.stop_brake_acc;

  // acceleration limit
  max_acc_ = config.max_acc;
  min_acc_ = config.min_acc;

  // jerk limit
  max_jerk_ = config.max_jerk;
  min_jerk_ = config.min_jerk;

  // slope compensation
  max_pitch_rad_ = config.max_pitch_rad;
  min_pitch_rad_ = config.min_pitch_rad;
  current_vel_threshold_pid_integrate_ = config.current_velocity_threshold_pid_integration;

  lpf_pitch_.init(config.lpf_pitch_gain);

  // velocity feedback
  pid_vel_.setGains(config.kp, config.ki, config.kd);
  pid_vel_.setLimits(
    config.max_out, config.min_out, config.max_p_effort, config.min_p_effort, config.max_i_effort,
    config.min_i_effort, config.max_d_effort, config.min_d_effort);
  lpf_vel_error_.init(config.lpf_velocity_error_gain);
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
        *trajectory_ptr_, current_pose, closest_angle_thr_, closest_dist_thr_, closest_idx)) {
    double vel_cmd = applyRateFilter(0.0, prev_vel_cmd_, dt, emergency_stop_acc_);
    double acc_cmd = applyRateFilter(emergency_stop_acc_, prev_acc_cmd_, dt, emergency_stop_jerk_);
    controller_mode_ = ControlMode::ERROR;
    ROS_ERROR_DELAYED_THROTTLE(
      5.0, "closest not found. Emergency Stop! (dist_thr: %.3f [m], angle_thr = %.3f [rad])",
      closest_dist_thr_, closest_angle_thr_);
    ROS_DEBUG("[closest error]. vel: %3.3f, acc: %3.3f", vel_cmd, acc_cmd);
    return CtrlCmd{vel_cmd, acc_cmd};
  }

  const double current_vel = current_vel_ptr_->twist.linear.x;
  const double closest_vel = calcInterpolatedTargetValue(
    *trajectory_ptr_, *current_pose_ptr_, current_vel, closest_idx, "twist");
  const double closest_acc = calcInterpolatedTargetValue(
    *trajectory_ptr_, *current_pose_ptr_, current_vel, closest_idx, "accel");
  int target_idx = DelayCompensator::getTrajectoryPointIndexAfterTimeDelay(
    *trajectory_ptr_, closest_idx, delay_compensation_time_, current_vel);
  auto target_pose = DelayCompensator::calcPoseAfterTimeDelay(
    *current_pose_ptr_, delay_compensation_time_, current_vel);
  double target_vel =
    calcInterpolatedTargetValue(*trajectory_ptr_, target_pose, closest_vel, target_idx, "twist");
  double target_acc =
    calcInterpolatedTargetValue(*trajectory_ptr_, target_pose, closest_vel, target_idx, "accel");
  const double pred_vel_in_target =
    predictedVelocityInTargetPoint(current_vel, closest_acc, delay_compensation_time_);

  /* shift check */
  const Shift shift = getCurrentShift(target_vel);
  if (shift != prev_shift_) pid_vel_.reset();
  prev_shift_ = shift;

  const double pitch_filtered = lpf_pitch_.filter(getPitch(current_pose.orientation));
  const double stop_dist = calcStopDistance(*trajectory_ptr_, closest_idx);
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
  if (checkIsStopped(current_vel, target_vel)) {
    double acc_cmd = calcFilteredAcc(stop_state_acc_, pitch_filtered, dt, shift);
    controller_mode_ = ControlMode::STOPPED;
    ROS_DEBUG("[Stopped]. vel: %3.3f, acc: %3.3f", stop_state_vel_, acc_cmd);
    return CtrlCmd{stop_state_vel_, acc_cmd};
  }

  /* ===== EMERGENCY STOP =====
   *
   * If the emergency flag is true, enter the emergency state.
   * The condition of the energency is checked in checkEmergency() function.
   * The flag is reset when the vehicle is stopped.
   *
   * Outout velocity : "0" with maximum acceleration constraint
   * Output acceleration : "emergency_stop_acc_" with max_jerk limit.
   *
   */
  is_emergency_stop_ = checkEmergency(closest_idx, target_vel);
  if (is_emergency_stop_) {
    double vel_cmd = applyRateFilter(0.0, prev_vel_cmd_, dt, emergency_stop_acc_);
    double acc_cmd = applyRateFilter(emergency_stop_acc_, prev_acc_cmd_, dt, emergency_stop_jerk_);
    controller_mode_ = ControlMode::EMERGENCY_STOP;
    ROS_ERROR("[Emergency stop] vel: %3.3f, acc: %3.3f", 0.0, acc_cmd);
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
      start_time_smooth_stop_ = std::make_shared<ros::Time>(ros::Time::now());
    }
    double smooth_stop_acc_cmd = calcSmoothStopAcc();
    double acc_cmd = calcFilteredAcc(smooth_stop_acc_cmd, pitch_filtered, dt, shift);
    controller_mode_ = ControlMode::SMOOTH_STOP;
    ROS_WARN_THROTTLE(
      0.5, "[smooth stop]: Smooth stopping. vel: %3.3f, acc: %3.3f", target_vel, acc_cmd);
    return CtrlCmd{target_vel, acc_cmd};
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
  controller_mode_ = ControlMode::PID_CONTROL;
  ROS_DEBUG(
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
  // convert format
  autoware_control_msgs::ControlCommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.control.acceleration = accel;

  // store published ctrl cmd
  ctrl_cmd_vec_.emplace_back(cmd);

  // remove unused ctrl cmd
  if (ctrl_cmd_vec_.size() <= 2) {
    return;
  }
  if ((ros::Time::now() - ctrl_cmd_vec_.at(1).header.stamp).toSec() > delay_compensation_time_) {
    ctrl_cmd_vec_.erase(ctrl_cmd_vec_.begin());
  }
}

void VelocityController::publishCtrlCmd(const double vel, const double acc)
{
  prev_acc_cmd_ = acc;
  prev_vel_cmd_ = vel;

  autoware_control_msgs::ControlCommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "base_link";
  cmd.control.velocity = vel;
  cmd.control.acceleration = acc;
  pub_control_cmd_.publish(cmd);

  // debug
  debug_values_.data.at(DBGVAL::CTRL_MODE) = static_cast<double>(controller_mode_);
  debug_values_.data.at(DBGVAL::ACCCMD_PUBLISHED) = acc;
  pub_debug_.publish(debug_values_);
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
  if (is_large_target_speed || is_large_ego_speed) {
    return false;
  }

  const double stop_dist = calcStopDistance(*trajectory_ptr_, closest);
  if (std::fabs(stop_dist) < smooth_stop_param_.stop_dist_) {
    return true;  // stop point is found around ego position.
  }
  return false;
}

bool VelocityController::checkIsStopped(double current_vel, double target_vel) const
{
  if (is_smooth_stop_) return false;  // stopping.

  if (
    std::fabs(current_vel) < stop_state_entry_ego_speed_ &&
    std::fabs(target_vel) < stop_state_entry_target_speed_) {
    return true;
  } else {
    return false;
  }
}

bool VelocityController::checkEmergency(int closest, double target_vel) const
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

  double acc_jerk_filtered =
    applyRateFilter(acc_max_filtered, prev_acc_cmd_, dt, max_jerk_, min_jerk_);
  debug_values_.data.at(DBGVAL::ACCCMD_JERK_LIMITED) = acc_jerk_filtered;

  // store ctrl cmd without slope filter
  storeAccelCmd(acc_jerk_filtered);

  double acc_slope_filtered = applySlopeCompensation(acc_jerk_filtered, pitch, shift);
  debug_values_.data.at(DBGVAL::ACCCMD_SLOPE_APPLIED) = acc_slope_filtered;
  return acc_slope_filtered;
}

double VelocityController::getDt()
{
  double dt;
  if (!prev_control_time_) {
    dt = 1.0 / control_rate_;
    prev_control_time_ = std::make_shared<ros::Time>(ros::Time::now());
  } else {
    dt = (ros::Time::now() - *prev_control_time_).toSec();
    *prev_control_time_ = ros::Time::now();
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

double VelocityController::getPitch(const geometry_msgs::Quaternion & quaternion) const
{
  Eigen::Quaterniond q(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
  Eigen::Vector3d v = q.toRotationMatrix() * Eigen::Vector3d::UnitX();
  double den = std::max(std::sqrt(v.x() * v.x() + v.y() * v.y()), 1.0E-8 /* avoid 0 divide */);
  double pitch = (-1.0) * std::atan2(v.z(), den);
  return pitch;
}

double VelocityController::calcSmoothStopAcc()
{
  const double elapsed_time = (ros::Time::now() - *start_time_smooth_stop_).toSec();

  double acc_cmd;
  const double t0 = smooth_stop_param_.weak_brake_time;
  const double t1 = t0 + smooth_stop_param_.increasing_brake_time;
  const double t2 = t1 + smooth_stop_param_.stop_brake_time;
  if (elapsed_time < t0) {
    acc_cmd = smooth_stop_param_.weak_brake_acc;
    ROS_DEBUG("[VC Smooth Stop] weak breaking! acc = %f", acc_cmd);
  } else if (elapsed_time < t1) {
    const double dt = elapsed_time - t0;
    acc_cmd = smooth_stop_param_.weak_brake_acc + smooth_stop_param_.increasing_brake_gradient * dt;
    ROS_DEBUG("[VC Smooth Stop] break increasing! acc = %f", acc_cmd);
  } else if (elapsed_time < t2) {
    acc_cmd = smooth_stop_param_.stop_brake_acc;
    ROS_DEBUG("[VC Smooth Stop] stop breaking! acc = %f", acc_cmd);
  } else {
    acc_cmd = smooth_stop_param_.stop_brake_acc;
    ROS_DEBUG("[VC Smooth Stop] finish smooth stopping! acc = %f", acc_cmd);
    resetSmoothStop();
  }

  return acc_cmd;
}

double VelocityController::calcStopDistance(
  const autoware_planning_msgs::Trajectory & trajectory, const int origin) const
{
  constexpr double zero_velocity = 0.01;
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
  const double current_vel, const double closest_acc, const double delay_compensation_time)
{
  if (ctrl_cmd_vec_.size() == 0) {
    const double pred_vel = current_vel + closest_acc * delay_compensation_time;
    // avoid to change sign of current_vel and pred_vel
    return current_vel * pred_vel > 0 ? pred_vel : 0.0;
  }

  double pred_vel = current_vel;

  const double past_delay_time = ros::Time::now().toSec() - delay_compensation_time;
  for (int i = 0; i < ctrl_cmd_vec_.size(); i++) {
    if ((ros::Time::now() - ctrl_cmd_vec_.at(i).header.stamp).toSec() < delay_compensation_time_) {
      if (i == 0) {
        // lack of data
        return current_vel + ctrl_cmd_vec_.at(i).control.acceleration * delay_compensation_time;
      }
      // add velocity to accel * dt
      const double current_acc = ctrl_cmd_vec_.at(i - 1).control.acceleration;
      const double time_to_next_acc = std::min(
        ctrl_cmd_vec_.at(i).header.stamp.toSec() - ctrl_cmd_vec_.at(i - 1).header.stamp.toSec(),
        ctrl_cmd_vec_.at(i).header.stamp.toSec() - past_delay_time);
      pred_vel += current_acc * time_to_next_acc;
    }
  }

  const double last_acc = ctrl_cmd_vec_.at(ctrl_cmd_vec_.size() - 1).control.acceleration;
  const double time_to_current =
    (ros::Time::now() - ctrl_cmd_vec_.at(ctrl_cmd_vec_.size() - 1).header.stamp).toSec();
  pred_vel += last_acc * time_to_current;

  // avoid to change sign of current_vel and pred_vel
  return current_vel * pred_vel > 0 ? pred_vel : 0.0;
}

double VelocityController::getPointValue(
  const autoware_planning_msgs::TrajectoryPoint & point, const std::string & value_type)
{
  if (value_type == "twist") {
    return point.twist.linear.x;
  } else if (value_type == "accel") {
    return point.accel.linear.x;
  }

  ROS_WARN_STREAM("value_type in VelocityController::getPointValue is invalid.");
  return 0.0;
}

double VelocityController::calcInterpolatedTargetValue(
  const autoware_planning_msgs::Trajectory & traj, const geometry_msgs::PoseStamped & curr_pose,
  const double current_vel, const int closest, const std::string & value_type)
{
  const double closest_value = getPointValue(traj.points.at(closest), value_type);

  if (traj.points.size() < 2) {
    return closest_value;
  }

  /* If the current position is at the edge of the reference trajectory, enable the edge value(vel/acc).
   * Else, calc secondary closest index for interpolation */
  int closest_second;
  const auto & closest_pos = traj.points.at(closest).pose;
  geometry_msgs::Point rel_pos =
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

void VelocityController::resetEmergencyStop() { is_emergency_stop_ = false; }

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
