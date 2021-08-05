// Copyright 2018 Tier IV, Inc. All rights reserved.
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
#include <string>
#include <utility>
#include <vector>

#include "trajectory_follower_nodes/longitudinal_controller_node.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower_nodes
{
LongitudinalController::LongitudinalController(const rclcpp::NodeOptions & node_options)
: Node("longitudinal_controller", node_options), m_tf_buffer(this->get_clock()), m_tf_listener(
    m_tf_buffer)
{
  using std::placeholders::_1;

  // parameters timer
  control_rate_ = declare_parameter("control_rate").get<float64_t>();
  // const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  // TODO(Maxime CLEMENT) get the proper value (parameter?)
  wheel_base_ = 0.0;  // vehicle_info.wheel_base_m;

  // parameters for delay compensation
  delay_compensation_time_ = declare_parameter("delay_compensation_time").get<float64_t>();  // [s]

  // parameters to enable functions
  enable_smooth_stop_ = declare_parameter("enable_smooth_stop").get<bool8_t>();
  enable_overshoot_emergency_ = declare_parameter("enable_overshoot_emergency").get<bool8_t>();
  enable_slope_compensation_ = declare_parameter("enable_slope_compensation").get<bool8_t>();

  // parameters for state transition
  {
    auto & p = state_transition_params_;
    // drive
    p.drive_state_stop_dist = declare_parameter("drive_state_stop_dist").get<float64_t>();  // [m]
    p.drive_state_offset_stop_dist = declare_parameter(
      "drive_state_offset_stop_dist").get<float64_t>();  // [m]
    // stopping
    p.stopping_state_stop_dist =
      declare_parameter("stopping_state_stop_dist").get<float64_t>();  // [m]
    // stop
    p.stopped_state_entry_vel =
      declare_parameter("stopped_state_entry_vel").get<float64_t>();  // [m/s]
    p.stopped_state_entry_acc =
      declare_parameter("stopped_state_entry_acc").get<float64_t>();  // [m/s²]
    // emergency
    p.emergency_state_overshoot_stop_dist = declare_parameter(
      "emergency_state_overshoot_stop_dist").get<float64_t>();  // [m]
    p.emergency_state_traj_trans_dev = declare_parameter(
      "emergency_state_traj_trans_dev").get<float64_t>();  // [m]
    p.emergency_state_traj_rot_dev = declare_parameter(
      "emergency_state_traj_rot_dev").get<float64_t>();  // [m]
  }

  // parameters for drive state
  {
    // initialize PID gain
    const float64_t kp{declare_parameter("kp").get<float64_t>()};
    const float64_t ki{declare_parameter("ki").get<float64_t>()};
    const float64_t kd{declare_parameter("kd").get<float64_t>()};
    pid_vel_.setGains(kp, ki, kd);

    // initialize PID limits
    const float64_t max_pid{declare_parameter("max_out").get<float64_t>()};     // [m/s^2]
    const float64_t min_pid{declare_parameter("min_out").get<float64_t>()};     // [m/s^2]
    const float64_t max_p{declare_parameter("max_p_effort").get<float64_t>()};  // [m/s^2]
    const float64_t min_p{declare_parameter("min_p_effort").get<float64_t>()};  // [m/s^2]
    const float64_t max_i{declare_parameter("max_i_effort").get<float64_t>()};  // [m/s^2]
    const float64_t min_i{declare_parameter("min_i_effort").get<float64_t>()};  // [m/s^2]
    const float64_t max_d{declare_parameter("max_d_effort").get<float64_t>()};  // [m/s^2]
    const float64_t min_d{declare_parameter("min_d_effort").get<float64_t>()};  // [m/s^2]
    pid_vel_.setLimits(max_pid, min_pid, max_p, min_p, max_i, min_i, max_d, min_d);

    // set lowpass filter for vel error and pitch
    const float64_t lpf_vel_error_gain{declare_parameter("lpf_vel_error_gain").get<float64_t>()};
    lpf_vel_error_ =
      std::make_shared<trajectory_follower::LowpassFilter1d>(0.0, lpf_vel_error_gain);

    current_vel_threshold_pid_integrate_ = declare_parameter(
      "current_vel_threshold_pid_integration").get<float64_t>();  // [m/s]
  }

  // parameters for smooth stop state
  {
    const float64_t max_strong_acc{declare_parameter(
        "smooth_stop_max_strong_acc").get<float64_t>()};         // [m/s^2]
    const float64_t min_strong_acc{declare_parameter(
        "smooth_stop_min_strong_acc").get<float64_t>()};         // [m/s^2]
    const float64_t weak_acc{declare_parameter(
        "smooth_stop_weak_acc").get<float64_t>()};               // [m/s^2]
    const float64_t weak_stop_acc{declare_parameter(
        "smooth_stop_weak_stop_acc").get<float64_t>()};          // [m/s^2]
    const float64_t strong_stop_acc{declare_parameter(
        "smooth_stop_strong_stop_acc").get<float64_t>()};        // [m/s^2]

    const float64_t max_fast_vel{declare_parameter(
        "smooth_stop_max_fast_vel").get<float64_t>()};            // [m/s]
    const float64_t min_running_vel{declare_parameter(
        "smooth_stop_min_running_vel").get<float64_t>()};        // [m/s]
    const float64_t min_running_acc{declare_parameter(
        "smooth_stop_min_running_acc").get<float64_t>()};        // [m/s^2]
    const float64_t weak_stop_time{declare_parameter(
        "smooth_stop_weak_stop_time").get<float64_t>()};          // [s]

    const float64_t weak_stop_dist{declare_parameter(
        "smooth_stop_weak_stop_dist").get<float64_t>()};         // [m]
    const float64_t strong_stop_dist{declare_parameter(
        "smooth_stop_strong_stop_dist").get<float64_t>()};       // [m]

    smooth_stop_.setParams(
      max_strong_acc, min_strong_acc, weak_acc, weak_stop_acc, strong_stop_acc, max_fast_vel,
      min_running_vel, min_running_acc, weak_stop_time, weak_stop_dist, strong_stop_dist);
  }

  // parameters for stop state
  {
    auto & p = stopped_state_params_;
    p.vel = declare_parameter("stopped_vel").get<float64_t>();   // [m/s]
    p.acc = declare_parameter("stopped_acc").get<float64_t>();  // [m/s^2]
    p.jerk = declare_parameter("stopped_jerk").get<float64_t>();  // [m/s^3]
  }

  // parameters for emergency state
  {
    auto & p = emergency_state_params_;
    p.vel = declare_parameter("emergency_vel").get<float64_t>();     // [m/s]
    p.acc = declare_parameter("emergency_acc").get<float64_t>();    // [m/s^2]
    p.jerk = declare_parameter("emergency_jerk").get<float64_t>();  // [m/s^3]
  }

  // parameters for acceleration limit
  max_acc_ = declare_parameter("max_acc").get<float64_t>();   // [m/s^2]
  min_acc_ = declare_parameter("min_acc").get<float64_t>();  // [m/s^2]

  // parameters for jerk limit
  max_jerk_ = declare_parameter("max_jerk").get<float64_t>();   // [m/s^3]
  min_jerk_ = declare_parameter("min_jerk").get<float64_t>();  // [m/s^3]

  // parameters for slope compensation
  use_traj_for_pitch_ = declare_parameter("use_trajectory_for_pitch_calculation").get<bool8_t>();
  const float64_t lpf_pitch_gain{declare_parameter("lpf_pitch_gain").get<float64_t>()};
  lpf_pitch_ = std::make_shared<trajectory_follower::LowpassFilter1d>(0.0, lpf_pitch_gain);
  max_pitch_rad_ = declare_parameter("max_pitch_rad").get<float64_t>();   // [rad]
  min_pitch_rad_ = declare_parameter("min_pitch_rad").get<float64_t>();  // [rad]

  // subscriber, publisher
  sub_current_state_ = create_subscription<autoware_auto_msgs::msg::VehicleKinematicState>(
    "input/current_state", 1,
    std::bind(&LongitudinalController::callbackCurrentState, this, _1));
  sub_trajectory_ = create_subscription<autoware_auto_msgs::msg::Trajectory>(
    "input/current_trajectory", 1,
    std::bind(&LongitudinalController::callbackTrajectory, this, _1));
  pub_control_cmd_ = create_publisher<autoware_auto_msgs::msg::LongitudinalCommand>(
    "output/longitudinal_control_cmd", rclcpp::QoS{1});
  pub_slope_ = create_publisher<std_msgs::msg::Float32>(
    "output/slope_angle", rclcpp::QoS{1});
  pub_debug_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    "output/debug_values", rclcpp::QoS{1});

  // Timer
  {
    auto timer_callback = std::bind(&LongitudinalController::callbackTimerControl, this);
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<float64_t>(1.0 / control_rate_));
    timer_control_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
      this->get_clock(), period, std::move(timer_callback),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_control_, nullptr);
  }

  // set parameter callback
  set_param_res_ =
    this->add_on_set_parameters_callback(
    std::bind(
      &LongitudinalController::paramCallback, this,
      _1));

  // set lowpass filter for acc
  lpf_acc_ = std::make_shared<trajectory_follower::LowpassFilter1d>(0.0, 0.2);
}

void LongitudinalController::callbackCurrentState(
  const autoware_auto_msgs::msg::VehicleKinematicState::ConstSharedPtr msg)
{
  if (current_state_ptr_) {
    prev_state_ptr_ = current_state_ptr_;
  }
  current_state_ptr_ = std::make_shared<autoware_auto_msgs::msg::VehicleKinematicState>(*msg);
}

void LongitudinalController::callbackTrajectory(
  const autoware_auto_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  if (!trajectory_follower::longitudinal_utils::isValidTrajectory(*msg)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "received invalid trajectory. ignore.");
    return;
  }

  if (msg->points.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Unexpected trajectory size < 2. Ignored.");
    return;
  }

  trajectory_ptr_ = std::make_shared<autoware_auto_msgs::msg::Trajectory>(*msg);
}

rcl_interfaces::msg::SetParametersResult LongitudinalController::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto update_param = [&](const std::string & name, float64_t & v) {
      auto it = std::find_if(
        parameters.cbegin(), parameters.cend(),
        [&name](const rclcpp::Parameter & parameter) {return parameter.get_name() == name;});
      if (it != parameters.cend()) {
        v = it->as_double();
        return true;
      }
      return false;
    };

  // delay compensation
  update_param("delay_compensation_time", delay_compensation_time_);

  // state transition
  {
    auto & p = state_transition_params_;
    update_param("drive_state_stop_dist", p.drive_state_stop_dist);
    update_param("stopping_state_stop_dist", p.stopping_state_stop_dist);
    update_param("stopped_state_entry_vel", p.stopped_state_entry_vel);
    update_param("stopped_state_entry_acc", p.stopped_state_entry_acc);
    update_param("emergency_state_overshoot_stop_dist", p.emergency_state_overshoot_stop_dist);
    update_param("emergency_state_traj_trans_dev", p.emergency_state_traj_trans_dev);
    update_param("emergency_state_traj_rot_dev", p.emergency_state_traj_rot_dev);
  }

  // drive state
  {
    float64_t kp{get_parameter("kp").as_double()};
    float64_t ki{get_parameter("ki").as_double()};
    float64_t kd{get_parameter("kd").as_double()};
    update_param("kp", kp);
    update_param("ki", ki);
    update_param("kd", kd);
    pid_vel_.setGains(kp, ki, kd);

    float64_t max_pid{get_parameter("max_out").as_double()};
    float64_t min_pid{get_parameter("min_out").as_double()};
    float64_t max_p{get_parameter("max_p_effort").as_double()};
    float64_t min_p{get_parameter("min_p_effort").as_double()};
    float64_t max_i{get_parameter("max_i_effort").as_double()};
    float64_t min_i{get_parameter("min_i_effort").as_double()};
    float64_t max_d{get_parameter("max_d_effort").as_double()};
    float64_t min_d{get_parameter("min_d_effort").as_double()};
    update_param("max_out", max_pid);
    update_param("min_out", min_pid);
    update_param("max_p_effort", max_p);
    update_param("min_p_effort", min_p);
    update_param("max_i_effort", max_i);
    update_param("min_i_effort", min_i);
    update_param("max_d_effort", max_d);
    update_param("min_d_effort", min_d);
    pid_vel_.setLimits(max_pid, min_pid, max_p, min_p, max_i, min_i, max_d, min_d);

    update_param("current_vel_threshold_pid_integration", current_vel_threshold_pid_integrate_);
  }

  // stopping state
  {
    float64_t max_strong_acc{get_parameter("smooth_stop_max_strong_acc").as_double()};
    float64_t min_strong_acc{get_parameter("smooth_stop_min_strong_acc").as_double()};
    float64_t weak_acc{get_parameter("smooth_stop_weak_acc").as_double()};
    float64_t weak_stop_acc{get_parameter("smooth_stop_weak_stop_acc").as_double()};
    float64_t strong_stop_acc{get_parameter("smooth_stop_strong_stop_acc").as_double()};
    float64_t max_fast_vel{get_parameter("smooth_stop_max_fast_vel").as_double()};
    float64_t min_running_vel{get_parameter("smooth_stop_min_running_vel").as_double()};
    float64_t min_running_acc{get_parameter("smooth_stop_min_running_acc").as_double()};
    float64_t weak_stop_time{get_parameter("smooth_stop_weak_stop_time").as_double()};
    float64_t weak_stop_dist{get_parameter("smooth_stop_weak_stop_dist").as_double()};
    float64_t strong_stop_dist{get_parameter("smooth_stop_strong_stop_dist").as_double()};
    update_param("smooth_stop_max_strong_acc", max_strong_acc);
    update_param("smooth_stop_min_strong_acc", min_strong_acc);
    update_param("smooth_stop_weak_acc", weak_acc);
    update_param("smooth_stop_weak_stop_acc", weak_stop_acc);
    update_param("smooth_stop_strong_stop_acc", strong_stop_acc);
    update_param("smooth_stop_max_fast_vel", max_fast_vel);
    update_param("smooth_stop_min_running_vel", min_running_vel);
    update_param("smooth_stop_min_running_acc", min_running_acc);
    update_param("smooth_stop_weak_stop_time", weak_stop_time);
    update_param("smooth_stop_weak_stop_dist", weak_stop_dist);
    update_param("smooth_stop_strong_stop_dist", strong_stop_dist);
    smooth_stop_.setParams(
      max_strong_acc, min_strong_acc, weak_acc, weak_stop_acc, strong_stop_acc, max_fast_vel,
      min_running_vel, min_running_acc, weak_stop_time, weak_stop_dist, strong_stop_dist);
  }

  // stop state
  {
    auto & p = stopped_state_params_;
    update_param("stopped_vel", p.vel);
    update_param("stopped_acc", p.acc);
    update_param("stopped_jerk", p.jerk);
  }

  // emergency state
  {
    auto & p = emergency_state_params_;
    update_param("emergency_vel", p.vel);
    update_param("emergency_acc", p.acc);
    update_param("emergency_jerk", p.jerk);
  }

  // acceleration limit
  update_param("min_acc", min_acc_);

  // jerk limit
  update_param("max_jerk", max_jerk_);
  update_param("min_jerk", min_jerk_);

  // slope compensation
  update_param("max_pitch_rad", max_pitch_rad_);
  update_param("min_pitch_rad", min_pitch_rad_);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void LongitudinalController::callbackTimerControl()
{
  updateCurrentPose();
  // wait for initial pointers
  if (!current_state_ptr_ || !prev_state_ptr_ || !trajectory_ptr_ || !m_current_pose_ptr) {
    return;
  }

  // calculate current pose and contorl data
  const auto current_pose = m_current_pose_ptr->pose;
  const auto control_data = getControlData(current_pose);

  // self pose is far from trajectory
  if (control_data.is_far_from_trajectory) {
    control_state_ = ControlState::EMERGENCY;                           // update control state
    const Motion raw_ctrl_cmd = calcEmergencyCtrlCmd(control_data.dt);  // calculate control command
    prev_raw_ctrl_cmd_ = raw_ctrl_cmd;
    publishCtrlCmd(raw_ctrl_cmd, control_data.current_motion.vel);  // publish control command
    publishDebugData(raw_ctrl_cmd, control_data);     // publish debug data
    return;
  }

  // update control state
  control_state_ = updateControlState(control_state_, control_data);

  // calculate control command
  const Motion ctrl_cmd = calcCtrlCmd(control_state_, current_pose, control_data);

  // publish control command
  publishCtrlCmd(ctrl_cmd, control_data.current_motion.vel);

  // publish debug data
  publishDebugData(ctrl_cmd, control_data);
}

LongitudinalController::ControlData LongitudinalController::getControlData(
  const geometry_msgs::msg::Pose & current_pose)
{
  ControlData control_data{};

  // dt
  control_data.dt = getDt();

  // current velocity and acceleration
  control_data.current_motion = getCurrentMotion();

  // nearest idx
  const float64_t max_dist = state_transition_params_.emergency_state_traj_trans_dev;
  const float64_t max_yaw = state_transition_params_.emergency_state_traj_rot_dev;
  const auto nearest_idx_opt =
    motion_common::findNearestIndex(trajectory_ptr_->points, current_pose, max_dist, max_yaw);

  // return here if nearest index is not found
  if (!nearest_idx_opt) {
    control_data.is_far_from_trajectory = true;
    return control_data;
  }
  control_data.nearest_idx = *nearest_idx_opt;

  // shift
  control_data.shift = getCurrentShift(control_data.nearest_idx);
  if (control_data.shift != prev_shift_) {pid_vel_.reset();}
  prev_shift_ = control_data.shift;

  // distance to stopline
  control_data.stop_dist =
    trajectory_follower::longitudinal_utils::calcStopDistance(
    current_pose.position,
    *trajectory_ptr_);

  // pitch
  const float64_t raw_pitch = trajectory_follower::longitudinal_utils::getPitchByPose(
    current_pose.orientation);
  const float64_t traj_pitch = trajectory_follower::longitudinal_utils::getPitchByTraj(
    *trajectory_ptr_, control_data.nearest_idx, wheel_base_);
  control_data.slope_angle = use_traj_for_pitch_ ? traj_pitch : lpf_pitch_->filter(raw_pitch);
  updatePitchDebugValues(control_data.slope_angle, traj_pitch, raw_pitch);

  return control_data;
}

LongitudinalController::Motion LongitudinalController::calcEmergencyCtrlCmd(const float64_t dt)
const
{
  // These accelerations are without slope compensation
  const auto & p = emergency_state_params_;
  const float64_t vel = trajectory_follower::longitudinal_utils::applyDiffLimitFilter(
    p.vel, prev_raw_ctrl_cmd_.vel,
    dt, p.acc);
  const float64_t acc = trajectory_follower::longitudinal_utils::applyDiffLimitFilter(
    p.acc, prev_raw_ctrl_cmd_.acc,
    dt, p.jerk);

  auto clock = rclcpp::Clock{RCL_ROS_TIME};
  RCLCPP_ERROR_THROTTLE(
    get_logger(), clock, 3000,
    "[Emergency stop] vel: %3.3f, acc: %3.3f", vel, acc);

  return Motion{vel, acc};
}

LongitudinalController::ControlState LongitudinalController::updateControlState(
  const ControlState current_control_state, const ControlData & control_data)
{
  const float64_t current_vel = control_data.current_motion.vel;
  const float64_t current_acc = control_data.current_motion.acc;
  const float64_t stop_dist = control_data.stop_dist;

  // flags for state transition
  const auto & p = state_transition_params_;

  const bool8_t departure_condition_from_stopping =
    stop_dist > p.drive_state_stop_dist + p.drive_state_offset_stop_dist;
  const bool8_t departure_condition_from_stopped = stop_dist > p.drive_state_stop_dist;

  const bool8_t stopping_condition = stop_dist < p.stopping_state_stop_dist;
  if (
    std::fabs(current_vel) > p.stopped_state_entry_vel ||
    std::fabs(current_acc) > p.stopped_state_entry_acc)
  {
    last_running_time_ = std::make_shared<rclcpp::Time>(this->now());
  }
  const bool8_t stopped_condition =
    last_running_time_ ? (this->now() - *last_running_time_).seconds() > 0.5 : false;

  const bool8_t emergency_condition =
    enable_overshoot_emergency_ && stop_dist < -p.emergency_state_overshoot_stop_dist;

  // transit state
  if (current_control_state == ControlState::DRIVE) {
    if (emergency_condition) {
      return ControlState::EMERGENCY;
    }

    if (enable_smooth_stop_) {
      if (stopping_condition) {
        // predictions after input time delay
        const float64_t pred_vel_in_target =
          predictedVelocityInTargetPoint(control_data.current_motion, delay_compensation_time_);
        const float64_t pred_stop_dist =
          control_data.stop_dist -
          0.5 * (pred_vel_in_target + current_vel) * delay_compensation_time_;
        smooth_stop_.init(pred_vel_in_target, pred_stop_dist);
        return ControlState::STOPPING;
      }
    } else {
      if (stopped_condition && !departure_condition_from_stopped) {
        return ControlState::STOPPED;
      }
    }
  } else if (current_control_state == ControlState::STOPPING) {
    if (emergency_condition) {
      return ControlState::EMERGENCY;
    }

    if (stopped_condition) {
      return ControlState::STOPPED;
    }

    if (departure_condition_from_stopping) {
      pid_vel_.reset();
      lpf_vel_error_->reset(0.0);
      return ControlState::DRIVE;
    }
  } else if (current_control_state == ControlState::STOPPED) {
    if (departure_condition_from_stopped) {
      pid_vel_.reset();
      lpf_vel_error_->reset(0.0);
      return ControlState::DRIVE;
    }
  } else if (control_state_ == ControlState::EMERGENCY) {
    if (stopped_condition && !emergency_condition) {
      return ControlState::STOPPED;
    }
  }

  return current_control_state;
}

LongitudinalController::Motion LongitudinalController::calcCtrlCmd(
  const ControlState & current_control_state, const geometry_msgs::msg::Pose & current_pose,
  const ControlData & control_data)
{
  const size_t nearest_idx = control_data.nearest_idx;
  const float64_t current_vel = control_data.current_motion.vel;
  const float64_t current_acc = control_data.current_motion.acc;

  // velocity and acceleration command
  Motion raw_ctrl_cmd{};
  Motion target_motion{};
  if (current_control_state == ControlState::DRIVE) {
    const auto target_pose = trajectory_follower::longitudinal_utils::calcPoseAfterTimeDelay(
      current_pose, delay_compensation_time_, current_vel);
    const auto target_interpolated_point =
      calcInterpolatedTargetValue(*trajectory_ptr_, target_pose.position, nearest_idx);
    target_motion =
      Motion{target_interpolated_point.longitudinal_velocity_mps,
      target_interpolated_point.acceleration_mps2};

    const float64_t pred_vel_in_target =
      predictedVelocityInTargetPoint(control_data.current_motion, delay_compensation_time_);
    debug_values_.setValues(
      trajectory_follower::DebugValues::TYPE::PREDICTED_VEL,
      pred_vel_in_target);

    raw_ctrl_cmd.vel = target_motion.vel;
    raw_ctrl_cmd.acc = applyVelocityFeedback(target_motion, control_data.dt, pred_vel_in_target);
    RCLCPP_DEBUG(
      get_logger(),
      "[feedback control]  vel: %3.3f, acc: %3.3f, dt: %3.3f, v_curr: %3.3f, v_ref: %3.3f "
      "feedback_ctrl_cmd.ac: %3.3f",
      raw_ctrl_cmd.vel, raw_ctrl_cmd.acc, control_data.dt, current_vel, target_motion.vel,
      raw_ctrl_cmd.acc);
  } else if (current_control_state == ControlState::STOPPING) {
    raw_ctrl_cmd.acc = smooth_stop_.calculate(
      control_data.stop_dist, current_vel, current_acc, vel_hist_, delay_compensation_time_);
    raw_ctrl_cmd.vel = stopped_state_params_.vel;

    RCLCPP_DEBUG(
      get_logger(),
      "[smooth stop]: Smooth stopping. vel: %3.3f, acc: %3.3f",
      raw_ctrl_cmd.vel, raw_ctrl_cmd.acc);
  } else if (current_control_state == ControlState::STOPPED) {
    // This acceleration is without slope compensation
    const auto & p = stopped_state_params_;
    raw_ctrl_cmd.vel = trajectory_follower::longitudinal_utils::applyDiffLimitFilter(
      p.vel,
      prev_raw_ctrl_cmd_.vel,
      control_data.dt, p.acc);
    raw_ctrl_cmd.acc = trajectory_follower::longitudinal_utils::applyDiffLimitFilter(
      p.acc,
      prev_raw_ctrl_cmd_.acc,
      control_data.dt, p.jerk);

    RCLCPP_DEBUG(
      get_logger(), "[Stopped]. vel: %3.3f, acc: %3.3f",
      raw_ctrl_cmd.vel, raw_ctrl_cmd.acc);
  } else if (current_control_state == ControlState::EMERGENCY) {
    raw_ctrl_cmd = calcEmergencyCtrlCmd(control_data.dt);
  }

  // store acceleration without slope compensation
  prev_raw_ctrl_cmd_ = raw_ctrl_cmd;

  // apply slope compensation and filter acceleration and jerk
  const float64_t filtered_acc_cmd = calcFilteredAcc(raw_ctrl_cmd.acc, control_data);
  const Motion filtered_ctrl_cmd{raw_ctrl_cmd.vel, filtered_acc_cmd};

  // update debug visualization
  updateDebugVelAcc(target_motion, current_pose, control_data);

  return filtered_ctrl_cmd;
}

// Do not use nearest_idx here
void LongitudinalController::publishCtrlCmd(const Motion & ctrl_cmd, float64_t current_vel)
{
  // publish control command
  autoware_auto_msgs::msg::LongitudinalCommand cmd{};
  cmd.stamp = this->now();
  cmd.speed = static_cast<decltype(cmd.speed)>(ctrl_cmd.vel);
  cmd.acceleration = static_cast<decltype(cmd.acceleration)>(ctrl_cmd.acc);
  pub_control_cmd_->publish(cmd);

  // store current velocity history
  vel_hist_.push_back({this->now(), current_vel});
  while (vel_hist_.size() > static_cast<size_t>(control_rate_ * 0.5)) {
    vel_hist_.erase(vel_hist_.begin());
  }

  prev_ctrl_cmd_ = ctrl_cmd;
}

void LongitudinalController::publishDebugData(
  const Motion & ctrl_cmd, const ControlData & control_data)
{
  using trajectory_follower::DebugValues;
  // set debug values
  debug_values_.setValues(DebugValues::TYPE::DT, control_data.dt);
  debug_values_.setValues(DebugValues::TYPE::CALCULATED_ACC, control_data.current_motion.acc);
  debug_values_.setValues(DebugValues::TYPE::SHIFT, static_cast<float64_t>(control_data.shift));
  debug_values_.setValues(DebugValues::TYPE::STOP_DIST, control_data.stop_dist);
  debug_values_.setValues(DebugValues::TYPE::CONTROL_STATE, static_cast<float64_t>(control_state_));
  debug_values_.setValues(DebugValues::TYPE::ACC_CMD_PUBLISHED, ctrl_cmd.acc);

  // publish debug values
  std_msgs::msg::Float32MultiArray debug_msg{};
  // TODO(Maxime CLEMENT) use custom message with stamp
  // debug_msg.stamp = this->now();
  for (const auto & v : debug_values_.getValues()) {
    debug_msg.data.push_back(static_cast<decltype(debug_msg.data)::value_type>(v));
  }
  pub_debug_->publish(debug_msg);

  // slope angle
  std_msgs::msg::Float32 slope_msg{};
  // TODO(Maxime CLEMENT): use custom messages with a stamp field
  // autoware_debug_msgs::msg::Float32Stamped slope_msg{};
  // slope_msg.stamp = this->now();
  slope_msg.data = static_cast<decltype(slope_msg.data)>(control_data.slope_angle);
  pub_slope_->publish(slope_msg);
}

float64_t LongitudinalController::getDt()
{
  float64_t dt;
  if (!prev_control_time_) {
    dt = 1.0 / control_rate_;
    prev_control_time_ = std::make_shared<rclcpp::Time>(this->now());
  } else {
    dt = (this->now() - *prev_control_time_).seconds();
    *prev_control_time_ = this->now();
  }
  const float64_t max_dt = 1.0 / control_rate_ * 2.0;
  const float64_t min_dt = 1.0 / control_rate_ * 0.5;
  return std::max(std::min(dt, max_dt), min_dt);
}

LongitudinalController::Motion LongitudinalController::getCurrentMotion() const
{
  const float64_t dv = current_state_ptr_->state.longitudinal_velocity_mps -
    prev_state_ptr_->state.longitudinal_velocity_mps;
  const float64_t dt =
    std::max(
    (rclcpp::Time(current_state_ptr_->header.stamp) -
    rclcpp::Time(prev_state_ptr_->header.stamp)).seconds(), 1e-03);
  const float64_t accel = dv / dt;

  const float64_t current_vel = current_state_ptr_->state.longitudinal_velocity_mps;
  const float64_t current_acc = lpf_acc_->filter(accel);

  return Motion{current_vel, current_acc};
}

enum LongitudinalController::Shift LongitudinalController::getCurrentShift(const size_t nearest_idx)
const
{
  constexpr float64_t epsilon = 1e-5;

  const float64_t target_vel = trajectory_ptr_->points.at(nearest_idx).longitudinal_velocity_mps;

  if (target_vel > epsilon) {
    return Shift::Forward;
  } else if (target_vel < -epsilon) {
    return Shift::Reverse;
  }

  return prev_shift_;
}

float64_t LongitudinalController::calcFilteredAcc(
  const float64_t raw_acc,
  const ControlData & control_data)
{
  using trajectory_follower::DebugValues;
  const float64_t acc_max_filtered = ::motion::motion_common::clamp(raw_acc, min_acc_, max_acc_);
  debug_values_.setValues(DebugValues::TYPE::ACC_CMD_ACC_LIMITED, acc_max_filtered);

  // store ctrl cmd without slope filter
  storeAccelCmd(acc_max_filtered);

  const float64_t acc_slope_filtered =
    applySlopeCompensation(acc_max_filtered, control_data.slope_angle, control_data.shift);
  debug_values_.setValues(DebugValues::TYPE::ACC_CMD_SLOPE_APPLIED, acc_slope_filtered);

  // This jerk filter must be applied after slope compensation
  const float64_t acc_jerk_filtered = trajectory_follower::longitudinal_utils::applyDiffLimitFilter(
    acc_slope_filtered, prev_ctrl_cmd_.acc, control_data.dt, max_jerk_, min_jerk_);
  debug_values_.setValues(DebugValues::TYPE::ACC_CMD_JERK_LIMITED, acc_jerk_filtered);

  return acc_jerk_filtered;
}

void LongitudinalController::storeAccelCmd(const float64_t accel)
{
  if (control_state_ == ControlState::DRIVE) {
    // convert format
    autoware_auto_msgs::msg::LongitudinalCommand cmd;
    cmd.stamp = this->now();
    cmd.acceleration = static_cast<decltype(cmd.acceleration)>(accel);

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
    (this->now() - ctrl_cmd_vec_.at(1).stamp).seconds() > delay_compensation_time_)
  {
    ctrl_cmd_vec_.erase(ctrl_cmd_vec_.begin());
  }
}

float64_t LongitudinalController::applySlopeCompensation(
  const float64_t input_acc, const float64_t pitch, const Shift shift) const
{
  if (!enable_slope_compensation_) {
    return input_acc;
  }
  const float64_t pitch_limited = std::min(std::max(pitch, min_pitch_rad_), max_pitch_rad_);

  // Acceleration command is always positive independent of direction (= shift) when car is running
  float64_t sign = (shift == Shift::Forward) ? -1 : (shift == Shift::Reverse ? 1 : 0);
  float64_t compensated_acc = input_acc + sign * 9.81 * std::sin(pitch_limited);
  return compensated_acc;
}

autoware_auto_msgs::msg::TrajectoryPoint LongitudinalController::calcInterpolatedTargetValue(
  const autoware_auto_msgs::msg::Trajectory & traj, const geometry_msgs::msg::Point & point,
  const size_t nearest_idx) const
{
  if (traj.points.size() == 1) {
    return traj.points.at(0);
  }

  // If the current position is not within the reference trajectory, enable the edge value.
  // Else, apply linear interpolation
  if (nearest_idx == 0) {
    if (motion_common::calcSignedArcLength(traj.points, point, 0) > 0) {
      return traj.points.at(0);
    }
  }
  if (nearest_idx == traj.points.size() - 1) {
    if (motion_common::calcSignedArcLength(traj.points, point, traj.points.size() - 1) < 0) {
      return traj.points.at(traj.points.size() - 1);
    }
  }

  // apply linear interpolation
  return trajectory_follower::longitudinal_utils::lerpTrajectoryPoint(traj.points, point);
}

float64_t LongitudinalController::predictedVelocityInTargetPoint(
  const Motion current_motion, const float64_t delay_compensation_time) const
{
  const float64_t current_vel = current_motion.vel;
  const float64_t current_acc = current_motion.acc;

  if (std::fabs(current_vel) < 1e-01) {  // when velocity is low, no prediction
    return current_vel;
  }

  const float64_t current_vel_abs = std::fabs(current_vel);
  if (ctrl_cmd_vec_.size() == 0) {
    const float64_t pred_vel = current_vel + current_acc * delay_compensation_time;
    // avoid to change sign of current_vel and pred_vel
    return pred_vel > 0 ? std::copysign(pred_vel, current_vel) : 0.0;
  }

  float64_t pred_vel = current_vel_abs;

  const auto past_delay_time =
    this->now() - rclcpp::Duration::from_seconds(delay_compensation_time);
  for (std::size_t i = 0; i < ctrl_cmd_vec_.size(); ++i) {
    if (
      (this->now() - ctrl_cmd_vec_.at(i).stamp).seconds() <
      delay_compensation_time_)
    {
      if (i == 0) {
        // size of ctrl_cmd_vec_ is less than delay_compensation_time_
        pred_vel =
          current_vel_abs + static_cast<float64_t>(ctrl_cmd_vec_.at(i).acceleration) *
          delay_compensation_time;
        return pred_vel > 0 ? std::copysign(pred_vel, current_vel) : 0.0;
      }
      // add velocity to accel * dt
      const float64_t acc = ctrl_cmd_vec_.at(i - 1).acceleration;
      const auto curr_time_i = rclcpp::Time(ctrl_cmd_vec_.at(i).stamp);
      const float64_t time_to_next_acc = std::min(
        (curr_time_i - rclcpp::Time(ctrl_cmd_vec_.at(i - 1).stamp)).seconds(),
        (curr_time_i - past_delay_time).seconds());
      pred_vel += acc * time_to_next_acc;
    }
  }

  const float64_t last_acc = ctrl_cmd_vec_.at(ctrl_cmd_vec_.size() - 1).acceleration;
  const float64_t time_to_current =
    (this->now() - ctrl_cmd_vec_.at(ctrl_cmd_vec_.size() - 1).stamp).seconds();
  pred_vel += last_acc * time_to_current;

  // avoid to change sign of current_vel and pred_vel
  return pred_vel > 0 ? std::copysign(pred_vel, current_vel) : 0.0;
}

float64_t LongitudinalController::applyVelocityFeedback(
  const Motion target_motion, const float64_t dt, const float64_t current_vel)
{
  using trajectory_follower::DebugValues;
  const float64_t current_vel_abs = std::fabs(current_vel);
  const float64_t target_vel_abs = std::fabs(target_motion.vel);
  const bool8_t enable_integration = (current_vel_abs > current_vel_threshold_pid_integrate_);
  const float64_t error_vel_filtered = lpf_vel_error_->filter(target_vel_abs - current_vel_abs);

  std::vector<float64_t> pid_contributions(3);
  const float64_t pid_acc =
    pid_vel_.calculate(error_vel_filtered, dt, enable_integration, pid_contributions);
  const float64_t feedback_acc = target_motion.acc + pid_acc;

  debug_values_.setValues(DebugValues::TYPE::ACC_CMD_PID_APPLIED, feedback_acc);
  debug_values_.setValues(DebugValues::TYPE::ERROR_VEL_FILTERED, error_vel_filtered);
  debug_values_.setValues(
    DebugValues::TYPE::ACC_CMD_FB_P_CONTRIBUTION, pid_contributions.at(0));  // P
  debug_values_.setValues(
    DebugValues::TYPE::ACC_CMD_FB_I_CONTRIBUTION, pid_contributions.at(1));  // I
  debug_values_.setValues(
    DebugValues::TYPE::ACC_CMD_FB_D_CONTRIBUTION, pid_contributions.at(2));  // D

  return feedback_acc;
}

void LongitudinalController::updatePitchDebugValues(
  const float64_t pitch, const float64_t traj_pitch, const float64_t raw_pitch)
{
  using trajectory_follower::DebugValues;
  const float64_t to_degrees = (180.0 / static_cast<float64_t>(autoware::common::types::PI));
  debug_values_.setValues(DebugValues::TYPE::PITCH_LPF_RAD, pitch);
  debug_values_.setValues(DebugValues::TYPE::PITCH_LPF_DEG, pitch * to_degrees);
  debug_values_.setValues(DebugValues::TYPE::PITCH_RAW_RAD, raw_pitch);
  debug_values_.setValues(DebugValues::TYPE::PITCH_RAW_DEG, raw_pitch * to_degrees);
  debug_values_.setValues(DebugValues::TYPE::PITCH_RAW_TRAJ_RAD, traj_pitch);
  debug_values_.setValues(
    DebugValues::TYPE::PITCH_RAW_TRAJ_DEG, traj_pitch * to_degrees);
}

void LongitudinalController::updateDebugVelAcc(
  const Motion & target_motion, const geometry_msgs::msg::Pose & current_pose,
  const ControlData & control_data)
{
  using trajectory_follower::DebugValues;
  const float64_t current_vel = control_data.current_motion.vel;
  const size_t nearest_idx = control_data.nearest_idx;

  const auto interpolated_point =
    calcInterpolatedTargetValue(*trajectory_ptr_, current_pose.position, nearest_idx);

  debug_values_.setValues(DebugValues::TYPE::CURRENT_VEL, current_vel);
  debug_values_.setValues(DebugValues::TYPE::TARGET_VEL, target_motion.vel);
  debug_values_.setValues(DebugValues::TYPE::TARGET_ACC, target_motion.acc);
  debug_values_.setValues(
    DebugValues::TYPE::NEAREST_VEL,
    interpolated_point.longitudinal_velocity_mps);
  debug_values_.setValues(DebugValues::TYPE::NEAREST_ACC, interpolated_point.acceleration_mps2);
  debug_values_.setValues(DebugValues::TYPE::ERROR_VEL, target_motion.vel - current_vel);
}

void LongitudinalController::updateCurrentPose()
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = m_tf_buffer.lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/,
      "cannot get map to base_link transform. %s", ex.what());
    return;
  }

  geometry_msgs::msg::PoseStamped ps;
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  m_current_pose_ptr = std::make_shared<geometry_msgs::msg::PoseStamped>(ps);
}
}  // namespace trajectory_follower_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::motion::control::trajectory_follower_nodes::LongitudinalController)
