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

#ifndef AUTOWARE_CONTROL_VELOCITY_CONTROLLER_H
#define AUTOWARE_CONTROL_VELOCITY_CONTROLLER_H

#include "delay_compensation.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "velocity_controller_mathutils.h"

#include <autoware_control_msgs/msg/control_command_stamped.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>

struct CtrlCmd
{
  double vel;
  double acc;
};

class VelocityController : public rclcpp::Node
{
public:
  VelocityController();
  ~VelocityController() = default;

private:
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_current_vel_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Publisher<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr pub_control_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_debug_;
  rclcpp::TimerBase::SharedPtr timer_control_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  //!< @brief tf listener

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // parameters
  // enabled flags
  bool enable_smooth_stop_;
  bool enable_overshoot_emergency_;
  bool enable_slope_compensation_;

  // timer callback
  double control_rate_;

  // closest waypoint
  double closest_dist_thr_;
  double closest_angle_thr_;

  // stop state
  double stop_state_vel_;
  double stop_state_acc_;
  double stop_state_entry_ego_speed_;
  double stop_state_entry_target_speed_;
  double stop_state_keep_stopping_dist_;

  // delay compensation
  double delay_compensation_time_;

  // emergency stop
  double emergency_stop_acc_;
  double emergency_stop_jerk_;
  double emergency_overshoot_dist_;

  // smooth stop
  struct SmoothStopParam
  {
    double exit_ego_speed;
    double entry_ego_speed;
    double exit_target_speed;
    double entry_target_speed;
    double weak_brake_time;
    double weak_brake_acc;
    double increasing_brake_gradient;
    double increasing_brake_time;
    double stop_brake_time;
    double stop_brake_acc;
    double stop_dist_;
  } smooth_stop_param_;

  // acceleration limit
  double max_acc_;
  double min_acc_;

  // jerk limit
  double max_jerk_;
  double min_jerk_;

  // slope compensation
  double max_pitch_rad_;
  double min_pitch_rad_;

  // velocity feedback
  double current_vel_threshold_pid_integrate_;

  // buffer of send command
  std::vector<autoware_control_msgs::msg::ControlCommandStamped> ctrl_cmd_vec_;

  // controller mode (0: init check, 1: PID, 2: Stop, 3: Smooth stop, 4: Emergency stop, 5: Error)
  enum class ControlMode {
    INIT = 0,
    PID_CONTROL = 1,
    STOPPED = 2,
    SMOOTH_STOP = 3,
    EMERGENCY_STOP = 4,
    ERROR = 5,
  };
  ControlMode control_mode_;

  // variables
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_ptr_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_vel_ptr_;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory_ptr_;

  // calculate acc
  geometry_msgs::msg::TwistStamped::ConstSharedPtr prev_vel_ptr_;
  double prev_accel_ = 0.0;
  const double accel_lowpass_gain_ = 0.2;

  // calculate dt
  std::shared_ptr<rclcpp::Time> prev_control_time_;

  // shift mode
  enum Shift {
    Forward = 0,
    Reverse,
  } prev_shift_;

  // smooth stop
  bool is_smooth_stop_ = false;
  bool is_emergency_stop_ = false;
  std::shared_ptr<rclcpp::Time> start_time_smooth_stop_;

  // diff limit
  double prev_acc_cmd_ = 0.0;
  double prev_vel_cmd_ = 0.0;

  // slope compensation
  Lpf1d lpf_pitch_;

  // velocity feedback
  PIDController pid_vel_;
  Lpf1d lpf_vel_error_;

  // methods
  void callbackCurrentVelocity(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void callbackTrajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
  void callbackTimerControl();

  void blockUntilVehiclePositionAvailable(const tf2::Duration & timeout);
  bool updateCurrentPose();

  double getPitch(const geometry_msgs::msg::Quaternion & quaternion) const;
  double getDt();
  enum Shift getCurrentShift(const double target_velocity) const;

  void setToIsStopped();

  /* check condition */
  bool checkIsStopped(double current_vel, double target_vel, int closest) const;
  bool checkSmoothStop(const int closest, const double target_vel) const;
  bool checkEmergency(int closest, double target_vel) const;

  /* reset flags */
  void resetHandling(ControlMode control_mode);
  void resetEmergencyStop();
  void resetSmoothStop();

  /* calc control command */
  CtrlCmd calcCtrlCmd();
  void storeAccelCmd(const double accel);
  void publishCtrlCmd(const double vel, const double acc);
  double predictedVelocityInTargetPoint(
    const double current_vel, const double current_acc, const double delay_compensation_time);
  double getPointValue(
    const autoware_planning_msgs::msg::TrajectoryPoint & point, const std::string & value_type);
  double calcInterpolatedTargetValue(
    const autoware_planning_msgs::msg::Trajectory & traj,
    const geometry_msgs::msg::PoseStamped & curr_pose, const double current_vel, const int closest,
    const std::string & value_type);
  double calcSmoothStopAcc();
  double calcFilteredAcc(
    const double raw_acc, const double pitch, const double dt, const Shift shift);
  double applyVelocityFeedback(
    const double target_acc, const double target_vel, const double dt, const double current_vel);
  double applyLimitFilter(const double input_val, const double max_val, const double min_val) const;
  double applyRateFilter(
    const double input_val, const double prev_val, const double dt, const double max_val,
    const double min_val) const;
  double applyRateFilter(
    const double input_val, const double prev_val, const double dt, const double lim_val) const;
  double applySlopeCompensation(const double acc, const double pitch, const Shift shift) const;
  double calcStopDistance(
    const autoware_planning_msgs::msg::Trajectory & trajectory, const int closest) const;

  /* Debug */
  mutable std_msgs::msg::Float32MultiArray debug_values_;
  enum DBGVAL {
    DT = 0,
    CURR_V = 1,
    TARGET_V = 2,
    TARGET_ACC = 3,
    CLOSEST_V = 4,
    CLOSEST_ACC = 5,
    SHIFT = 6,
    PITCH_LPFED_RAD = 7,
    PITCH_RAW_RAD = 8,
    PITCH_LPFED_DEG = 9,
    PITCH_RAW_DEG = 10,
    ERROR_V = 11,
    ERROR_V_FILTERED = 12,
    CTRL_MODE = 13,
    ACCCMD_PID_APPLIED = 14,
    ACCCMD_ACC_LIMITED = 15,
    ACCCMD_JERK_LIMITED = 16,
    ACCCMD_SLOPE_APPLIED = 17,
    ACCCMD_PUBLISHED = 18,
    ACCCMD_FB_P_CONTRIBUTION = 19,
    ACCCMD_FB_I_CONTRIBUTION = 20,
    ACCCMD_FB_D_CONTRIBUTION = 21,
    FLAG_SMOOTH_STOP = 22,
    FLAG_EMERGENCY_STOP = 23,
    PREDICTED_V = 24,
    CALCULATED_ACC = 25,
  };
  static constexpr unsigned int num_debug_values_ = 26;

  void writeDebugValues(
    const double dt, const double current_velocity, const double predicted_velocity,
    const double target_vel, const double target_acc, const Shift shift, const double pitch,
    const int32_t closest_waypoint_index);
};

#endif
