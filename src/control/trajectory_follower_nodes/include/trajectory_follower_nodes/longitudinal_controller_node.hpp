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

#ifndef TRAJECTORY_FOLLOWER_NODES__LONGITUDINAL_CONTROLLER_NODE_HPP_
#define TRAJECTORY_FOLLOWER_NODES__LONGITUDINAL_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "autoware_auto_msgs/msg/longitudinal_command.hpp"
#include "autoware_auto_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "motion_common/trajectory_common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "trajectory_follower/debug_values.hpp"
#include "trajectory_follower/longitudinal_controller_utils.hpp"
#include "trajectory_follower/lowpass_filter.hpp"
#include "trajectory_follower/pid.hpp"
#include "trajectory_follower/smooth_stop.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower_nodes
{
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;
namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;
namespace motion_common = ::autoware::motion::motion_common;

class TRAJECTORY_FOLLOWER_PUBLIC LongitudinalController : public rclcpp::Node
{
public:
  explicit LongitudinalController(const rclcpp::NodeOptions & node_options);

private:
  struct Motion
  {
    double vel{0.0};
    double acc{0.0};
  };

  enum class Shift { Forward = 0, Reverse };

  struct ControlData
  {
    bool is_far_from_trajectory{false};
    size_t nearest_idx{0};  // nearest_idx = 0 when nearest_idx is not found with findNearestIdx
    Motion current_motion{};
    Shift shift{Shift::Forward};  // shift is used only to calculate the sign of pitch compensation
    double stop_dist{0.0};  // signed distance that is positive when car is before the stopline
    double slope_angle{0.0};
    double dt{0.0};
  };

  // ros variables
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_current_vel_;
  rclcpp::Subscription<autoware_auto_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Publisher<autoware_auto_msgs::msg::LongitudinalCommand>::SharedPtr pub_control_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_slope_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_debug_;
  rclcpp::TimerBase::SharedPtr timer_control_;

  tf2_ros::Buffer m_tf_buffer;
  //!< @brief tf listener
  tf2_ros::TransformListener m_tf_listener;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // pointers for ros topic
  std::shared_ptr<geometry_msgs::msg::TwistStamped> current_vel_ptr_{nullptr};
  std::shared_ptr<geometry_msgs::msg::TwistStamped> prev_vel_ptr_{nullptr};
  std::shared_ptr<autoware_auto_msgs::msg::Trajectory> trajectory_ptr_{nullptr};

  // vehicle info TODO get as param
  double wheel_base_;

  // control state
  enum class ControlState { DRIVE = 0, STOPPING, STOPPED, EMERGENCY };
  ControlState control_state_{ControlState::STOPPED};

  // timer callback
  double control_rate_;

  // delay compensation
  double delay_compensation_time_;

  // enable flags
  bool enable_smooth_stop_;
  bool enable_overshoot_emergency_;
  bool enable_slope_compensation_;

  // smooth stop transition
  struct StateTransitionParams
  {
    // drive
    double drive_state_stop_dist;
    double drive_state_offset_stop_dist;
    // stopping
    double stopping_state_stop_dist;
    // stop
    double stopped_state_entry_vel;
    double stopped_state_entry_acc;
    // emergency
    double emergency_state_overshoot_stop_dist;
    double emergency_state_traj_trans_dev;
    double emergency_state_traj_rot_dev;
  };
  StateTransitionParams state_transition_params_;

  // drive
  trajectory_follower::PIDController pid_vel_;
  std::shared_ptr<trajectory_follower::LowpassFilter1d> lpf_vel_error_{nullptr};
  double current_vel_threshold_pid_integrate_;

  // smooth stop
  trajectory_follower::SmoothStop smooth_stop_;

  // stop
  struct StoppedStateParams
  {
    double vel;
    double acc;
    double jerk;
  };
  StoppedStateParams stopped_state_params_;

  // emergency
  struct EmergencyStateParams
  {
    double vel;
    double acc;
    double jerk;
  };
  EmergencyStateParams emergency_state_params_;

  // acceleration limit
  double max_acc_;
  double min_acc_;

  // jerk limit
  double max_jerk_;
  double min_jerk_;

  // slope compensation
  bool use_traj_for_pitch_;
  std::shared_ptr<trajectory_follower::LowpassFilter1d> lpf_pitch_{nullptr};
  double max_pitch_rad_;
  double min_pitch_rad_;

  // 1st order lowpass filter for acceleration
  std::shared_ptr<trajectory_follower::LowpassFilter1d> lpf_acc_{nullptr};

  // current pose
  geometry_msgs::msg::PoseStamped::SharedPtr m_current_pose_ptr;
  // buffer of send command
  std::vector<autoware_auto_msgs::msg::LongitudinalCommand> ctrl_cmd_vec_;

  // for calculating dt
  std::shared_ptr<rclcpp::Time> prev_control_time_{nullptr};

  // shift mode
  Shift prev_shift_{Shift::Forward};

  // diff limit
  Motion prev_ctrl_cmd_{};      // with slope compensation
  Motion prev_raw_ctrl_cmd_{};  // without slope compensation
  std::vector<std::pair<rclcpp::Time, double>> vel_hist_;

  // debug values
  trajectory_follower::DebugValues debug_values_;

  std::shared_ptr<rclcpp::Time> last_running_time_{std::make_shared<rclcpp::Time>(this->now())};

  /**
   * @brief set current and previous velocity with received message
   */
  void callbackCurrentVelocity(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);

  /**
   * @brief set reference trajectory with received message
   */
  void callbackTrajectory(const autoware_auto_msgs::msg::Trajectory::ConstSharedPtr msg);

  /**
   * @brief compute control command, and publish periodically
   */
  void callbackTimerControl();

  /**
   * @brief calculate data for controllers whose type is ControlData
   */
  ControlData getControlData(const geometry_msgs::msg::Pose & current_pose);

  /**
   * @brief calculate control command in emergency state
   * @param [in] dt time between previous and current one
   */
  Motion calcEmergencyCtrlCmd(const double dt) const;

  /**
   * @brief update control state according to the current situation
   */
  ControlState updateControlState(
    const ControlState current_control_state, const ControlData & control_data);

  /**
   * @brief calculate control command based on the current control state
   */
  Motion calcCtrlCmd(
    const ControlState & current_control_state, const geometry_msgs::msg::Pose & current_pose,
    const ControlData & control_data);

  /**
   * @brief publish control command
   * @param [in] ctrl_cmd calculated control command to control velocity
   */
  void publishCtrlCmd(const Motion & ctrl_cmd, const double current_vel);

  /**
   * @brief publish debug data
   * @param [in] ctrl_cmd calculated control command to control velocity
   */
  void publishDebugData(const Motion & ctrl_cmd, const ControlData & control_data);

  /**
   * @brief calculate time between current and previous one
   */
  double getDt();

  /**
   * @brief calculate current velocity and acceleration
   */
  Motion getCurrentMotion() const;

  /**
   * @brief calculate direction (forward or backward) that vehicle moves
   * @param [in] nearest_idx nearest index on trajectory to vehicle
   */
  enum Shift getCurrentShift(const size_t nearest_idx) const;

  /**
   * @brief filter acceleration command with limitation of acceleration and jerk, and slope compensation
   * @param [in] raw_acc acceleration before filtered
   */
  double calcFilteredAcc(const double raw_acc, const ControlData & control_data);

  /**
   * @brief store acceleration command before slope compensation
   * @param [in] acceleration command before slope compensation
   */
  void storeAccelCmd(const double accel);

  /**
   * @brief add acceleration to compensate for slope
   * @param [in] acc acceleration before slope compensation
   * @param [in] pitch pitch angle (upward is negative)
   * @param [in] shift direction that vehicle move (forward or backward)
   */
  double applySlopeCompensation(const double acc, const double pitch, const Shift shift) const;

  /**
   * @brief interpolate trajectory point that is nearest to vehicle
   * @param [in] point vehicle position
   */
  autoware_auto_msgs::msg::TrajectoryPoint calcInterpolatedTargetValue(
    const autoware_auto_msgs::msg::Trajectory & traj, const geometry_msgs::msg::Point & point,
    const size_t nearest_idx) const;

  /**
   * @brief calculate predicted velocity after time delay based on past control commands
   */
  double predictedVelocityInTargetPoint(
    const Motion current_motion, const double delay_compensation_time) const;

  /**
   * @brief calculate velocity feedback with feed forward and pid controller
   * @param [in] target_motion reference velocity and acceleration. This acceleration will be used as feed forward.
   */
  double applyVelocityFeedback(
    const Motion target_motion, const double dt, const double current_vel);

  /**
   * @brief update variables for debugging about pitch
   */
  void updatePitchDebugValues(const double pitch, const double traj_pitch, const double raw_pitch);

  /**
   * @brief update variables for velocity and acceleration
   */
  void updateDebugVelAcc(
    const Motion & ctrl_cmd, const geometry_msgs::msg::Pose & current_pose,
    const ControlData & control_data);

  /**
   * @brief update current_pose from tf
   */
  void updateCurrentPose();
};
}  // namespace trajectory_follower_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER_NODES__LONGITUDINAL_CONTROLLER_NODE_HPP_
