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
#include "autoware_auto_msgs/msg/vehicle_kinematic_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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

/// \class LongitudinalController
/// \brief The node class used for generating longitudinal control commands (velocity/acceleration)
class TRAJECTORY_FOLLOWER_PUBLIC LongitudinalController : public rclcpp::Node
{
public:
  explicit LongitudinalController(const rclcpp::NodeOptions & node_options);

private:
  struct Motion
  {
    float64_t vel{0.0};
    float64_t acc{0.0};
  };

  enum class Shift { Forward = 0, Reverse };

  struct ControlData
  {
    bool8_t is_far_from_trajectory{false};
    size_t nearest_idx{0};  // nearest_idx = 0 when nearest_idx is not found with findNearestIdx
    Motion current_motion{};
    Shift shift{Shift::Forward};  // shift is used only to calculate the sign of pitch compensation
    float64_t stop_dist{0.0};  // signed distance that is positive when car is before the stopline
    float64_t slope_angle{0.0};
    float64_t dt{0.0};
  };

  // ros variables
  rclcpp::Subscription<autoware_auto_msgs::msg::VehicleKinematicState>::SharedPtr
    sub_current_state_;
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
  std::shared_ptr<autoware_auto_msgs::msg::VehicleKinematicState> current_state_ptr_{nullptr};
  std::shared_ptr<autoware_auto_msgs::msg::VehicleKinematicState> prev_state_ptr_{nullptr};
  std::shared_ptr<autoware_auto_msgs::msg::Trajectory> trajectory_ptr_{nullptr};

  // vehicle info TODO get as param
  float64_t wheel_base_;

  // control state
  enum class ControlState { DRIVE = 0, STOPPING, STOPPED, EMERGENCY };
  ControlState control_state_{ControlState::STOPPED};

  // timer callback
  float64_t control_rate_;

  // delay compensation
  float64_t delay_compensation_time_;

  // enable flags
  bool8_t enable_smooth_stop_;
  bool8_t enable_overshoot_emergency_;
  bool8_t enable_slope_compensation_;

  // smooth stop transition
  struct StateTransitionParams
  {
    // drive
    float64_t drive_state_stop_dist;
    float64_t drive_state_offset_stop_dist;
    // stopping
    float64_t stopping_state_stop_dist;
    // stop
    float64_t stopped_state_entry_vel;
    float64_t stopped_state_entry_acc;
    // emergency
    float64_t emergency_state_overshoot_stop_dist;
    float64_t emergency_state_traj_trans_dev;
    float64_t emergency_state_traj_rot_dev;
  };
  StateTransitionParams state_transition_params_;

  // drive
  trajectory_follower::PIDController pid_vel_;
  std::shared_ptr<trajectory_follower::LowpassFilter1d> lpf_vel_error_{nullptr};
  float64_t current_vel_threshold_pid_integrate_;

  // smooth stop
  trajectory_follower::SmoothStop smooth_stop_;

  // stop
  struct StoppedStateParams
  {
    float64_t vel;
    float64_t acc;
    float64_t jerk;
  };
  StoppedStateParams stopped_state_params_;

  // emergency
  struct EmergencyStateParams
  {
    float64_t vel;
    float64_t acc;
    float64_t jerk;
  };
  EmergencyStateParams emergency_state_params_;

  // acceleration limit
  float64_t max_acc_;
  float64_t min_acc_;

  // jerk limit
  float64_t max_jerk_;
  float64_t min_jerk_;

  // slope compensation
  bool8_t use_traj_for_pitch_;
  std::shared_ptr<trajectory_follower::LowpassFilter1d> lpf_pitch_{nullptr};
  float64_t max_pitch_rad_;
  float64_t min_pitch_rad_;

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
  std::vector<std::pair<rclcpp::Time, float64_t>> vel_hist_;

  // debug values
  trajectory_follower::DebugValues debug_values_;

  std::shared_ptr<rclcpp::Time> last_running_time_{std::make_shared<rclcpp::Time>(this->now())};

  /**
   * @brief set current and previous velocity with received message
   */
  void callbackCurrentState(
    const autoware_auto_msgs::msg::VehicleKinematicState::ConstSharedPtr msg);

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
  Motion calcEmergencyCtrlCmd(const float64_t dt) const;

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
   * @param [in] current_vel current velocity of the vehicle
   */
  void publishCtrlCmd(const Motion & ctrl_cmd, const float64_t current_vel);

  /**
   * @brief publish debug data
   * @param [in] ctrl_cmd calculated control command to control velocity
   * @param [in] control_data data for control calculation
   */
  void publishDebugData(const Motion & ctrl_cmd, const ControlData & control_data);

  /**
   * @brief calculate time between current and previous one
   */
  float64_t getDt();

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
   * @param [in] control_data data for control calculation
   */
  float64_t calcFilteredAcc(const float64_t raw_acc, const ControlData & control_data);

  /**
   * @brief store acceleration command before slope compensation
   * @param [in] accel command before slope compensation
   */
  void storeAccelCmd(const float64_t accel);

  /**
   * @brief add acceleration to compensate for slope
   * @param [in] acc acceleration before slope compensation
   * @param [in] pitch pitch angle (upward is negative)
   * @param [in] shift direction that vehicle move (forward or backward)
   */
  float64_t applySlopeCompensation(
    const float64_t acc, const float64_t pitch,
    const Shift shift) const;

  /**
   * @brief interpolate trajectory point that is nearest to vehicle
   * @param [in] traj reference trajectory
   * @param [in] point vehicle position
   * @param [in] nearest_idx index of the trajectory point nearest to the vehicle position
   */
  autoware_auto_msgs::msg::TrajectoryPoint calcInterpolatedTargetValue(
    const autoware_auto_msgs::msg::Trajectory & traj, const geometry_msgs::msg::Point & point,
    const size_t nearest_idx) const;

  /**
   * @brief calculate predicted velocity after time delay based on past control commands
   * @param [in] current_motion current velocity and acceleration of the vehicle
   * @param [in] delay_compensation_time predicted time delay
   */
  float64_t predictedVelocityInTargetPoint(
    const Motion current_motion, const float64_t delay_compensation_time) const;

  /**
   * @brief calculate velocity feedback with feed forward and pid controller
   * @param [in] target_motion reference velocity and acceleration. This acceleration will be used as feed forward.
   * @param [in] dt time step to use
   * @param [in] current_vel current velocity of the vehicle
   */
  float64_t applyVelocityFeedback(
    const Motion target_motion, const float64_t dt, const float64_t current_vel);

  /**
   * @brief update variables for debugging about pitch
   * @param [in] pitch current pitch of the vehicle (filtered)
   * @param [in] traj_pitch current trajectory pitch
   * @param [in] raw_pitch current raw pitch of the vehicle (unfiltered)
   */
  void updatePitchDebugValues(
    const float64_t pitch, const float64_t traj_pitch,
    const float64_t raw_pitch);

  /**
   * @brief update variables for velocity and acceleration
   * @param [in] ctrl_cmd latest calculated control command
   * @param [in] current_pose current pose of the vehicle
   * @param [in] control_data data for control calculation
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
