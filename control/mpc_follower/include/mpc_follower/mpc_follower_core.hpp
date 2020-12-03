/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

/**
 * @file moc_follower.h
 * @brief mpc follower class
 * @author Takamasa Horibe
 * @date 2019.05.01
 */

#ifndef CONTROL_MPC_FOLLOWER_INCLUDE_MPC_FOLLOWER_MPC_FOLLOWER_CORE_H
#define CONTROL_MPC_FOLLOWER_INCLUDE_MPC_FOLLOWER_MPC_FOLLOWER_CORE_H

#include "mpc_follower/interpolate.hpp"
#include "mpc_follower/lowpass_filter.hpp"
#include "mpc_follower/mpc_trajectory.hpp"
#include "mpc_follower/mpc_utils.hpp"
#include "mpc_follower/qp_solver/qp_solver_osqp.hpp"
#include "mpc_follower/qp_solver/qp_solver_unconstr_fast.hpp"
#include "mpc_follower/vehicle_model/vehicle_model_bicycle_dynamics.hpp"
#include "mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"

#include <osqp_interface/osqp_interface.hpp>
#include <autoware_control_msgs/msg/control_command_stamped.hpp>
#include <autoware_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <autoware_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/steering.hpp>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <unistd.h>
#include <deque>
#include <memory>
#include <string>
#include <vector>

/**
 * @class MPC-based waypoints follower class
 * @brief calculate control command to follow reference waypoints
 */
class MPCFollower : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   */
  MPCFollower();

  /**
   * @brief destructor
   */
  virtual ~MPCFollower();

private:
  rclcpp::Publisher<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr
    pub_ctrl_cmd_;  //!< @brief topic publisher for control command
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Steering>::SharedPtr
    pub_debug_steer_cmd_;  //!< @brief topic publisher for control command
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    sub_ref_path_;  //!< @brief topic subscription for reference waypoints
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Steering>::SharedPtr
    sub_steering_;  //!< @brief subscription for currrent steering
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    sub_current_vel_;  //!< @brief subscription for currrent velocity

  rclcpp::TimerBase::SharedPtr timer_;  //!< @brief timer to update after a given interval
  void initTimer(double period_s);  //!< initialize timer to work in real, simulation, and replay

  MPCTrajectory ref_traj_;                //!< @brief reference trajectory to be followed
  Butterworth2dFilter lpf_steering_cmd_;  //!< @brief lowpass filter for steering command
  Butterworth2dFilter
    lpf_lateral_error_;  //!< @brief lowpass filter for lateral error to calculate derivative
  Butterworth2dFilter
    lpf_yaw_error_;  //!< @brief lowpass filter for heading error to calculate derivatie
  std::string vehicle_model_type_;                            //!< @brief vehicle model type for MPC
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr_;  //!< @brief vehicle model for MPC
  std::shared_ptr<QPSolverInterface> qpsolver_ptr_;           //!< @brief qp solver for MPC
  std::deque<double>
    input_buffer_;  //!< @brief control input (mpc_output) buffer for delay time conpemsation

  /* parameters for control*/
  double ctrl_period_;  //!< @brief control frequency [s]
  double
    steering_lpf_cutoff_hz_;  //!< @brief cutoff frequency of lowpass filter for steering command [Hz]
  double
    admisible_position_error_;  //!< @brief stop MPC calculation when lateral error is large than this value [m]
  double
    admisible_yaw_error_;  //!< @brief stop MPC calculation when heading error is large than this value [rad]
  double steer_lim_;       //!< @brief steering command limit [rad]
  double steer_rate_lim_;  //!< @brief steering rate limit [rad/s]
  double
    wheelbase_;  //!< @brief vehicle wheelbase length [m] to convert steering angle to angular velocity

  /* parameters for path smoothing */
  bool enable_path_smoothing_;      //< @brief flag for path smoothing
  bool enable_yaw_recalculation_;   //< @brief flag for recalculation of yaw angle after resampling
  int path_filter_moving_ave_num_;  //< @brief param of moving average filter for path smoothing
  int
    curvature_smoothing_num_;  //< @brief point-to-point index distance used in curvature calculation
  double traj_resample_dist_;  //< @brief path resampling interval [m]

  struct MPCParam
  {
    int prediction_horizon;                   //< @brief prediction horizon step
    double prediction_dt;                     //< @brief prediction horizon sampleing time
    double weight_lat_error;                  //< @brief lateral error weight in matrix Q
    double weight_heading_error;              //< @brief heading error weight in matrix Q
    double weight_heading_error_squared_vel;  //< @brief heading error * velocity weight in matrix Q
    double weight_steering_input;             //< @brief steering error weight in matrix R
    double
      weight_steering_input_squared_vel;   //< @brief steering error * velocity weight in matrix R
    double weight_lat_jerk;                //< @brief lateral jerk weight in matrix R
    double weight_steer_rate;              //< @brief steering rate weight in matrix R
    double weight_steer_acc;               //< @brief steering angle acceleration weight in matrix R
    double weight_terminal_lat_error;      //< @brief terminal lateral error weight in matrix Q
    double weight_terminal_heading_error;  //< @brief terminal heading error weight in matrix Q
    double zero_ff_steer_deg;              //< @brief threshold that feed-forward angle becomes zero
    double input_delay;             //< @brief delay time for steering input to be compensated
    double acceleration_limit;      //< @brief for trajectory velocity calculation
    double velocity_time_constant;  //< @brief for trajectory velocity calculation
  } mpc_param_;                     // for mpc design parameter

  struct MPCMatrix
  {
    Eigen::MatrixXd Aex;
    Eigen::MatrixXd Bex;
    Eigen::MatrixXd Wex;
    Eigen::MatrixXd Cex;
    Eigen::MatrixXd Qex;
    Eigen::MatrixXd R1ex;
    Eigen::MatrixXd R2ex;
    Eigen::MatrixXd Urefex;
    Eigen::MatrixXd Yrefex;
  };

  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_ptr_;  //!< @brief current measured pose
  geometry_msgs::msg::TwistStamped::SharedPtr
    current_velocity_ptr_;  //!< @brief current measured velocity
  autoware_vehicle_msgs::msg::Steering::SharedPtr
    current_steer_ptr_;  //!< @brief current measured steering
  autoware_planning_msgs::msg::Trajectory::SharedPtr
    current_trajectory_ptr_;  //!< @brief referece trajectory

  double raw_steer_cmd_prev_;  //< @brief steering command calculated by mpc in previous period
  double
    raw_steer_cmd_pprev_;  //< @brief steering command calculated by mpc in two times previous period
  double
    steer_cmd_prev_;  //< @brief steering command calculated by mpc and some filters in previous period
  double lateral_error_prev_;  //< @brief previous lateral error for derivative
  double yaw_error_prev_;      //< @brief previous lateral error for derivative

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;  //!< @brief tf listener

  /**
   * @brief compute and publish control command for path follow with a constant control period
   */
  void timerCallback();

  /**
   * @brief set current_trajectory_ with received message
   */
  void callbackTrajectory(autoware_planning_msgs::msg::Trajectory::SharedPtr);

  /**
   * @brief update current_pose from tf
   */
  void updateCurrentPose();

  /**
   * @brief check if the received data is valid.
   */
  bool checkData();

  /**
   * @brief get varables for mpc calculation
   */
  bool getVar(
    const MPCTrajectory & traj, int * closest_idx, double * closest_time,
    geometry_msgs::msg::Pose * closest_pose, float * steer, double * lat_err, double * yaw_err);

  /**
   * @brief set current_steer with received message
   */
  void callbackSteering(autoware_vehicle_msgs::msg::Steering::SharedPtr msg);

  /**
   * @brief set current_velocity with received message
   */
  void callbackCurrentVelocity(geometry_msgs::msg::TwistStamped::SharedPtr msg);

  /**
   * @brief publish control command as autoware_msgs/ControlCommand type
   * @param [in] cmd published control command
   */
  void publishCtrlCmd(const autoware_control_msgs::msg::ControlCommand & cmd);

  /**
   * @brief calculate control command by MPC algorithm
   * @param [out] cmd calculated control command with mpc algorithm
   */
  bool calculateMPC(autoware_control_msgs::msg::ControlCommand * cmd);

  /**
   * @brief set initial condition for mpc
   * @param [in] lat_err lateral error
   * @param [in] yaw_err yaw error
   */
  Eigen::VectorXd getInitialState(
    const double & lat_err, const double & yaw_err, const double & steer);

  /**
   * @brief update status for delay compensation
   * @param [in] start_time start time for update
   * @param [out] x updated state at delayed_time
   */
  bool updateStateForDelayCompensation(
    const MPCTrajectory & traj, const double & start_time, Eigen::VectorXd * x);

  /**
   * @brief generate MPC matrix with trajectory and vehicle model
   * @param [in] reference_trajectory used for linearization around reference trajectory
   */
  MPCMatrix generateMPCMatrix(const MPCTrajectory & reference_trajectory);

  /**
   * @brief generate MPC matrix with trajectory and vehicle model
   * @param [out] Uex optimized input vector
   */
  bool executeOptimization(
    const MPCMatrix & mpc_matrix, const Eigen::VectorXd & x0, Eigen::VectorXd * Uex);

  /**
   * @brief get stop command
   */
  autoware_control_msgs::msg::ControlCommand getStopControlCommand() const;

  bool resampleMPCTrajectoryByTime(
    double start_time, const MPCTrajectory & input, MPCTrajectory * output) const;
  MPCTrajectory calcActualVelocity(const MPCTrajectory & trajectory);
  double getPredictionTime() const;
  void addSteerWeightR(Eigen::MatrixXd * R) const;
  void addSteerWeightF(Eigen::MatrixXd * f) const;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /**
   * @brief Declare MPC parameters as ROS parameters to allow tuning on the fly
   */
  void declareMPCparameters();

  /**
   * @brief Called when parameters are changed outside of node
   */
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /* ---------- debug ---------- */
  bool show_debug_info_;  //!< @brief flag to display debug info
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_marker_;
  rclcpp::Publisher<autoware_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr
    pub_debug_values_;  //!< @brief publisher for debug info
  rclcpp::Publisher<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr
    pub_debug_mpc_calc_time_;                        //!< @brief publisher for debug info
  geometry_msgs::msg::TwistStamped estimate_twist_;  //!< @brief received /estimate_twist for debug
};

#endif
