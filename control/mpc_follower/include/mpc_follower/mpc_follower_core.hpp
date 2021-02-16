// Copyright 2018-2019 Autoware Foundation
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

#include "autoware_control_msgs/msg/control_command_stamped.hpp"
#include "autoware_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "autoware_debug_msgs/msg/float32_stamped.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_vehicle_msgs/msg/steering.hpp"
#include "osqp_interface/osqp_interface.hpp"
#include "vehicle_info_util/vehicle_info.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <deque>
#include <memory>
#include <string>
#include <vector>
#include "unistd.h"

/**
 * @class MPC-based waypoints follower class
 * @brief calculate control command to follow reference waypoints
 */

struct MPCData
{
  int nearest_idx;
  double nearest_time;
  geometry_msgs::msg::Pose nearest_pose;
  double steer;
  double predicted_steer;
  double lateral_err;
  double yaw_err;
};
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
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    pub_predicted_traj_;  //!< @brief topic publisher for predicted trajectory
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    sub_ref_path_;  //!< @brief topic subscription for reference waypoints
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Steering>::SharedPtr
    sub_steering_;  //!< @brief subscription for currrent steering
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    sub_current_vel_;  //!< @brief subscription for currrent velocity

  rclcpp::TimerBase::SharedPtr timer_;  //!< @brief timer to update after a given interval
  void initTimer(double period_s);  //!< initialize timer to work in real, simulation, and replay

  MPCTrajectory ref_traj_;                 //!< @brief reference trajectory to be followed
  Butterworth2dFilter lpf_steering_cmd_;   //!< @brief lowpass filter for steering command
  Butterworth2dFilter lpf_lateral_error_;  //!< @brief lowpass filter for lateral error
  Butterworth2dFilter lpf_yaw_error_;      //!< @brief lowpass filter for heading error
  std::string vehicle_model_type_;         //!< @brief vehicle model type for MPC
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr_;  //!< @brief vehicle model for MPC
  std::shared_ptr<QPSolverInterface> qpsolver_ptr_;           //!< @brief qp solver for MPC
  std::deque<double> input_buffer_;  //!< @brief mpc_output buffer for delay time compensation

  /* parameters for control*/
  double ctrl_period_;               //!< @brief control frequency [s]
  double steering_lpf_cutoff_hz_;    //!< @brief cutoff frequency for steering command [Hz]
  double admisible_position_error_;  //!< @brief use stop cmd when lateral error exceeds this [m]
  double admisible_yaw_error_;       //!< @briefuse stop cmd when yaw error exceeds this [rad]
  double steer_lim_;                 //!< @brief steering command limit [rad]
  double steer_rate_lim_;            //!< @brief steering rate limit [rad/s]
  double wheelbase_;                 //!< @brief vehicle wheelbase length [m]

  /* parameters for path smoothing */
  bool enable_path_smoothing_;      //!< @brief flag for path smoothing
  bool enable_yaw_recalculation_;   //!< @brief flag for recalculation of yaw angle after resampling
  bool use_steer_prediction_;       //!< @brief flag to use predicted steer, not measured steer.
  int path_filter_moving_ave_num_;  //!< @brief param of moving average filter for path smoothing
  int curvature_smoothing_num_;  //!< @brief point-to-point index distance for curvature calculation
  double traj_resample_dist_;    //!< @brief path resampling interval [m]

  /* parameters for stop state */
  double stop_state_entry_ego_speed_;
  double stop_state_entry_target_speed_;
  double stop_state_keep_stopping_dist_;

  struct MPCParam
  {
    int prediction_horizon;  //!< @brief prediction horizon step
    double prediction_dt;    //!< @brief prediction horizon sampling time

    double zero_ff_steer_deg;       //!< @brief threshold that feed-forward angle becomes zero
    double input_delay;             //!< @brief delay time for steering input to be compensated
    double acceleration_limit;      //!< @brief for trajectory velocity calculation
    double velocity_time_constant;  //!< @brief for trajectory velocity calculation
    double steer_tau;               //!< @brief time constant for steer model

    // for weight matrix Q
    double weight_lat_error;                  //!< @brief lateral error weight
    double weight_heading_error;              //!< @brief heading error weight
    double weight_heading_error_squared_vel;  //!< @brief heading error * velocity weight
    double weight_terminal_lat_error;         //!< @brief terminal lateral error weight
    double weight_terminal_heading_error;     //!< @brief terminal heading error weight
    double low_curvature_weight_lat_error;  //!< @brief lateral error weight in matrix Q in low curvature point
    double low_curvature_weight_heading_error;  //!< @brief heading error weight in matrix Q in low curvature point
    double low_curvature_weight_heading_error_squared_vel;  //!< @brief heading error * velocity weight in matrix Q in low curvature point

    // for weight matrix R
    double weight_steering_input;              //!< @brief steering error weight
    double weight_steering_input_squared_vel;  //!< @brief steering error * velocity weight
    double weight_lat_jerk;                    //!< @brief lateral jerk weight
    double weight_steer_rate;                  //!< @brief steering rate weight
    double weight_steer_acc;                   //!< @brief steering angle acceleration weight
    double low_curvature_weight_steering_input;  //!< @brief steering error weight in matrix R in low curvature point
    double low_curvature_weight_steering_input_squared_vel;  //!< @brief steering error * velocity weight in matrix R in low curvature point
    double low_curvature_weight_lat_jerk;  //!< @brief lateral jerk weight in matrix R in low curvature point
    double low_curvature_weight_steer_rate;  //!< @brief steering rate weight in matrix R in low curvature point
    double low_curvature_weight_steer_acc;  //!< @brief steering angle acceleration weight in matrix R in low curvature

    double low_curvature_thresh_curvature;  //!< @brief threshold of curvature to use "low curvature" parameter
  };
  MPCParam mpc_param_;  // for mpc design parameter

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

  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_ptr_;        //!< @brief measured pose
  geometry_msgs::msg::TwistStamped::SharedPtr current_velocity_ptr_;   //!< @brief measured velocity
  autoware_vehicle_msgs::msg::Steering::SharedPtr current_steer_ptr_;  //!< @brief measured steering
  autoware_planning_msgs::msg::Trajectory::SharedPtr
    current_trajectory_ptr_;  //!< @brief reference trajectory

  double raw_steer_cmd_prev_ = 0.0;   //!< @brief mpc raw output in previous period
  double raw_steer_cmd_pprev_ = 0.0;  //!< @brief mpc raw output in two times previous period
  double steer_cmd_prev_ = 0.0;       //!< @brief mpc filtered output in previous period
  double lateral_error_prev_ = 0.0;   //!< @brief previous lateral error for derivative
  double yaw_error_prev_ = 0.0;       //!< @brief previous lateral error for derivative

  std::shared_ptr<double> steer_prediction_prev_;
  rclcpp::Time time_prev_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  double sign_vx_ = 0.0;  //!< @brief sign of previous target speed to calculate curvature when the target speed is 0.
  std::vector<autoware_control_msgs::msg::ControlCommandStamped>
    ctrl_cmd_vec_;  //!< buffer of send command

  bool is_ctrl_cmd_prev_initialized_ = false;  //!< @brief flag of ctrl_cmd_prev_ initialization
  autoware_control_msgs::msg::ControlCommand ctrl_cmd_prev_;  //!< @brief previous control command

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;  //!< @brief tf listener

  /**
   * @brief compute and publish control command for path follow with a constant control period
   */
  void onTimer();

  /**
   * @brief set current_trajectory_ with received message
   */
  void onTrajectory(const autoware_planning_msgs::msg::Trajectory::SharedPtr);

  /**
   * @brief update current_pose from tf
   */
  void updateCurrentPose();

  /**
   * @brief check if the received data is valid.
   */
  bool checkData();

  /**
   * @brief get variables for mpc calculation
   */
  bool getData(const MPCTrajectory & traj, MPCData * data);

  double calcSteerPrediction();
  double getSteerCmdSum(const rclcpp::Time &t_start, const rclcpp::Time &t_end, const double time_constant);
  void storeSteerCmd(const double steer);

  /**
   * @brief set current_steer with received message
   */
  void onSteering(const autoware_vehicle_msgs::msg::Steering::SharedPtr msg);

  /**
   * @brief set current_velocity with received message
   */
  void onVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

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
   * @param [in] mpc data
   */
  Eigen::VectorXd getInitialState(const MPCData & data);

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

  /**
   * @brief get initial command
   */
  autoware_control_msgs::msg::ControlCommand getInitialControlCommand() const;

  /**
   * @brief check ego car is in stopped state
   */
  bool isStoppedState() const;

  /**
   * @brief calculate distance to stop point
   */
  double calcStopDistance(const int origin) const;

  /**
   * @brief resample trajectory with mpc resampling time
   */
  bool resampleMPCTrajectoryByTime(
    double start_time, const MPCTrajectory & input, MPCTrajectory * output) const;

  /**
   * @brief apply velocity dynamics filter with v0 from closest index
   */
  MPCTrajectory applyVelocityDynamicsFilter(const MPCTrajectory & trajectory, const double v0);

  /**
   * @brief get total prediction time of mpc
   */
  double getPredictionTime() const;

  /**
   * @brief add weights related to lateral_jerk, steering_rate, steering_acc into R
   */
  void addSteerWeightR(Eigen::MatrixXd * R) const;

  /**
   * @brief add weights related to lateral_jerk, steering_rate, steering_acc into f
   */
  void addSteerWeightF(Eigen::MatrixXd * f) const;

  /**
   * @brief check if the matrix has invalid value
   */
  bool isValid(const MPCMatrix & m) const;

  /**
   * @brief check if the trajectory has valid value
   */
  bool isValidTrajectory(const autoware_planning_msgs::msg::Trajectory & traj) const;

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


  bool isLowCurvature(const double curvature)
  {
    return std::fabs(curvature) < mpc_param_.low_curvature_thresh_curvature;
  }

  double getWeightLatError(const double curvature)
  {
    return isLowCurvature(curvature) ? mpc_param_.low_curvature_weight_lat_error
                                     : mpc_param_.weight_lat_error;
  }

  double getWeightHeadingError(const double curvature)
  {
    return isLowCurvature(curvature) ? mpc_param_.low_curvature_weight_heading_error
                                     : mpc_param_.weight_heading_error;
  }

  double getWeightHeadingErrorSqVel(const double curvature)
  {
    return isLowCurvature(curvature) ? mpc_param_.low_curvature_weight_heading_error_squared_vel
                                     : mpc_param_.weight_heading_error_squared_vel;
  }

  double getWeightSteerInput(const double curvature)
  {
    return isLowCurvature(curvature) ? mpc_param_.low_curvature_weight_steering_input
                                     : mpc_param_.weight_steering_input;
  }

  double getWeightSteerInputSqVel(const double curvature)
  {
    return isLowCurvature(curvature) ? mpc_param_.low_curvature_weight_steering_input_squared_vel
                                     : mpc_param_.low_curvature_weight_steering_input_squared_vel;
  }

  double getWeightLatJerk(const double curvature)
  {
    return isLowCurvature(curvature) ? mpc_param_.low_curvature_weight_lat_jerk
                                     : mpc_param_.weight_lat_jerk;
  }

  double getWeightSteerRate(const double curvature)
  {
    return isLowCurvature(curvature) ? mpc_param_.low_curvature_weight_steer_rate
                                     : mpc_param_.weight_steer_rate;
  }

  double getWeightSteerAcc(const double curvature)
  {
    return isLowCurvature(curvature) ? mpc_param_.low_curvature_weight_steer_acc
                                     : mpc_param_.weight_steer_acc;
  }

  /* ---------- debug ---------- */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_marker_;
  rclcpp::Publisher<autoware_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr
    pub_debug_values_;
  rclcpp::Publisher<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr pub_debug_mpc_calc_time_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Steering>::SharedPtr pub_debug_steer_cmd_;
};

#endif
