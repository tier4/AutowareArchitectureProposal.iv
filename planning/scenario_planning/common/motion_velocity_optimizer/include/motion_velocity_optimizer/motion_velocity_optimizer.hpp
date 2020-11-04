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
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_debug_msgs/msg/float32_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <autoware_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>
#include <mutex>
#include <string>

#include <motion_velocity_optimizer/motion_velocity_optimizer_utils.hpp>
#include <motion_velocity_optimizer/optimizer/optimizer_base.hpp>

#include <osqp_interface/osqp_interface.h>

class MotionVelocityOptimizer : public rclcpp::Node
{
public:
  MotionVelocityOptimizer();
  ~MotionVelocityOptimizer();

private:
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_; //!< @brief publisher for output trajectory
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_current_velocity_;  //!< @brief subscriber for current velocity
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_current_trajectory_;  //!< @brief subscriber for reference trajectory
  rclcpp::Subscription<autoware_planning_msgs::msg::VelocityLimit>::SharedPtr sub_external_velocity_limit_;//!< @brief subscriber for external velocity limit
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;  //!< @brief tf butter
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  //!< @brief tf listener

  double external_velocity_limit_update_rate_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;


  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_ptr_;           // current vehicle pose
  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_velocity_ptr_;      // current vehicle twist
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr base_traj_raw_ptr_;  // current base_waypoints
  autoware_planning_msgs::msg::VelocityLimit::ConstSharedPtr external_velocity_limit_ptr_;  // current external_velocity_limit
  std::shared_ptr<double> external_velocity_limit_filtered_;

  autoware_planning_msgs::msg::Trajectory prev_output_;  // previously published trajectory

  enum class InitializeType {
    INIT = 0,
    LARGE_DEVIATION_REPLAN = 1,
    ENGAGING = 2,
    NORMAL = 3,
  };
  InitializeType initialize_type_;

  std::shared_ptr<OptimizerBase> optimizer_;

  bool publish_debug_trajs_;  // publish planned trajectories

  struct MotionVelocityOptimizerParam
  {
    double max_velocity;                 // max velocity [m/s]
    double max_accel;                    // max acceleration in planning [m/s2] > 0
    double min_decel;                    // min deceltion in planning [m/s2] < 0
    double max_lateral_accel;            // max lateral acceleartion [m/ss] > 0
    double min_curve_velocity;           // min velocity at curve [m/s]
    double decel_distance_before_curve;  // distance before slow down for lateral acc at a curve
    double decel_distance_after_curve;   // distance after slow down for lateral acc at a curve
    double replan_vel_deviation;  // if speed error exceeds this [m/s], replan from current velocity
    double engage_velocity;       // use this speed when start moving [m/s]
    double engage_acceleration;   // use this acceleration when start moving [m/ss]
    double engage_exit_ratio;     // exit engage sequence when the speed exceeds ratio x engege_vel.
    double extract_ahead_dist;    // forward waypoints distance from current position [m]
    double extract_behind_dist;   // backward waypoints distance from current position [m]
    double max_trajectory_length;             // max length of the objective trajectory for resample
    double min_trajectory_length;             // min length of the objective trajectory for resample
    double resample_time;                     // max time to calculate trajectory length
    double resample_dt;                       // dt to calculate trajectory length
    double min_trajectory_interval_distance;  // for resampling
    double stop_dist_to_prohibit_engage;      // prevent to move toward close stop point
    double delta_yaw_threshold;               // for closest index calculation
    std::string algorithm_type;               // Option : Linf, L2
  } planning_param_;

  /* topic callback */
  void callbackCurrentVelocity(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void callbackCurrentTrajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
  void callbackExternalVelocityLimit(const autoware_planning_msgs::msg::VelocityLimit::ConstSharedPtr msg);
  void timerCallback();


  /* non-const methods */
  void run();

  void blockUntilVehiclePositionAvailable(const tf2::Duration & duration);

  void updateCurrentPose();

  autoware_planning_msgs::msg::Trajectory calcTrajectoryVelocity(
    const autoware_planning_msgs::msg::Trajectory & base_traj);
  void updateExternalVelocityLimit(const double dt);

  autoware_planning_msgs::msg::Trajectory optimizeVelocity(
    const autoware_planning_msgs::msg::Trajectory & input, const int input_closest,
    const autoware_planning_msgs::msg::Trajectory & prev_output_traj, const int prev_output_closest);

  void calcInitialMotion(
    const double & base_speed, const autoware_planning_msgs::msg::Trajectory & base_waypoints,
    const int base_closest, const autoware_planning_msgs::msg::Trajectory & prev_replanned_traj,
    const int prev_replanned_traj_closest, double & initial_vel, double & initial_acc);

  /* const methods */
  bool resampleTrajectory(
    const autoware_planning_msgs::msg::Trajectory & input,
    autoware_planning_msgs::msg::Trajectory & output) const;

  bool lateralAccelerationFilter(
    const autoware_planning_msgs::msg::Trajectory & input,
    autoware_planning_msgs::msg::Trajectory & output) const;

  bool extractPathAroundIndex(
    const autoware_planning_msgs::msg::Trajectory & input, const int index,
    autoware_planning_msgs::msg::Trajectory & output) const;

  bool externalVelocityLimitFilter(
    const autoware_planning_msgs::msg::Trajectory & input,
    autoware_planning_msgs::msg::Trajectory & output) const;

  void preventMoveToCloseStopLine(
    const int closest, autoware_planning_msgs::msg::Trajectory & trajectory) const;

  void publishTrajectory(const autoware_planning_msgs::msg::Trajectory & traj) const;

  void publishStopDistance(
    const autoware_planning_msgs::msg::Trajectory & trajectory, const int closest) const;

  void insertBehindVelocity(
    const int prev_out_closest, const autoware_planning_msgs::msg::Trajectory & prev_output,
    const int output_closest, autoware_planning_msgs::msg::Trajectory & output) const;

  /* parameter update */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);


  /* debug */

  rclcpp::Publisher<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr pub_dist_to_stopline_;  //!< @brief publisher for stop distance
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_raw_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_vel_lim_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_latcc_filtered_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_resampled_;
  rclcpp::Publisher<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr debug_closest_velocity_;
  rclcpp::Publisher<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr debug_closest_acc_;
  void publishFloat(const double & data, const rclcpp::Publisher<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr pub) const;
};
