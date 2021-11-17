// Copyright 2021 Tier IV, Inc.
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

#ifndef MOTION_VELOCITY_SMOOTHER__MOTION_VELOCITY_SMOOTHER_NODE_HPP_
#define MOTION_VELOCITY_SMOOTHER__MOTION_VELOCITY_SMOOTHER_NODE_HPP_

#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/math/unit_conversion.hpp"
#include "autoware_utils/ros/self_pose_listener.hpp"
#include "autoware_utils/system/stop_watch.hpp"
#include "autoware_utils/trajectory/tmp_conversion.hpp"
#include "autoware_utils/trajectory/trajectory.hpp"
#include "motion_velocity_smoother/resample.hpp"
#include "motion_velocity_smoother/smoother/jerk_filtered_smoother.hpp"
#include "motion_velocity_smoother/smoother/l2_pseudo_jerk_smoother.hpp"
#include "motion_velocity_smoother/smoother/linf_pseudo_jerk_smoother.hpp"
#include "osqp_interface/osqp_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "autoware_debug_msgs/msg/float32_stamped.hpp"         // temporary
#include "autoware_planning_msgs/msg/stop_speed_exceeded.hpp"  // temporary
#include "autoware_planning_msgs/msg/velocity_limit.hpp"       // temporary
#include "nav_msgs/msg/odometry.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

// *INDENT-OFF*
#include "motion_velocity_smoother/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.hpp"
// *INDENT-ON*
#include "motion_velocity_smoother/smoother/smoother_base.hpp"
#include "motion_velocity_smoother/trajectory_utils.hpp"

namespace motion_velocity_smoother
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware_debug_msgs::msg::Float32Stamped;        // temporary
using autoware_planning_msgs::msg::StopSpeedExceeded;  // temporary
using autoware_planning_msgs::msg::VelocityLimit;      // temporary
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;

class MotionVelocitySmootherNode : public rclcpp::Node
{
public:
  explicit MotionVelocitySmootherNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Publisher<Trajectory>::SharedPtr
    pub_trajectory_;  //!< @brief publisher for output trajectory
  rclcpp::Publisher<StopSpeedExceeded>::SharedPtr
    pub_over_stop_velocity_;  //!< @brief publisher for over stop velocity warning
  rclcpp::Subscription<Odometry>::SharedPtr
    sub_current_odometry_;  //!< @brief subscriber for current velocity
  rclcpp::Subscription<Trajectory>::SharedPtr
    sub_current_trajectory_;  //!< @brief subscriber for reference trajectory
  rclcpp::Subscription<VelocityLimit>::SharedPtr
    sub_external_velocity_limit_;  //!< @brief subscriber for external velocity limit

  PoseStamped::ConstSharedPtr current_pose_ptr_;   // current vehicle pose
  Odometry::ConstSharedPtr current_odometry_ptr_;  // current odometry
  Trajectory::ConstSharedPtr base_traj_raw_ptr_;   // current base_waypoints
  double external_velocity_limit_;                 // current external_velocity_limit
  double max_velocity_with_deceleration_;          // maximum velocity with deceleration
                                                   // for external velocity limit
  double external_velocity_limit_dist_{0.0};       // distance to set external velocity limit

  TrajectoryPoints prev_output_;                           // previously published trajectory
  boost::optional<TrajectoryPoint> prev_closest_point_{};  // previous trajectory point
                                                           // closest to ego vehicle

  autoware_utils::SelfPoseListener self_pose_listener_{this};

  enum class AlgorithmType {
    INVALID = 0,
    JERK_FILTERED = 1,
    L2 = 2,
    LINF = 3,
    ANALYTICAL = 4,
  };

  enum class InitializeType {
    INIT = 0,
    LARGE_DEVIATION_REPLAN = 1,
    ENGAGING = 2,
    NORMAL = 3,
  };

  struct Param
  {
    double max_velocity;                              // max velocity [m/s]
    double margin_to_insert_external_velocity_limit;  // for external velocity limit [m]
    double replan_vel_deviation;                      // if speed error exceeds this [m/s],
                                                      // replan from current velocity
    double engage_velocity;                           // use this speed when start moving [m/s]
    double engage_acceleration;           // use this acceleration when start moving [m/ss]
    double engage_exit_ratio;             // exit engage sequence
                                          // when the speed exceeds ratio x engage_vel.
    double stopping_velocity;             // change target velocity to this value before v=0 point.
    double stopping_distance;             // distance for the stopping_velocity
    double extract_ahead_dist;            // forward waypoints distance from current position [m]
    double extract_behind_dist;           // backward waypoints distance from current position [m]
    double stop_dist_to_prohibit_engage;  // prevent to move toward close stop point
    double delta_yaw_threshold;           // for closest index calculation
    resampling::ResampleParam post_resample_param;
    AlgorithmType algorithm_type;  // Option : JerkFiltered, Linf, L2
  } node_param_{};

  SmootherBase::BaseParam base_param_{};
  JerkFilteredSmoother::Param jerk_filtered_smoother_param_{};
  L2PseudoJerkSmoother::Param l2_pseudo_jerk_smoother_param_{};
  LinfPseudoJerkSmoother::Param linf_pseudo_jerk_smoother_param_{};
  AnalyticalJerkConstrainedSmoother::Param analytical_jerk_constrained_smoother_param_{};

  std::shared_ptr<SmootherBase> smoother_;

  bool publish_debug_trajs_;  // publish planned trajectories

  double over_stop_velocity_warn_thr_;  // threshold to publish over velocity warn

  // parameter update
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);

  // topic callback
  void onCurrentOdometry(const Odometry::ConstSharedPtr msg);

  void onCurrentTrajectory(const Trajectory::ConstSharedPtr msg);

  void onExternalVelocityLimit(const VelocityLimit::ConstSharedPtr msg);

  // publish methods
  void publishTrajectory(const TrajectoryPoints & traj) const;

  void publishStopDistance(const TrajectoryPoints & trajectory, const size_t closest) const;

  // non-const methods
  void publishClosestState(const TrajectoryPoint & closest_point);

  // const methods
  bool checkData() const;

  AlgorithmType getAlgorithmType(const std::string & algorithm_name) const;

  TrajectoryPoints calcTrajectoryVelocity(const TrajectoryPoints & input) const;

  bool smoothVelocity(const TrajectoryPoints & input, TrajectoryPoints & traj_smoothed) const;

  std::tuple<double, double, InitializeType> calcInitialMotion(
    const TrajectoryPoints & input_traj, const size_t input_closest,
    const TrajectoryPoints & prev_traj) const;

  void applyExternalVelocityLimit(TrajectoryPoints & traj) const;

  void insertBehindVelocity(
    const size_t output_closest, const InitializeType type, TrajectoryPoints & output) const;

  void applyStopApproachingVelocity(TrajectoryPoints & traj) const;

  void overwriteStopPoint(const TrajectoryPoints & input, TrajectoryPoints & output) const;

  double calcTravelDistance() const;

  bool isEngageStatus(const double target_vel) const;

  void publishDebugTrajectories(const std::vector<TrajectoryPoints> & debug_trajectories) const;

  void publishClosestVelocity(
    const TrajectoryPoints & trajectory, const Pose & current_pose,
    const rclcpp::Publisher<Float32Stamped>::SharedPtr pub) const;

  Trajectory toTrajectoryMsg(
    const TrajectoryPoints & points, const std_msgs::msg::Header & header) const;

  // parameter handling
  void initCommonParam();

  void initSmootherBaseParam();

  void initJerkFilteredSmootherParam();

  void initL2PseudoJerkSmootherParam();

  void initLinfPseudoJerkSmootherParam();

  void initAnalyticalJerkConstrainedSmootherParam();

  // debug
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch_;
  std::shared_ptr<rclcpp::Time> prev_time_;
  double prev_acc_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_dist_to_stopline_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_raw_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr pub_velocity_limit_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_vel_lim_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_latacc_filtered_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_resampled_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_closest_velocity_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_closest_acc_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_closest_jerk_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_calculation_time_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_closest_max_velocity_;

  // For Jerk Filtered Algorithm Debug
  rclcpp::Publisher<Trajectory>::SharedPtr pub_forward_filtered_trajectory_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_backward_filtered_trajectory_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_merged_filtered_trajectory_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_closest_merged_velocity_;
};
}  // namespace motion_velocity_smoother

#endif  // MOTION_VELOCITY_SMOOTHER__MOTION_VELOCITY_SMOOTHER_NODE_HPP_
