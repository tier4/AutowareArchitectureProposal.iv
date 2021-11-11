// Copyright 2021 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SIMPLE_PLANNING_SIMULATOR__SIMPLE_PLANNING_SIMULATOR_CORE_HPP_
#define SIMPLE_PLANNING_SIMULATOR__SIMPLE_PLANNING_SIMULATOR_CORE_HPP_


#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"

#include "simple_planning_simulator/visibility_control.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_control_command.hpp"
#include "autoware_auto_geometry_msgs/msg/complex32.hpp"
#include "common/types.hpp"

#include "simple_planning_simulator/vehicle_model/sim_model_interface.hpp"


namespace simulation
{
namespace simple_planning_simulator
{
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_vehicle_msgs::msg::ControlModeReport;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::msg::GearReport;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::VehicleControlCommand;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Odometry;
using autoware_auto_geometry_msgs::msg::Complex32;

class DeltaTime
{
public:
  DeltaTime()
  : prev_updated_time_ptr_(nullptr) {}
  float64_t get_dt(const rclcpp::Time & now)
  {
    if (prev_updated_time_ptr_ == nullptr) {
      prev_updated_time_ptr_ = std::make_shared<rclcpp::Time>(now);
      return 0.0;
    }
    const float64_t dt = (now - *prev_updated_time_ptr_).seconds();
    *prev_updated_time_ptr_ = now;
    return dt;
  }

private:
  std::shared_ptr<rclcpp::Time> prev_updated_time_ptr_;
};

class MeasurementNoiseGenerator
{
public:
  MeasurementNoiseGenerator() {}

  std::shared_ptr<std::mt19937> rand_engine_;
  std::shared_ptr<std::normal_distribution<>> pos_dist_;
  std::shared_ptr<std::normal_distribution<>> vel_dist_;
  std::shared_ptr<std::normal_distribution<>> rpy_dist_;
  std::shared_ptr<std::normal_distribution<>> steer_dist_;
};

class PLANNING_SIMULATOR_PUBLIC SimplePlanningSimulator : public rclcpp::Node
{
public:
  explicit SimplePlanningSimulator(const rclcpp::NodeOptions & options);

private:
  /* ros system */
  rclcpp::Publisher<TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steer_;
  rclcpp::Publisher<ControlModeReport>::SharedPtr pub_control_mode_report_;
  rclcpp::Publisher<GearReport>::SharedPtr pub_gear_report_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_current_pose_;

  rclcpp::Subscription<GearCommand>::SharedPtr sub_gear_cmd_;
  rclcpp::Subscription<VehicleControlCommand>::SharedPtr sub_vehicle_cmd_;
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_ackermann_cmd_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_init_pose_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;

  uint32_t timer_sampling_time_ms_;  //!< @brief timer sampling time
  rclcpp::TimerBase::SharedPtr on_timer_;  //!< @brief timer for simulation

  /* tf */
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  /* received & published topics */
  TwistStamped current_twist_;
  Odometry current_odometry_;
  SteeringReport current_steer_;
  VehicleControlCommand::ConstSharedPtr current_vehicle_cmd_ptr_;
  AckermannControlCommand::ConstSharedPtr current_ackermann_cmd_ptr_;
  GearCommand::ConstSharedPtr current_gear_cmd_ptr_;
  Trajectory::ConstSharedPtr current_trajectory_ptr_;

  /* frame_id */
  std::string simulated_frame_id_;  //!< @brief simulated vehicle frame id
  std::string origin_frame_id_;  //!< @brief map frame_id

  /* flags */
  bool8_t is_initialized_;         //!< @brief flag to check the initial position is set
  bool8_t add_measurement_noise_;  //!< @brief flag to add measurement noise

  DeltaTime delta_time_;  //!< @brief to calculate delta time

  MeasurementNoiseGenerator measurement_noise_;  //!< @brief for measurement noise

  /* vehicle model */
  enum class VehicleModelType
  {
    IDEAL_STEER_ACC = 0,
    IDEAL_STEER_ACC_GEARED = 1,
    DELAY_STEER_ACC = 2,
    DELAY_STEER_ACC_GEARED = 3,
    IDEAL_STEER_VEL = 4,
  } vehicle_model_type_;  //!< @brief vehicle model type to decide the model dynamics
  std::shared_ptr<SimModelInterface> vehicle_model_ptr_;  //!< @brief vehicle model pointer

  /**
   * @brief set current_vehicle_cmd_ptr_ with received message
   */
  void on_vehicle_cmd(const VehicleControlCommand::ConstSharedPtr msg);

  /**
   * @brief set current_ackermann_cmd_ptr_ with received message
   */
  void on_ackermann_cmd(const AckermannControlCommand::ConstSharedPtr msg);

  /**
   * @brief set input steering, velocity, and acceleration of the vehicle model
   */
  void set_input(const float steer, const float vel, const float accel);

  /**
   * @brief set current_vehicle_state_ with received message
   */
  void on_gear_cmd(const GearCommand::ConstSharedPtr msg);

  /**
   * @brief set initial pose for simulation with received message
   */
  void on_initialpose(const PoseWithCovarianceStamped::ConstSharedPtr msg);

  /**
   * @brief subscribe trajector for deciding self z position.
   */
  void on_trajectory(const Trajectory::ConstSharedPtr msg);

  /**
   * @brief get z-position from trajectory
   * @param [in] x current x-position
   * @param [in] y current y-position
   * @return get z-position from trajectory
   */
  double get_z_pose_from_trajectory(const double x, const double y);

  /**
   * @brief get transform from two frame_ids
   * @param [in] parent_frame parent frame id
   * @param [in] child_frame child frame id
   * @return transform from parent frame to child frame
   */
  TransformStamped get_transform_msg(const std::string parent_frame, const std::string child_frame);

  /**
   * @brief timer callback for simulation with loop_rate
   */
  void on_timer();

  /**
   * @brief initialize vehicle_model_ptr
   */
  void initialize_vehicle_model();

  /**
   * @brief add measurement noise
   * @param [in] odometry odometry to add noise
   * @param [in] twist twist to add noise
   * @param [in] steer steering to add noise
   */
  void add_measurement_noise(Odometry & odom, TwistStamped & twist, SteeringReport & steer) const;

  /**
   * @brief set initial state of simulated vehicle
   * @param [in] pose initial position and orientation
   * @param [in] twist initial velocity and angular velocity
   */
  void set_initial_state(const Pose & pose, const Twist & twist);

  /**
   * @brief set initial state of simulated vehicle with pose transformation based on frame_id
   * @param [in] pose initial position and orientation with header
   * @param [in] twist initial velocity and angular velocity
   */
  void set_initial_state_with_transform(const PoseStamped & pose, const Twist & twist);

  /**
   * @brief publish twist
   * @param [in] odometry The twist to publish
   */
  void publish_twist(const TwistStamped & twist);

  /**
   * @brief publish pose and twist
   * @param [in] odometry The odometry to publish
   */
  void publish_odometry(const Odometry & odometry);

  /**
   * @brief publish steering
   * @param [in] steer The steering to publish
   */
  void publish_steering(const SteeringReport & steer);

  /**
   * @brief publish control_mode report
   */
  void publish_control_mode_report();

  /**
   * @brief publish gear report
   */
  void publish_gear_report();

  /**
   * @brief publish tf
   * @param [in] state The kinematic state to publish as a TF
   */
  void publish_tf(const Odometry & odometry);
};
}  // namespace simple_planning_simulator
}  // namespace simulation

#endif  // SIMPLE_PLANNING_SIMULATOR__SIMPLE_PLANNING_SIMULATOR_CORE_HPP_
