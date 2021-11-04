// Copyright 2020 Tier IV, Inc.
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
//
//
// Author: v1.0 Yukihiro Saito
//

#include "multi_object_tracker/tracker/model/bicycle_tracker.hpp"

#include "multi_object_tracker/utils/utils.hpp"

#include <autoware_utils/autoware_utils.hpp>

#include <bits/stdc++.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

BicycleTracker::BicycleTracker(
  const rclcpp::Time & time, const autoware_perception_msgs::msg::DynamicObject & object)
: Tracker(time, object.semantic.type),
  logger_(rclcpp::get_logger("BicycleTracker")),
  last_update_time_(time),
  z_(object.state.pose_covariance.pose.position.z)
{
  object_ = object;

  // initialize params
  ekf_params_.use_measurement_covariance = false;
  float q_stddev_x = 0.0;                               // [m/s]
  float q_stddev_y = 0.0;                               // [m/s]
  float q_stddev_yaw = autoware_utils::deg2rad(10);     // [rad/s]
  float q_stddev_vx = autoware_utils::kmph2mps(10);     // [m/(s*s)]
  float q_stddev_wz = autoware_utils::deg2rad(25);      // [rad/(s*s)]
  float r_stddev_x = 0.6;                               // [m]
  float r_stddev_y = 0.4;                               // [m]
  float r_stddev_yaw = autoware_utils::deg2rad(30);     // [rad]
  float p0_stddev_x = 0.8;                              // [m/s]
  float p0_stddev_y = 0.5;                              // [m/s]
  float p0_stddev_yaw = autoware_utils::deg2rad(1000);  // [rad/s]
  float p0_stddev_vx = autoware_utils::kmph2mps(1000);  // [m/(s*s)]
  float p0_stddev_wz = autoware_utils::deg2rad(10);     // [rad/(s*s)]
  ekf_params_.q_cov_x = std::pow(q_stddev_x, 2.0);
  ekf_params_.q_cov_y = std::pow(q_stddev_y, 2.0);
  ekf_params_.q_cov_yaw = std::pow(q_stddev_yaw, 2.0);
  ekf_params_.q_cov_vx = std::pow(q_stddev_vx, 2.0);
  ekf_params_.q_cov_wz = std::pow(q_stddev_wz, 2.0);
  ekf_params_.r_cov_x = std::pow(r_stddev_x, 2.0);
  ekf_params_.r_cov_y = std::pow(r_stddev_y, 2.0);
  ekf_params_.r_cov_yaw = std::pow(r_stddev_yaw, 2.0);
  ekf_params_.p0_cov_x = std::pow(p0_stddev_x, 2.0);
  ekf_params_.p0_cov_y = std::pow(p0_stddev_y, 2.0);
  ekf_params_.p0_cov_yaw = std::pow(p0_stddev_yaw, 2.0);
  ekf_params_.p0_cov_vx = std::pow(p0_stddev_vx, 2.0);
  ekf_params_.p0_cov_wz = std::pow(p0_stddev_wz, 2.0);
  max_vx_ = autoware_utils::kmph2mps(100);  // [m/s]
  max_wz_ = autoware_utils::deg2rad(30);    // [rad/s]

  // initialize X matrix
  Eigen::MatrixXd X(ekf_params_.dim_x, 1);
  X(IDX::X) = object.state.pose_covariance.pose.position.x;
  X(IDX::Y) = object.state.pose_covariance.pose.position.y;
  X(IDX::YAW) = tf2::getYaw(object.state.pose_covariance.pose.orientation);
  if (object.state.twist_reliable) {
    X(IDX::VX) = object.state.twist_covariance.twist.linear.x;
    X(IDX::WZ) = object.state.twist_covariance.twist.angular.z;
  } else {
    X(IDX::VX) = 0.0;
    X(IDX::WZ) = 0.0;
  }

  // initialize P matrix
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);
  if (
    !ekf_params_.use_measurement_covariance ||
    object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_X] == 0.0 ||
    object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_Y] == 0.0 ||
    object.state.pose_covariance.covariance[utils::MSG_COV_IDX::YAW_YAW] == 0.0) {
    const double cos_yaw = std::cos(X(IDX::YAW));
    const double sin_yaw = std::sin(X(IDX::YAW));
    const double sin_2yaw = std::sin(2.0f * X(IDX::YAW));
    // Rotate the covariance matrix according to the vehicle yaw
    // because p0_cov_x and y are in the vehicle coordinate system.
    P(IDX::X, IDX::X) =
      ekf_params_.p0_cov_x * cos_yaw * cos_yaw + ekf_params_.p0_cov_y * sin_yaw * sin_yaw;
    P(IDX::X, IDX::Y) = 0.5f * (ekf_params_.p0_cov_x - ekf_params_.p0_cov_y) * sin_2yaw;
    P(IDX::Y, IDX::Y) =
      ekf_params_.p0_cov_x * sin_yaw * sin_yaw + ekf_params_.p0_cov_y * cos_yaw * cos_yaw;
    P(IDX::Y, IDX::X) = P(IDX::X, IDX::Y);
    P(IDX::YAW, IDX::YAW) = ekf_params_.p0_cov_yaw;
    P(IDX::VX, IDX::VX) = ekf_params_.p0_cov_vx;
    P(IDX::WZ, IDX::WZ) = ekf_params_.p0_cov_wz;
  } else {
    P(IDX::X, IDX::X) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_X];
    P(IDX::X, IDX::Y) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_Y];
    P(IDX::Y, IDX::Y) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_Y];
    P(IDX::Y, IDX::X) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_X];
    P(IDX::YAW, IDX::YAW) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::YAW_YAW];
    if (object.state.twist_reliable) {
      P(IDX::VX, IDX::VX) = object.state.twist_covariance.covariance[utils::MSG_COV_IDX::X_X];
      P(IDX::WZ, IDX::WZ) = object.state.twist_covariance.covariance[utils::MSG_COV_IDX::YAW_YAW];
    } else {
      P(IDX::VX, IDX::VX) = ekf_params_.p0_cov_vx;
      P(IDX::WZ, IDX::WZ) = ekf_params_.p0_cov_wz;
    }
  }

  if (object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    bounding_box_ = {
      object.shape.dimensions.x, object.shape.dimensions.y, object.shape.dimensions.z};
  } else {
    bounding_box_ = {1.0, 0.5, 1.7};
  }
  ekf_.init(X, P);
}

bool BicycleTracker::predict(const rclcpp::Time & time)
{
  const double dt = (time - last_update_time_).seconds();
  bool ret = predict(dt, ekf_);
  if (ret) {
    last_update_time_ = time;
  }
  return ret;
}

bool BicycleTracker::predict(const double dt, KalmanFilter & ekf) const
{
  /*  == Nonlinear model ==
   *
   * x_{k+1}   = x_k + vx_k * cos(yaw_k) * dt
   * y_{k+1}   = y_k + vx_k * sin(yaw_k) * dt
   * yaw_{k+1} = yaw_k + (wz_k) * dt
   * vx_{k+1}  = vx_k
   * wz_{k+1}  = wz_k
   *
   */

  /*  == Linearized model ==
   *
   * A = [ 1, 0, -vx*sin(yaw)*dt, cos(yaw)*dt,  0]
   *     [ 0, 1,  vx*cos(yaw)*dt, sin(yaw)*dt,  0]
   *     [ 0, 0,               1,           0, dt]
   *     [ 0, 0,               0,           1,  0]
   *     [ 0, 0,               0,           0,  1]
   */

  // X t
  Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);  // predicted state
  ekf.getX(X_t);
  const double cos_yaw = std::cos(X_t(IDX::YAW));
  const double sin_yaw = std::sin(X_t(IDX::YAW));
  const double sin_2yaw = std::sin(2.0f * X_t(IDX::YAW));

  // X t+1
  Eigen::MatrixXd X_next_t(ekf_params_.dim_x, 1);                // predicted state
  X_next_t(IDX::X) = X_t(IDX::X) + X_t(IDX::VX) * cos_yaw * dt;  // dx = v * cos(yaw)
  X_next_t(IDX::Y) = X_t(IDX::Y) + X_t(IDX::VX) * sin_yaw * dt;  // dy = v * sin(yaw)
  X_next_t(IDX::YAW) = X_t(IDX::YAW) + (X_t(IDX::WZ)) * dt;      // dyaw = omega
  X_next_t(IDX::VX) = X_t(IDX::VX);
  X_next_t(IDX::WZ) = X_t(IDX::WZ);

  // A
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(ekf_params_.dim_x, ekf_params_.dim_x);
  A(IDX::X, IDX::YAW) = -X_t(IDX::VX) * sin_yaw * dt;
  A(IDX::X, IDX::VX) = cos_yaw * dt;
  A(IDX::Y, IDX::YAW) = X_t(IDX::VX) * cos_yaw * dt;
  A(IDX::Y, IDX::VX) = sin_yaw * dt;
  A(IDX::YAW, IDX::WZ) = dt;

  // Q
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);
  // Rotate the covariance matrix according to the vehicle yaw
  // because q_cov_x and y are in the vehicle coordinate system.
  Q(IDX::X, IDX::X) =
    (ekf_params_.q_cov_x * cos_yaw * cos_yaw + ekf_params_.q_cov_y * sin_yaw * sin_yaw) * dt * dt;
  Q(IDX::X, IDX::Y) = (0.5f * (ekf_params_.q_cov_x - ekf_params_.q_cov_y) * sin_2yaw) * dt * dt;
  Q(IDX::Y, IDX::Y) =
    (ekf_params_.q_cov_x * sin_yaw * sin_yaw + ekf_params_.q_cov_y * cos_yaw * cos_yaw) * dt * dt;
  Q(IDX::Y, IDX::X) = Q(IDX::X, IDX::Y);
  Q(IDX::YAW, IDX::YAW) = ekf_params_.q_cov_yaw * dt * dt;
  Q(IDX::VX, IDX::VX) = ekf_params_.q_cov_vx * dt * dt;
  Q(IDX::WZ, IDX::WZ) = ekf_params_.q_cov_wz * dt * dt;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);
  Eigen::MatrixXd u = Eigen::MatrixXd::Zero(ekf_params_.dim_x, 1);

  if (!ekf.predict(X_next_t, A, Q)) {
    RCLCPP_WARN(logger_, "Cannot predict");
  }

  return true;
}

bool BicycleTracker::measureWithPose(const autoware_perception_msgs::msg::DynamicObject & object)
{
  constexpr int dim_y = 2;  // pos x, pos y, depending on Pose output
  // double measurement_yaw =
  //   autoware_utils::normalizeRadian(tf2::getYaw(object.state.pose_covariance.pose.orientation));
  // {
  //   Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);
  //   ekf_.getX(X_t);
  //   while (M_PI_2 <= X_t(IDX::YAW) - measurement_yaw) {
  //     measurement_yaw = measurement_yaw + M_PI;
  //   }
  //   while (M_PI_2 <= measurement_yaw - X_t(IDX::YAW)) {
  //     measurement_yaw = measurement_yaw - M_PI;
  //   }
  //   float theta = std::acos(
  //     std::cos(X_t(IDX::YAW)) * std::cos(measurement_yaw) +
  //     std::sin(X_t(IDX::YAW)) * std::sin(measurement_yaw));
  //   if (autoware_utils::deg2rad(60) < std::fabs(theta)) return false;
  // }

  /* Set measurement matrix */
  Eigen::MatrixXd Y(dim_y, 1);
  Y << object.state.pose_covariance.pose.position.x, object.state.pose_covariance.pose.position.y;

  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, ekf_params_.dim_x);
  C(0, IDX::X) = 1.0;  // for pos x
  C(1, IDX::Y) = 1.0;  // for pos y
  // C(2, IDX::YAW) = 1.0;  // for yaw

  /* Set measurement noise covariance */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  if (
    !ekf_params_.use_measurement_covariance ||
    object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_X] == 0.0 ||
    object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_Y] == 0.0) {
    R(0, 0) = ekf_params_.r_cov_x;  // x - x
    R(0, 1) = 0.0;                  // x - y
    R(1, 1) = ekf_params_.r_cov_y;  // y - y
    R(1, 0) = R(0, 1);              // y - x
    // R(2, 2) = ekf_params_.r_cov_yaw;                        // yaw - yaw
  } else {
    R(0, 0) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_X];
    R(0, 1) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_Y];
    // R(0, 2) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_YAW];
    R(1, 0) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_X];
    R(1, 1) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_Y];
    // R(1, 2) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_YAW];
    // R(2, 0) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::YAW_X];
    // R(2, 1) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::YAW_Y];
    // R(2, 2) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::YAW_YAW];
  }
  if (!ekf_.update(Y, C, R)) {
    RCLCPP_WARN(logger_, "Cannot update");
  }

  // normalize yaw and limit vx, wz
  {
    Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);
    Eigen::MatrixXd P_t(ekf_params_.dim_x, ekf_params_.dim_x);
    ekf_.getX(X_t);
    ekf_.getP(P_t);
    X_t(IDX::YAW) = autoware_utils::normalizeRadian(X_t(IDX::YAW));
    if (!(-max_vx_ <= X_t(IDX::VX) && X_t(IDX::VX) <= max_vx_)) {
      X_t(IDX::VX) = X_t(IDX::VX) < 0 ? -max_vx_ : max_vx_;
    }
    if (!(-max_wz_ <= X_t(IDX::WZ) && X_t(IDX::WZ) <= max_wz_)) {
      X_t(IDX::WZ) = X_t(IDX::WZ) < 0 ? -max_wz_ : max_wz_;
    }
    ekf_.init(X_t, P_t);
  }

  // position z
  constexpr float gain = 0.9;
  z_ = gain * z_ + (1.0 - gain) * object.state.pose_covariance.pose.position.z;

  return true;
}

bool BicycleTracker::measureWithShape(const autoware_perception_msgs::msg::DynamicObject & object)
{
  if (object.shape.type != autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return false;
  }
  constexpr float gain = 0.9;

  bounding_box_.width = gain * bounding_box_.width + (1.0 - gain) * object.shape.dimensions.x;
  bounding_box_.length = gain * bounding_box_.length + (1.0 - gain) * object.shape.dimensions.y;
  bounding_box_.height = gain * bounding_box_.height + (1.0 - gain) * object.shape.dimensions.z;

  return true;
}

bool BicycleTracker::measure(
  const autoware_perception_msgs::msg::DynamicObject & object, const rclcpp::Time & time)
{
  object_ = object;

  if (0.01 /*10msec*/ < std::fabs((time - last_update_time_).seconds())) {
    RCLCPP_WARN(
      logger_, "There is a large gap between predicted time and measurement time. (%f)",
      (time - last_update_time_).seconds());
  }

  measureWithPose(object);
  measureWithShape(object);

  return true;
}

bool BicycleTracker::getEstimatedDynamicObject(
  const rclcpp::Time & time, autoware_perception_msgs::msg::DynamicObject & object) const
{
  object = object_;
  object.id = getUUID();
  object.semantic.type = getType();

  // predict state
  KalmanFilter tmp_ekf_for_no_update = ekf_;
  const double dt = (time - last_update_time_).seconds();
  if (0.001 /*1msec*/ < dt) {
    predict(dt, tmp_ekf_for_no_update);
  }
  Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);                // predicted state
  Eigen::MatrixXd P(ekf_params_.dim_x, ekf_params_.dim_x);  // predicted state
  tmp_ekf_for_no_update.getX(X_t);
  tmp_ekf_for_no_update.getP(P);

  // set position
  object.state.pose_covariance.pose.position.x = X_t(IDX::X);
  object.state.pose_covariance.pose.position.y = X_t(IDX::Y);
  object.state.pose_covariance.pose.position.z = z_;

  // set yaw
  {
    double roll, pitch, yaw;
    tf2::Quaternion original_quaternion;
    tf2::fromMsg(object_.state.pose_covariance.pose.orientation, original_quaternion);
    tf2::Matrix3x3(original_quaternion).getRPY(roll, pitch, yaw);
    tf2::Quaternion filtered_quaternion;
    filtered_quaternion.setRPY(roll, pitch, X_t(IDX::YAW));
    object.state.pose_covariance.pose.orientation.x = filtered_quaternion.x();
    object.state.pose_covariance.pose.orientation.y = filtered_quaternion.y();
    object.state.pose_covariance.pose.orientation.z = filtered_quaternion.z();
    object.state.pose_covariance.pose.orientation.w = filtered_quaternion.w();
  }

  // set covariance
  object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_X] = P(IDX::X, IDX::X);
  object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_Y] = P(IDX::X, IDX::Y);
  object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_YAW] = P(IDX::X, IDX::YAW);
  object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_X] = P(IDX::Y, IDX::X);
  object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_Y] = P(IDX::Y, IDX::Y);
  object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_YAW] = P(IDX::Y, IDX::YAW);
  object.state.pose_covariance.covariance[utils::MSG_COV_IDX::YAW_X] = P(IDX::YAW, IDX::X);
  object.state.pose_covariance.covariance[utils::MSG_COV_IDX::YAW_Y] = P(IDX::YAW, IDX::Y);
  object.state.pose_covariance.covariance[utils::MSG_COV_IDX::YAW_YAW] = P(IDX::YAW, IDX::YAW);
  object.state.twist_covariance.covariance[utils::MSG_COV_IDX::X_X] = P(IDX::VX, IDX::VX);
  object.state.twist_covariance.covariance[utils::MSG_COV_IDX::X_YAW] = P(IDX::VX, IDX::WZ);
  object.state.twist_covariance.covariance[utils::MSG_COV_IDX::YAW_X] = P(IDX::WZ, IDX::VX);
  object.state.twist_covariance.covariance[utils::MSG_COV_IDX::YAW_YAW] = P(IDX::WZ, IDX::WZ);

  // set shape
  object.shape.dimensions.x = bounding_box_.width;
  object.shape.dimensions.y = bounding_box_.length;
  object.shape.dimensions.z = bounding_box_.height;

  // set velocity
  object.state.twist_covariance.twist.linear.x = X_t(IDX::VX);
  object.state.twist_covariance.twist.angular.z = X_t(IDX::WZ);

  object.state.twist_reliable = true;

  return true;
}
