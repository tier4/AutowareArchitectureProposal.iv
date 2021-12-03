// Copyright 2020 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef TEST_NDT_UTILS_HPP_
#define TEST_NDT_UTILS_HPP_
#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <Eigen/Core>
#include "test_ndt_map.hpp"
#include "test_ndt_optimization.hpp"

template<typename PoseT, typename TransformT>
void compare_pose_transform(PoseT & pose, TransformT & transform);

template<typename T>
using EigenPose = Eigen::Matrix<T, 6U, 1U>;
template<typename T>
using EigenTransform = Eigen::Transform<T, 3, Eigen::Affine, Eigen::ColMajor>;
using RosTransform = geometry_msgs::msg::Transform;
using RosPose = geometry_msgs::msg::Pose;

using autoware::localization::ndt::Real;

struct PoseParams
{
  PoseParams(
    double x, double y, double z, double ang_x, double ang_y, double ang_z);

  PoseParams(double translation_range, double rotation_range);
  EigenPose<Real> pose;
};

struct Cov3x3Param
{
  using CovMatrix = Eigen::Matrix<Real, 3, 3>;
  Cov3x3Param(Real e1, Real e2, Real e3, bool valid);
  Real e1, e2, e3;
  CovMatrix cov;
  bool valid{false};
};

template<typename VectorT>
inline auto _x(VectorT & v)->decltype(v.x) &
{
  return v.x;
}

template<typename VectorT>
inline auto _x(VectorT & v)->decltype(v.x()) &
{
  return v.x();
}


template<typename VectorT>
inline auto _y(VectorT & v)->decltype(v.y) &
{
  return v.y;
}

template<typename VectorT>
inline auto _y(VectorT & v)->decltype(v.y()) &
{
  return v.y();
}

template<typename VectorT>
inline auto _z(VectorT & v)->decltype(v.z) &
{
  return v.z;
}

template<typename VectorT>
inline auto _z(VectorT & v)->decltype(v.z()) &
{
  return v.z();
}

template<typename VectorT>
inline auto _w(VectorT & v)->decltype(v.w) &
{
  return v.w;
}

template<typename VectorT>
inline auto _w(VectorT & v)->decltype(v.w()) &
{
  return v.w();
}


template<typename VectorT>
inline auto _x(const VectorT & v)->decltype(v.x) {
  return v.x;
}

template<typename VectorT>
inline auto _x(const VectorT & v)->decltype(v.x()) {
  return v.x();
}


template<typename VectorT>
inline auto _y(const VectorT & v)->decltype(v.y) {
  return v.y;
}

template<typename VectorT>
inline auto _y(const VectorT & v)->decltype(v.y()) {
  return v.y();
}

template<typename VectorT>
inline auto _z(const VectorT & v)->decltype(v.z) {
  return v.z;
}

template<typename VectorT>
inline auto _z(const VectorT & v)->decltype(v.z()) {
  return v.z();
}

template<typename VectorT>
inline auto _w(const VectorT & v)->decltype(v.w) {
  return v.w;
}

template<typename VectorT>
inline auto _w(const VectorT & v)->decltype(v.w()) {
  return v.w();
}


template<typename T1, typename T2>
void set_vector3d(const T1 & t1, T2 & t2)
{
  _x(t2) = _x(t1);
  _y(t2) = _y(t1);
  _z(t2) = _z(t1);
}

template<typename T1, typename T2>
void set_vector4d(const T1 & t1, T2 & t2)
{
  set_vector3d(t1, t2);
  _w(t2) = _w(t1);
}

// tf2 quaternion doesn't expose any references, so an explicit specialization is needed
template<typename T1>
void set_vector4d(const T1 & t1, tf2::Quaternion & t2)
{
  t2.setX(_x(t1));
  t2.setY(_y(t1));
  t2.setZ(_z(t1));
  t2.setW(_w(t1));
}

/// Euler angle -> quaternion conversion on (X,Y,Z) order.
/// Taken from (Henderson, 1977):
/// https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770024290.pdf
/// Page A-2
template<typename QuatT>
void set_RPY_XYZ(
  QuatT & quat,
  const tf2Scalar roll, const tf2Scalar pitch, const tf2Scalar yaw)
{
  tf2Scalar halfYaw = tf2Scalar(yaw) * tf2Scalar(0.5);
  tf2Scalar halfPitch = tf2Scalar(pitch) * tf2Scalar(0.5);
  tf2Scalar halfRoll = tf2Scalar(roll) * tf2Scalar(0.5);
  tf2Scalar cosYaw = tf2Cos(halfYaw);
  tf2Scalar sinYaw = tf2Sin(halfYaw);
  tf2Scalar cosPitch = tf2Cos(halfPitch);
  tf2Scalar sinPitch = tf2Sin(halfPitch);
  tf2Scalar cosRoll = tf2Cos(halfRoll);
  tf2Scalar sinRoll = tf2Sin(halfRoll);
  tf2::Quaternion quat_temp;
  quat_temp.setValue(
    sinRoll * cosPitch * cosYaw + cosRoll * sinPitch * sinYaw,                    // x
    -sinRoll * cosPitch * sinYaw + cosRoll * sinPitch * cosYaw,              // y
    sinRoll * sinPitch * cosYaw + cosRoll * cosPitch * sinYaw,             // z
    -sinRoll * sinPitch * sinYaw + cosRoll * cosPitch * cosYaw);              // w

  // Just making sure this function works for more types.
  set_vector4d(quat_temp, quat);
}

template<typename QuatT, typename AngleT>
void check_quat_RPY_eq(const QuatT & q, const AngleT & RPY)
{
  tf2::Quaternion tf2_quat{_x(q), _y(q), _z(q), _w(q)};
  tf2::Quaternion tf2_quat_from_RPY{};
  // I chose to implement the conversion explicitly based on an actual paper because
  // tf2::Quaternion::setRPY(...) seemed to follow (z,y,x) intrinsic order as seen on:
  // https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions
  set_RPY_XYZ(tf2_quat_from_RPY, _x(RPY), _y(RPY), _z(RPY));

  const auto product = tf2_quat.dot(tf2_quat_from_RPY);   // Either 1 or -1

  EXPECT_FLOAT_EQ(std::fabs(product), 1.0);
  EXPECT_FLOAT_EQ(_x(tf2_quat), product * _x(tf2_quat_from_RPY));
  EXPECT_FLOAT_EQ(_y(tf2_quat), product * _y(tf2_quat_from_RPY));
  EXPECT_FLOAT_EQ(_z(tf2_quat), product * _z(tf2_quat_from_RPY));
  EXPECT_FLOAT_EQ(_w(tf2_quat), product * _w(tf2_quat_from_RPY));
}

template<typename T1, typename T2>
void compare_vector3d_eq(const T1 & t1, const T2 & t2)
{
  EXPECT_FLOAT_EQ(_x(t1), _x(t2));
  EXPECT_FLOAT_EQ(_y(t1), _y(t2));
  EXPECT_FLOAT_EQ(_z(t1), _z(t2));
}

template<typename T1, typename T2>
void compare_vector4d_eq(const T1 & q1, const T2 & q2)
{
  compare_vector3d_eq(q1, q2);
  EXPECT_FLOAT_EQ(_w(q1), _w(q2));
}

template<typename T>
void compare(const EigenPose<T> & pose, const EigenTransform<T> & transform)
{
  Eigen::Matrix<T, 3U, 1U> translation = pose.head(3);
  Eigen::Matrix<T, 3U, 1U> rotation = pose.tail(3);
  compare_vector3d_eq(translation, transform.translation());
  check_quat_RPY_eq(Eigen::Quaternion<T>{transform.rotation()}, rotation);
}

template<typename T>
void compare(const EigenPose<T> & pose, const RosTransform & transform)
{
  Eigen::Matrix<T, 3U, 1U> translation = pose.head(3);
  Eigen::Matrix<T, 3U, 1U> rotation = pose.tail(3);
  compare_vector3d_eq(translation, transform.translation);
  check_quat_RPY_eq(transform.rotation, rotation);
}

template<typename T>
void compare(const EigenPose<T> & pose, const RosPose & ros_pose)
{
  Eigen::Matrix<T, 3U, 1U> translation = pose.head(3);
  Eigen::Matrix<T, 3U, 1U> rotation = pose.tail(3);
  compare_vector3d_eq(translation, ros_pose.position);
  check_quat_RPY_eq(ros_pose.orientation, rotation);
}

template<typename T>
void compare(const EigenPose<T> & pose1, const EigenPose<T> & pose2)
{
  Eigen::Matrix<T, 3U, 1U> translation1 = pose1.head(3);
  Eigen::Matrix<T, 3U, 1U> translation2 = pose2.head(3);
  compare_vector3d_eq(translation1, translation2);
  // Since comparing two set of euler angles is more ambiguous, I am turning one of them into a
  // quaternion and then using `check_quat_RPY_eq`
  Eigen::Matrix<T, 3U, 1U> rotation1 = pose1.tail(3);
  Eigen::Matrix<T, 3U, 1U> rotation2 = pose2.tail(3);
  tf2::Quaternion quat1;
  set_RPY_XYZ(quat1, _x(rotation1), _y(rotation1), _z(rotation1));
  check_quat_RPY_eq(quat1, rotation2);
}

//////////////////////////////////////////////////

template<typename T>
void make_transform(const EigenPose<T> & pose, RosTransform & transform)
{
  set_vector3d(pose.head(3), transform.translation);
  tf2::Quaternion tf2_rot;
  set_RPY_XYZ(tf2_rot, pose(3), pose(4), pose(5));
  set_vector4d(tf2_rot, transform.rotation);
}

template<typename T>
void make_transform(const EigenPose<T> & pose, RosPose & ros_pose)
{
  set_vector3d(pose.head(3), ros_pose.position);
  tf2::Quaternion tf2_rot;
  set_RPY_XYZ(tf2_rot, pose(3), pose(4), pose(5));
  set_vector4d(tf2_rot, ros_pose.orientation);
}

#endif  // TEST_NDT_UTILS_HPP_
