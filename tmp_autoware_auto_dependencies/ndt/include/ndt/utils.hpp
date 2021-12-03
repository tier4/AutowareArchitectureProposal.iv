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

#ifndef NDT__UTILS_HPP_
#define NDT__UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <helper_functions/float_comparisons.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <limits>
#include <tuple>

namespace autoware
{
namespace localization
{
namespace ndt
{

struct PointWithCovariances
{
  double x;
  double y;
  double z;
  double icov_xx;
  double icov_xy;
  double icov_xz;
  double icov_yy;
  double icov_yz;
  double icov_zz;
  friend bool operator==(const PointWithCovariances & p1, const PointWithCovariances & p2)
  {
    constexpr auto eps = std::numeric_limits<double>::epsilon();
    return common::helper_functions::comparisons::rel_eq(p1.x, p2.x, eps) &&
           common::helper_functions::comparisons::rel_eq(p1.y, p2.y, eps) &&
           common::helper_functions::comparisons::rel_eq(p1.z, p2.z, eps) &&
           common::helper_functions::comparisons::rel_eq(p1.icov_xx, p2.icov_xx, eps) &&
           common::helper_functions::comparisons::rel_eq(p1.icov_xy, p2.icov_xy, eps) &&
           common::helper_functions::comparisons::rel_eq(p1.icov_xz, p2.icov_xz, eps) &&
           common::helper_functions::comparisons::rel_eq(p1.icov_yy, p2.icov_yy, eps) &&
           common::helper_functions::comparisons::rel_eq(p1.icov_yz, p2.icov_yz, eps) &&
           common::helper_functions::comparisons::rel_eq(p1.icov_zz, p2.icov_zz, eps);
  }
};
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(icov_xx);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(icov_xy);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(icov_xz);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(icov_yy);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(icov_yz);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(icov_zz);

using PointWithCovariancesFieldGenerators = std::tuple<
  point_cloud_msg_wrapper::field_x_generator,
  point_cloud_msg_wrapper::field_y_generator,
  point_cloud_msg_wrapper::field_z_generator,
  field_icov_xx_generator,
  field_icov_xy_generator,
  field_icov_xz_generator,
  field_icov_yy_generator,
  field_icov_yz_generator,
  field_icov_zz_generator>;

using NdtMapCloudModifier =
  point_cloud_msg_wrapper::PointCloud2Modifier<PointWithCovariances,
    PointWithCovariancesFieldGenerators>;

using NdtMapCloudView =
  point_cloud_msg_wrapper::PointCloud2View<PointWithCovariances,
    PointWithCovariancesFieldGenerators>;

/// This function will check if the covariance is valid based on its eigenvalues. If the covariance
/// is valid, eigen values smaller than a fraction of the biggest eigen value will be capped to the
/// threshold. Covariance then will be reconstructed from the modified set of eigen values and
/// vectors. This should result in increased numerical stability as stated in [Magnusson 2009].
/// If the covariance is invalid, it will not be updated and false will be returned.
/// \tparam Derived Deduced Eigen Matrix type.
/// \param covariance [in, out] Covariance matrix to get stabilized.
/// \param scaling_factor [in] The ratio between the max. eigen value and the minimum
/// allowed eigenvalue. Default value is 0.01 as suggested in [Magnusson 2009], page 60.
/// \return True if the covariance matrix is valid.
template<typename Derived>
bool try_stabilize_covariance(
  Eigen::MatrixBase<Derived> & covariance,
  typename Derived::PlainMatrix::Scalar scaling_factor = 0.1)
{
  using CovMatrixT = typename Derived::PlainMatrix;
  using ScalarT = typename CovMatrixT::Scalar;
  using IndexT = typename CovMatrixT::Index;
  using SolverT = Eigen::SelfAdjointEigenSolver<CovMatrixT>;
  using VectorT = typename SolverT::RealVectorType;

  Eigen::SelfAdjointEigenSolver<CovMatrixT> solver(covariance);
  if (solver.info() != Eigen::Success) {return false;}

  VectorT eigen_values = solver.eigenvalues();  // Sorted in increasing order.
  const auto min_eigen_value = eigen_values[0];
  const auto max_eigen_value = eigen_values[eigen_values.size() - IndexT{1U}];
  const auto stabilized_min_eigen_value = scaling_factor * max_eigen_value;
  if (stabilized_min_eigen_value < std::numeric_limits<ScalarT>::epsilon()) {return false;}
  if (min_eigen_value > scaling_factor * max_eigen_value) {return true;}  // Already stable.
  for (auto i = IndexT{0U}; i < eigen_values.size(); ++i) {
    eigen_values[i] = std::max(eigen_values[i], stabilized_min_eigen_value);
  }
  covariance =
    solver.eigenvectors() * eigen_values.asDiagonal() * solver.eigenvectors().transpose();
  return true;
}

template<typename T>
using EigenPose = Eigen::Matrix<T, 6U, 1U>;
template<typename T>
using EigenTransform = Eigen::Transform<T, 3, Eigen::Affine, Eigen::ColMajor>;
using RosTransform = geometry_msgs::msg::Transform;
using RosPose = geometry_msgs::msg::Pose;
namespace transform_adapters
{
/// Template function to convert a 6D pose to a transformation matrix.
/// This function should be specialized and implemented for the supported types.
/// \tparam PoseT Pose type.
/// \tparam TransformT Transform type.
/// \param[in] pose pose to convert
/// \param[out] transform resulting transform
template<typename PoseT, typename TransformT>
void pose_to_transform(const PoseT & pose, TransformT & transform);

/// Template function to convert a 6D pose to a transformation matrix.
/// This function should be specialized and implemented for the supported types.
/// \tparam PoseT Pose type.
/// \tparam TransformT Transform type.
/// \param[in] transform resulting transform
/// \param[out] pose pose to convert
template<typename PoseT, typename TransformT>
void transform_to_pose(const TransformT & transform, PoseT & pose);


template<typename T>
void pose_to_transform(
  const EigenPose<T> & pose,
  EigenTransform<T> & transform)
{
  static_assert(std::is_floating_point<T>::value, "Eigen transform should use floating points");
  transform.setIdentity();
  transform.translation() = pose.head(3);
  transform.rotate(Eigen::AngleAxis<T>(pose(3), Eigen::Vector3d::UnitX()));
  transform.rotate(Eigen::AngleAxis<T>(pose(4), Eigen::Vector3d::UnitY()));
  transform.rotate(Eigen::AngleAxis<T>(pose(5), Eigen::Vector3d::UnitZ()));
}

/// Specialization to convert from the eigen pose to the ros transform type.
/// \tparam T Eigen scalar type
template<typename T>
void pose_to_transform(
  const EigenPose<T> & pose,
  RosTransform & transform)
{
  static_assert(std::is_floating_point<T>::value, "Eigen pose should use floating points");
  Eigen::Quaternion<T> eig_rot{Eigen::Quaternion<T>{}.setIdentity()};
  eig_rot.setIdentity();
  eig_rot =
    Eigen::AngleAxis<T>(pose(3), Eigen::Matrix<T, 3, 1>::UnitX()) *
    Eigen::AngleAxis<T>(pose(4), Eigen::Matrix<T, 3, 1>::UnitY()) *
    Eigen::AngleAxis<T>(pose(5), Eigen::Matrix<T, 3, 1>::UnitZ());

  decltype(RosTransform::translation) trans;
  decltype(RosTransform::rotation) rot;

  trans.set__x(pose(0)).set__y(pose(1)).set__z(pose(2));
  transform.set__translation(trans);

  rot.set__x(eig_rot.x()).
  set__y(eig_rot.y()).
  set__z(eig_rot.z()).
  set__w(eig_rot.w());
  transform.set__rotation(rot);
}

/// Specialization to convert from the eigen pose to the ros pose type.
/// `pose_to_transform` template is used as conversion to `RosPose`
/// is identical to conversion to `RosTransform`
/// \tparam T Eigen scalar type
template<typename T>
void pose_to_transform(
  const EigenPose<T> & pose,
  RosPose & ros_pose)
{
  static_assert(std::is_floating_point<T>::value, "Eigen pose should use floating points");
  Eigen::Quaternion<T> eig_rot{Eigen::Quaternion<T>{}.setIdentity()};
  eig_rot =
    Eigen::AngleAxis<T>(pose(3), Eigen::Matrix<T, 3, 1>::UnitX()) *
    Eigen::AngleAxis<T>(pose(4), Eigen::Matrix<T, 3, 1>::UnitY()) *
    Eigen::AngleAxis<T>(pose(5), Eigen::Matrix<T, 3, 1>::UnitZ());

  decltype(RosPose::position) trans;
  decltype(RosTransform::rotation) rot;

  trans.set__x(pose(0)).set__y(pose(1)).set__z(pose(2));
  ros_pose.set__position(trans);

  rot.set__x(eig_rot.x()).
  set__y(eig_rot.y()).
  set__z(eig_rot.z()).
  set__w(eig_rot.w());
  ros_pose.set__orientation(rot);
}

/// Specialization to convert from the ros pose type to an eigen one.
/// \tparam T Eigen scalar type
template<typename T>
void transform_to_pose(const RosTransform & transform, EigenPose<T> & pose)
{
  static_assert(std::is_floating_point<T>::value, "Eigen pose should use floating points");
  const auto & ros_rot = transform.rotation;
  const auto & ros_trans = transform.translation;
  Eigen::Quaternion<T> eig_rot{ros_rot.w, ros_rot.x, ros_rot.y, ros_rot.z};
  pose(0) = ros_trans.x;
  pose(1) = ros_trans.y;
  pose(2) = ros_trans.z;

  const auto rot = eig_rot.matrix().eulerAngles(0, 1, 2);
  pose(3) = rot(0);
  pose(4) = rot(1);
  pose(5) = rot(2);
}

/// Specialization to convert from the ros pose type to an eigen one.
/// `transform_to_pose` template is used as conversion from `RosPose`
/// is identical to conversion from `RosTransform`
/// \tparam T Eigen scalar type
template<typename T>
void transform_to_pose(const RosPose & ros_pose, EigenPose<T> & pose)
{
  static_assert(std::is_floating_point<T>::value, "Eigen pose should use floating points");
  const auto & ros_rot = ros_pose.orientation;
  const auto & ros_trans = ros_pose.position;
  Eigen::Quaternion<T> eig_rot{ros_rot.w, ros_rot.x, ros_rot.y, ros_rot.z};
  pose(0) = ros_trans.x;
  pose(1) = ros_trans.y;
  pose(2) = ros_trans.z;

  const auto rot = eig_rot.matrix().eulerAngles(0, 1, 2);
  pose(3) = rot(0);
  pose(4) = rot(1);
  pose(5) = rot(2);
}

}  // namespace transform_adapters
}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // NDT__UTILS_HPP_
