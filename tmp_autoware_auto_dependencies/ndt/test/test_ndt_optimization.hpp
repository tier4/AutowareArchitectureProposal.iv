// Copyright 2019 the Autoware Foundation
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

#ifndef TEST_NDT_OPTIMIZATION_HPP_
#define TEST_NDT_OPTIMIZATION_HPP_

#include <ndt/ndt_optimization_problem.hpp>
#include <pcl/registration/ndt.h>
#include <gtest/gtest.h>
#include <vector>
#include <algorithm>
#include <limits>
#include "test_ndt_map.hpp"
#include "test_ndt_utils.hpp"
#include "common/types.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;

using autoware::localization::ndt::Real;
using autoware::localization::ndt::EigenPose;

pcl::PointCloud<pcl::PointXYZ> from_pointcloud2(const sensor_msgs::msg::PointCloud2 & msg);

class OptimizationTestContext : public DenseNDTMapContext
{
public:
  using Cloud = sensor_msgs::msg::PointCloud2;
  OptimizationTestContext()
  : DenseNDTMapContext(), m_grid_config(m_min_point, m_max_point, m_voxel_size,
      m_capacity), m_dynamic_map(m_grid_config)
  {
    // build a pointcloud map.
    // It contains 5*5*5*7 points where each cell would have a center
    // (ranging from (1,1,1) to (5,5,5))
    // and 6 surrounding points with a 0.3 distance from the center
    build_pc(m_grid_config);
    std::vector<Eigen::Vector3d> down_sampled_pts;

    std::transform(
      m_voxel_centers.begin(), m_voxel_centers.end(),
      std::back_inserter(down_sampled_pts),
      [](auto it) {return it.second;});

    m_downsampled_cloud = make_pcl(down_sampled_pts);

    set_up_maps();
  }

  void set_up_maps()
  {
    // Set up the dynamic map
    m_dynamic_map.insert(m_pc);

    // Pass the dynamic map to a static one:
    sensor_msgs::msg::PointCloud2 serialized_map;
    m_dynamic_map.serialize_as<decltype(m_static_map)>(serialized_map);
    m_static_map.set(serialized_map);
  }
  autoware::perception::filters::voxel_grid::Config m_grid_config;
  sensor_msgs::msg::PointCloud2 m_downsampled_cloud;
  DynamicNDTMap m_dynamic_map;
  autoware::localization::ndt::StaticNDTMap m_static_map;
};

template<typename Problem, typename DomainValueT>
void numerical_diff(
  Problem & problem, DomainValueT pose,
  typename Problem::JacobianRef jac_numeric,
  typename Problem::HessianRef hess_numeric
)
{
  const auto eps = std::cbrt(std::numeric_limits<Real>::epsilon());
  for (auto i = 0U; i < jac_numeric.rows(); ++i) {
    // compute jacobian element by differentiating the function
    {
      decltype(pose) pose_1{pose};
      decltype(pose) pose_2{pose};
      pose_1(i) -= eps;
      pose_2(i) += eps;
      const auto res1 = problem(pose_1);
      const auto res2 = problem(pose_2);
      jac_numeric(i) = (res2 - res1) / (2 * eps);
    }

    // compute hessian element by differentiating the analytical jacobian
    for (auto j = 0U; j < hess_numeric.cols(); ++j) {
      decltype(pose) pose_1{pose};
      decltype(pose) pose_2{pose};
      pose_1(j) -= eps;
      pose_2(j) += eps;
      typename std::decay_t<decltype(problem)>::Jacobian jacobian_1;
      typename std::decay_t<decltype(problem)>::Jacobian jacobian_2;
      problem.jacobian(pose_1, jacobian_1);
      problem.jacobian(pose_2, jacobian_2);
      const auto res1 = jacobian_1(i);
      const auto res2 = jacobian_2(i);
      hess_numeric(i, j) = (res2 - res1) / (2 * eps);
    }
  }
}

struct OptTestParams
{
  OptTestParams(
    float64_t x, float64_t y, float64_t z, float64_t ang_x, float64_t ang_y, float64_t ang_z,
    bool8_t large, bool8_t check_pcl);
  EigenPose<Real> diff;
  bool8_t is_large;
  bool8_t check_pcl;
};

template<typename T>
void is_pose_approx(
  const EigenPose<T> & pose1, const EigenPose<T> & pose2, Real trans_tol,
  Real rot_tol)
{
  // (a and b are in same direction if `a x b = 0` and `a * b > 0`)
  Eigen::Matrix<T, 3U, 1U> t1 = pose1.head(3);
  Eigen::Matrix<T, 3U, 1U> t2 = pose2.head(3);
  const T t_dot_prod = t1.dot(t2);
  const Eigen::Vector3d t_cross_prod = t1.cross(t2);
  EXPECT_TRUE(t_cross_prod.isZero(trans_tol));
  EXPECT_GT(t_dot_prod, 0.0);

  // Since comparing two set of euler angles is more ambiguous, I am turning one of them into a
  // quaternion and then using `check_quat_RPY_eq`
  Eigen::Matrix<T, 3U, 1U> rotation1 = pose1.tail(3);
  Eigen::Matrix<T, 3U, 1U> rotation2 = pose2.tail(3);
  Eigen::Quaternion<T> quat1;
  Eigen::Quaternion<T> quat2;
  set_RPY_XYZ(quat1, _x(rotation1), _y(rotation1), _z(rotation1));
  set_RPY_XYZ(quat2, _x(rotation2), _y(rotation2), _z(rotation2));
  const T r_dot_prod = quat1.dot(quat2);
  // two unit quaternions are similar if their dot products are close to '1.0' or '-1.0'
  EXPECT_NEAR(std::fabs(r_dot_prod), 1.0, rot_tol);
}

#endif  // TEST_NDT_OPTIMIZATION_HPP_
