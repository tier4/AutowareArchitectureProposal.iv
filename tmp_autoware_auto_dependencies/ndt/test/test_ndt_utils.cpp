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

#include <gtest/gtest.h>
#include <ndt/utils.hpp>
#include <Eigen/Core>
#include <random>
#include <vector>
#include <algorithm>
#include "test_ndt_utils.hpp"

using autoware::localization::ndt::try_stabilize_covariance;
using autoware::localization::ndt::transform_adapters::pose_to_transform;
using autoware::localization::ndt::transform_adapters::transform_to_pose;

PoseParams::PoseParams(
  double x, double y, double z, double ang_x, double ang_y, double ang_z)
{
  // euler angles XYZ
  pose << x, y, z, ang_x, ang_y, ang_z;
}

PoseParams::PoseParams(double translation_range, double rotation_range)
{
  std::uniform_real_distribution<double> uni_trans(-translation_range, translation_range);
  std::uniform_real_distribution<double> uni_rot(-rotation_range, rotation_range);
  std::default_random_engine re;
  for (auto i = 0U; i < 3U; ++i) {
    pose(i) = uni_trans(re);
  }
  for (auto i = 3U; i < 6U; ++i) {
    pose(i) = uni_rot(re);
  }
}

Cov3x3Param::Cov3x3Param(Real e1_, Real e2_, Real e3_, bool valid_)
: e1{e1_}, e2{e2_}, e3{e3_}, valid{valid_}
{
  Eigen::Vector3d e_vals{e1, e2, e3};
  // Fixed diagonal matrix as the eigen vectors for simplicity.
  Eigen::Vector3d e_vec_scales{2, 7, 13};
  CovMatrix e_vecs = e_vec_scales.asDiagonal();
  cov = e_vecs * e_vals.asDiagonal() * e_vecs.inverse();
}

class TestTransformAdapters : public ::testing::TestWithParam<PoseParams> {};

class CovarianceStabilityTest : public ::testing::TestWithParam<Cov3x3Param> {};

TEST_P(TestTransformAdapters, PoseToTransform) {
  const EigenPose<Real> pose = GetParam().pose;
  RosTransform ros_transform;
  RosPose ros_pose;
  EigenTransform<Real> eig_transform;

  pose_to_transform(pose, ros_transform);
  pose_to_transform(pose, ros_pose);
  pose_to_transform(pose, eig_transform);

  compare(pose, ros_transform);
  compare(pose, ros_pose);
  compare(pose, eig_transform);
}

TEST_P(TestTransformAdapters, TransformToPose) {
  const EigenPose<Real> pose = GetParam().pose;
  RosTransform ros_transform;
  RosPose ros_pose;

  make_transform(pose, ros_transform);
  make_transform(pose, ros_pose);

  {
    EigenPose<Real> pose_result;
    transform_to_pose(ros_transform, pose_result);
    compare(pose_result, ros_transform);
  }

  {
    EigenPose<Real> pose_result;
    transform_to_pose(ros_pose, pose_result);
    compare(pose_result, ros_pose);
  }
}

TEST_P(TestTransformAdapters, PoseToTransformToPose) {
  const EigenPose<Real> pose = GetParam().pose;
  RosTransform ros_transform;
  RosPose ros_pose;

  pose_to_transform(pose, ros_transform);
  pose_to_transform(pose, ros_pose);
  {
    EigenPose<Real> pose_result;
    transform_to_pose(ros_transform, pose_result);
    compare(pose, pose_result);
  }

  {
    EigenPose<Real> pose_result;
    transform_to_pose(ros_pose, pose_result);
    compare(pose, pose_result);
  }
}

INSTANTIATE_TEST_SUITE_P(
  TransformParamTests, TestTransformAdapters,
  ::testing::Values(
    PoseParams{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    PoseParams{0.5, 0.9, 0.1, 1.0, -3.1, 0.05},
    PoseParams{2.5, -1.9, 0.1, -2.1, 0.1, 3.05},
    PoseParams{2.5, -1.9, 64, -2.1, 31, -1.2},
    PoseParams{0.001, 1.00009, -1.0, 0.0, 0.75, 0.13}
    // cppcheck-suppress syntaxError
  ), );

INSTANTIATE_TEST_SUITE_P(
  FuzzedTests, TestTransformAdapters,
  ::testing::Values(
    PoseParams{0.1, 50.0},
    PoseParams{100.0, 0.005},
    PoseParams{25.5, 4.5}
    // cppcheck-suppress syntaxError
  ), );

TEST_P(CovarianceStabilityTest, Basic) {
  Cov3x3Param::CovMatrix covariance = GetParam().cov;
  std::vector<Real> e_vals{GetParam().e1, GetParam().e2, GetParam().e3};
  const auto scale = 0.01;
  std::vector<Real> filtered_evals;
  auto min_e_val = *std::max_element(e_vals.begin(), e_vals.end()) * scale;
  std::transform(
    e_vals.begin(), e_vals.end(), std::back_inserter(filtered_evals),
    [min_e_val](Real & e) {return e < min_e_val ? min_e_val : e;});

  const auto expected_valid = GetParam().valid;

  EXPECT_EQ(try_stabilize_covariance(covariance, scale), expected_valid);

  if (expected_valid) {
    EXPECT_TRUE(covariance.diagonal().minCoeff() >= scale * covariance.diagonal().maxCoeff());
    Eigen::SelfAdjointEigenSolver<Cov3x3Param::CovMatrix> solver;
    solver.compute(covariance);
    Eigen::Vector3d stable_e_vals = solver.eigenvalues();

    std::sort(filtered_evals.begin(), filtered_evals.end());
    std::sort(stable_e_vals.data(), stable_e_vals.data() + stable_e_vals.size());

    for (auto i = 0U; i < filtered_evals.size(); ++i) {
      EXPECT_FLOAT_EQ(filtered_evals[i], stable_e_vals(i));
    }
  }
}

// 3 eigen values and the expected validity of the covariance matrix.
INSTANTIATE_TEST_SUITE_P(
  Basic, CovarianceStabilityTest,
  ::testing::Values(
    Cov3x3Param(1e-2, 65.0, 24, true),
    Cov3x3Param(-12., 24., 65., true),
    Cov3x3Param(-12., -24., -65., false),
    Cov3x3Param(0.0, 0.0, 0.0, false),
    Cov3x3Param(1.0, 1.0, 0.0, true),
    Cov3x3Param(1e-2, 1e-3, 65, true),
    Cov3x3Param(1e-2, 1e-3, 1e-15, true)
    // cppcheck-suppress syntaxError
  ), );
