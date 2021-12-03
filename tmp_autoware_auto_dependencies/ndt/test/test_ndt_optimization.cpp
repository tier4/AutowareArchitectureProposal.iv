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

#include "test_ndt_optimization.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>
#include "common/types.hpp"

namespace
{
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

using autoware::localization::ndt::P2DNDTScan;
using autoware::localization::ndt::P2DNDTOptimizationProblem;
using autoware::localization::ndt::P2DNDTOptimizationConfig;
using autoware::localization::ndt::transform_adapters::pose_to_transform;

using P2DProblem = P2DNDTOptimizationProblem<autoware::localization::ndt::StaticNDTMap>;

constexpr double kPoseEpsilon{0.01};

sensor_msgs::msg::PointCloud2 create_xz_plane_point_cloud(
  const PointXYZI & corner,
  const float size,
  const float step)
{
  sensor_msgs::msg::PointCloud2 msg;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> msg_modifier{msg, "map"};
  const auto intensity = 1.0F;
  msg_modifier.push_back({corner.x, corner.y, corner.z, intensity});
  for (auto coord = step; coord < size; coord += step) {
    msg_modifier.push_back({corner.x + coord, corner.y, corner.z, intensity});
    msg_modifier.push_back({corner.x, corner.y, corner.z + coord, intensity});
  }
  return msg;
}
}  // namespace

OptTestParams::OptTestParams(
  float64_t x, float64_t y, float64_t z, float64_t ang_x, float64_t ang_y, float64_t ang_z,
  bool8_t large, bool8_t check_pcl)
: is_large{large}, check_pcl{check_pcl}
{
  // euler angles XYZ
  diff << x, y, z, ang_x, ang_y, ang_z;
}

class P2DOptimizationTest : public OptimizationTestContext, public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_EQ(m_dynamic_map.size(), m_static_map.size());
    for (const auto & pt_it : m_voxel_centers) {
      const auto static_vx = m_static_map.cell(pt_it.second)[0U];
      const auto dynamic_vx = m_dynamic_map.cell(pt_it.second)[0U];
      ASSERT_TRUE(static_vx.usable());
      ASSERT_TRUE(dynamic_vx.usable());

      ASSERT_TRUE(
        static_vx.centroid().isApprox(
          dynamic_vx.centroid(),
          std::numeric_limits<Real>::epsilon()));
      ASSERT_TRUE(
        static_vx.inverse_covariance().isApprox(
          dynamic_vx.inverse_covariance(),
          std::numeric_limits<Real>::epsilon() * 1e2));
    }
  }
};

class AlignmentXyzTest : public P2DOptimizationTest,
  public ::testing::WithParamInterface<OptTestParams> {};

class P2DOptimizationNumericalTest : public P2DOptimizationTest,
  public ::testing::WithParamInterface<OptTestParams> {};

TEST_P(P2DOptimizationNumericalTest, NumericalAnalysis) {
  {
    // m_pc is also used as the map, so this scan perfectly aligns with the map
    P2DNDTScan matching_scan(m_downsampled_cloud, m_downsampled_cloud.width);
    P2DProblem problem{matching_scan, m_static_map, P2DNDTOptimizationConfig{0.55}};

    EigenPose<Real> pose = GetParam().diff;
    problem.evaluate(pose, autoware::common::optimization::ComputeMode{true, true, true});
    P2DProblem::Jacobian jacobian;
    P2DProblem::Hessian hessian;
    problem.jacobian(pose, jacobian);
    problem.hessian(pose, hessian);

    decltype(hessian) numerical_hessian;
    decltype(jacobian) numerical_jacobian;
    numerical_diff(problem, pose, numerical_jacobian, numerical_hessian);
    // Compare numerical and analytical values
    // Linearization error is higher on the hessian
    constexpr auto zero_eps = 1e-4;
    constexpr auto jacob_eps = 1e-6;
    constexpr auto hessian_eps = 1e-1;
    // Zero matrices cannot be compared by `isApprox(...)`. If the below checks fail, that means
    // both of them are zero, therefore equal.
    if (!jacobian.isZero(zero_eps) || !numerical_jacobian.isZero(zero_eps)) {
      EXPECT_TRUE(jacobian.isApprox(numerical_jacobian, jacob_eps));
    }
    if (!hessian.isZero(zero_eps) || numerical_hessian.isZero(zero_eps)) {
      EXPECT_TRUE(hessian.isApprox(numerical_hessian, hessian_eps));
    }
  }
}
/// @test       The shape is fitting exactly into a single voxel. Its copy is moved in different
///             directions and aligned with the original.
TEST_P(AlignmentXyzTest, AlignShapesWithinOneVoxel) {
  const auto & param = GetParam();
  EigenPose<Real> diff = param.diff;

  geometry_msgs::msg::TransformStamped diff_tf2;
  diff_tf2.header.frame_id = "custom";
  pose_to_transform(diff, diff_tf2.transform);

  // TODO(#1035): picking this smaller e.g. 0.31 breaks the test. We need to investigate why.
  const auto shape_size = 0.4F;
  const auto voxel_side_size = 0.5F;
  const auto half_shape_size = 0.5F * shape_size;
  const PointXYZI corner_coord{-half_shape_size, 0.0F, -half_shape_size, 0.0F};
  ASSERT_LT(shape_size, voxel_side_size);
  ASSERT_LT(corner_coord.x + shape_size + diff.x(), voxel_side_size) <<
    "Moving the shape will move it out of the voxel which is not allowed in this test.";
  ASSERT_LT(corner_coord.y + diff.y(), voxel_side_size) <<
    "Moving the shape will move it out of the voxel which is not allowed in this test.";
  ASSERT_LT(corner_coord.z + shape_size + diff.z(), voxel_side_size) <<
    "Moving the shape will move it out of the voxel which is not allowed in this test.";

  // Pick a bigger grid but offset it by half the voxel size to make sure 0,0,0 is in the center of
  // a voxel in the middle of the grid.
  const auto half_grid_extent = 2.0F + 0.5F * voxel_side_size;
  geometry_msgs::msg::Point32 min_point;
  min_point.set__x(-half_grid_extent).set__y(-half_grid_extent).set__z(-half_grid_extent);
  geometry_msgs::msg::Point32 max_point;
  max_point.set__x(half_grid_extent).set__y(half_grid_extent).set__z(half_grid_extent);
  geometry_msgs::msg::Point32 voxel_size;
  voxel_size.set__x(voxel_side_size).set__y(voxel_side_size).set__z(voxel_side_size);
  const auto capacity = 1000U;
  autoware::perception::filters::voxel_grid::Config grid_config{
    min_point,
    max_point,
    voxel_size,
    capacity};
  // Create a plane-like point cloud that fits within a single voxel of the grid. The coordinates
  // are picked so that this plane lies in the center of the center-most voxel of the grid.
  const auto step = 0.01F;
  const auto initial_point_cloud = create_xz_plane_point_cloud(corner_coord, shape_size, step);
  ASSERT_GT(initial_point_cloud.width, 1U);
  autoware::localization::ndt::DynamicNDTMap dynamic_map{grid_config};
  dynamic_map.insert(initial_point_cloud);
  ASSERT_EQ(dynamic_map.size(), 1U);

  autoware::localization::ndt::StaticNDTMap static_map{};
  sensor_msgs::msg::PointCloud2 serialized_dynamic_map;
  dynamic_map.serialize_as<decltype(static_map)>(serialized_dynamic_map);
  static_map.set(serialized_dynamic_map);
  ASSERT_EQ(static_map.size(), 1U);

  auto translated_cloud = initial_point_cloud;
  tf2::doTransform(initial_point_cloud, translated_cloud, diff_tf2);

  P2DProblem::DomainValue guess = P2DProblem::DomainValue::Zero();
  const auto step_length = 0.1F;
  const auto num_iters = 100U;
  P2DNDTScan scan{translated_cloud, translated_cloud.width};
  // TODO(#1062): we have no outliers here, but setting this to 0.0 fails the test. Investigate.
  const auto dummy_outlier_ratio = 0.01F;
  P2DProblem problem{scan, static_map, P2DNDTOptimizationConfig{dummy_outlier_ratio}};

  for (auto i = 0U; i < num_iters; ++i) {
    problem.evaluate(
      guess,
      autoware::common::optimization::ComputeMode{}.set_score().set_jacobian()
      .set_hessian());
    P2DProblem::Jacobian jacobian;
    P2DProblem::Hessian hessian;
    problem.jacobian(guess, jacobian);
    problem.hessian(guess, hessian);
    P2DProblem::DomainValue delta = hessian.ldlt().solve(-jacobian);
    guess += step_length * delta;
    auto guess_cloud = translated_cloud;
    pose_to_transform(guess, diff_tf2.transform);
    tf2::doTransform(translated_cloud, guess_cloud, diff_tf2);
  }

  const auto found_diff = P2DProblem::DomainValue{-guess};
  for (int i = 0; i < found_diff.size(); ++i) {
    EXPECT_NEAR(found_diff[i], diff[i], kPoseEpsilon) << "Not matching at index " << i;
  }
}


// TODO(#1063): when these parameters are picked bigger the tests fail. We need to investigate why.
INSTANTIATE_TEST_SUITE_P(
  AlignmentXyzTest, AlignmentXyzTest,
  ::testing::Values(
    OptTestParams{0.2, 0.0, 0.0, 0.0, 0.0, 0.0, false, false},
    OptTestParams{0.0, 0.1, 0.0, 0.0, 0.0, 0.0, false, false},
    OptTestParams{0.0, 0.0, 0.2, 0.0, 0.0, 0.0, false, false},
    OptTestParams{0.1, 0.05, 0.1, 0.0, 0.0, 0.0, false, false}
    // cppcheck-suppress syntaxError
  ), );

INSTANTIATE_TEST_SUITE_P(
  NumericalAnalysis, P2DOptimizationNumericalTest,
  ::testing::Values(
    OptTestParams{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true, false},
    OptTestParams{0.5, 0.9, 0.1, 1.0, -3.1, 0.05, true, false},
    OptTestParams{2.5, -1.9, 0.1, -2.1, 0.1, 3.05, true, false}
    // cppcheck-suppress syntaxError
  ), );


////////////////////////////////////// Test function implementations

pcl::PointCloud<pcl::PointXYZ> from_pointcloud2(const sensor_msgs::msg::PointCloud2 & msg)
{
  point_cloud_msg_wrapper::PointCloud2View<autoware::common::types::PointXYZI> msg_view{msg};
  pcl::PointCloud<pcl::PointXYZ> res{};

  for (const auto & pt_in : msg_view) {
    pcl::PointXYZ pt;
    pt.x = pt_in.x;
    pt.y = pt_in.y;
    pt.z = pt_in.z;
    res.push_back(pt);
  }
  return res;
}
