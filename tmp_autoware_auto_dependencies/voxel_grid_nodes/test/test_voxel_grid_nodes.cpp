// Copyright 2017-2021 the Autoware Foundation
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

#include <common/types.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <voxel_grid_nodes/algorithm/voxel_cloud_approximate.hpp>
#include <voxel_grid_nodes/algorithm/voxel_cloud_centroid.hpp>
#include <voxel_grid_nodes/voxel_cloud_node.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using autoware::perception::filters::voxel_grid::PointXYZ;
using autoware::perception::filters::voxel_grid::Config;

using autoware::perception::filters::voxel_grid_nodes::algorithm::VoxelCloudBase;
using autoware::perception::filters::voxel_grid_nodes::algorithm::VoxelCloudApproximate;
using autoware::perception::filters::voxel_grid_nodes::algorithm::VoxelCloudCentroid;

using autoware::common::types::PointXYZI;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

using autoware::perception::filters::voxel_grid_nodes::VoxelCloudNode;

class VoxelAlgorithm : public ::testing::Test
{
protected:
  PointXYZI make(const float32_t x, const float32_t y, const float32_t z)
  {
    PointXYZI ret;
    ret.x = x;
    ret.y = y;
    ret.z = z;
    return ret;
  }
  std::unique_ptr<Config> cfg_ptr;
  std::array<PointXYZI, 16U> obs_points1;
  std::array<PointXYZI, 8U> ref_points1;
  const std::size_t m_capacity;

public:
  VoxelAlgorithm()
  : m_capacity(10U)
  {
    // Initialize config
    PointXYZ min_point;
    min_point.x = -1.0F;
    min_point.y = -1.0F;
    min_point.z = -1.0F;
    PointXYZ max_point;
    max_point.x = 1.0F;
    max_point.y = 1.0F;
    max_point.z = 1.0F;
    PointXYZ voxel_size;
    voxel_size.x = 1.0F;
    voxel_size.y = 1.0F;
    voxel_size.z = 1.0F;
    cfg_ptr = std::make_unique<Config>(min_point, max_point, voxel_size, m_capacity);
    // List of points
    obs_points1 = {
      make(-1.0F, -1.0F, -1.0F),  // voxel 0
      make(-0.5F, -0.5F, -0.5F),
      make(1.0F, -1.0F, -1.0F),  // voxel 1
      make(0.5F, -0.5F, -0.5F),
      make(-1.0F, 1.0F, -1.0F),  // voxel 2
      make(-0.5F, 0.5F, -0.5F),
      make(1.0F, 1.0F, -1.0F),  // voxel 3
      make(0.5F, 0.5F, -0.5F),
      make(-1.0F, -1.0F, 1.0F),  // voxel 4
      make(-0.5F, -0.5F, 0.5F),
      make(1.0F, -1.0F, 1.0F),  // voxel 5
      make(0.5F, -0.5F, 0.5F),
      make(-1.0F, 1.0F, 1.0F),  // voxel 6
      make(-0.5F, 0.5F, 0.5F),
      make(1.0F, 1.0F, 1.0F),  // voxel 7
      make(0.5F, 0.5F, 0.5F)
    };
  }
};
////////////////////////////////////////////////////////////////////////////////
class CloudAlgorithm : public VoxelAlgorithm
{
protected:
  using VoxelAlgorithm::make;
  void make(sensor_msgs::msg::PointCloud2 & cloud, std::size_t N)
  {
    point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> mod{cloud, "frame_id"};
    for (std::size_t idx = 0U; idx < N; ++idx) {
      mod.push_back(obs_points1[idx]);
    }
  }
  bool8_t check(const sensor_msgs::msg::PointCloud2 & cloud, std::size_t N)
  {
    point_cloud_msg_wrapper::PointCloud2View<PointXYZI> cloud_view{cloud};
    bool8_t ret = true;
    constexpr float32_t TOL = 1.0E-6F;
    for (const auto & msg_point_ref : cloud_view) {
      bool8_t found = false;
      for (std::size_t jdx = 0U; jdx < N; ++jdx) {
        const auto & ref = ref_points1[jdx];
        if ((fabsf(msg_point_ref.x - ref.x) < TOL) &&
          (fabsf(msg_point_ref.y - ref.y) < TOL) &&
          (fabsf(msg_point_ref.z - ref.z) < TOL))
        {
          found = true;
          break;
        }
      }
      if (!found) {
        ret = false;
        break;
      }
    }
    return ret;
  }
  sensor_msgs::msg::PointCloud2 cloud1;
  sensor_msgs::msg::PointCloud2 cloud2;
  std::unique_ptr<VoxelCloudBase> alg_ptr;

public:
  CloudAlgorithm()
  {
    // Make clouds
    make(cloud1, 8U);
    make(cloud2, obs_points1.size());
  }
};

TEST_F(CloudAlgorithm, Approximate)
{
  this->ref_points1[0U] = this->make(-0.5F, -0.5F, -0.5F);
  this->ref_points1[1U] = this->make(0.5F, -0.5F, -0.5F);
  this->ref_points1[2U] = this->make(-0.5F, 0.5F, -0.5F);
  this->ref_points1[3U] = this->make(0.5F, 0.5F, -0.5F);
  this->ref_points1[4U] = this->make(-0.5F, -0.5F, 0.5F);
  this->ref_points1[5U] = this->make(0.5F, -0.5F, 0.5F);
  this->ref_points1[6U] = this->make(-0.5F, 0.5F, 0.5F);
  this->ref_points1[7U] = this->make(0.5F, 0.5F, 0.5F);
  // initialize
  alg_ptr = std::make_unique<VoxelCloudApproximate>(*cfg_ptr);
  // check initial
  EXPECT_EQ(alg_ptr->get().width, 0U);
  // add points
  alg_ptr->insert(cloud1);
  // get
  EXPECT_TRUE(check(alg_ptr->get(), 4U));
  // check empty
  EXPECT_EQ(alg_ptr->get().width, 0U);
  // add more points
  alg_ptr->insert(cloud1);
  alg_ptr->insert(cloud2);
  // get again
  EXPECT_TRUE(check(alg_ptr->get(), ref_points1.size()));
  // check empty
  EXPECT_EQ(alg_ptr->get().width, 0U);
}

TEST_F(CloudAlgorithm, Centroid)
{
  this->ref_points1[0U] = this->make(-0.75F, -0.75F, -0.75F);
  this->ref_points1[1U] = this->make(0.75F, -0.75F, -0.75F);
  this->ref_points1[2U] = this->make(-0.75F, 0.75F, -0.75F);
  this->ref_points1[3U] = this->make(0.75F, 0.75F, -0.75F);
  this->ref_points1[4U] = this->make(-0.75F, -0.75F, 0.75F);
  this->ref_points1[5U] = this->make(0.75F, -0.75F, 0.75F);
  this->ref_points1[6U] = this->make(-0.75F, 0.75F, 0.75F);
  this->ref_points1[7U] = this->make(0.75F, 0.75F, 0.75F);
  // initialize
  alg_ptr = std::make_unique<VoxelCloudCentroid>(*cfg_ptr);
  // check empty
  EXPECT_EQ(alg_ptr->get().width, 0U);
  // add points
  alg_ptr->insert(cloud1);
  // get
  EXPECT_TRUE(check(alg_ptr->get(), 4U));
  // check empty
  EXPECT_EQ(alg_ptr->get().width, 0U);
  // add more points
  alg_ptr->insert(cloud1);
  alg_ptr->insert(cloud2);
  // get again
  EXPECT_TRUE(check(alg_ptr->get(), ref_points1.size()));
  // check empty
  EXPECT_EQ(alg_ptr->get().width, 0U);
}

TEST(VoxelGridNodes, Instantiate)
{
  // Basic test to ensure that VoxelCloudNode can be instantiated
  rclcpp::init(0, nullptr);

  rclcpp::NodeOptions node_options;

  std::vector<rclcpp::Parameter> params;

  params.emplace_back("subscription.qos.durability", "transient_local");
  params.emplace_back("subscription.qos.history_depth", 2);
  params.emplace_back("publisher.qos.durability", "transient_local");
  params.emplace_back("publisher.qos.history_depth", 4);

  params.emplace_back("is_approximate", false);
  node_options.parameter_overrides(params);
  ASSERT_THROW(VoxelCloudNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("config.capacity", 55000);
  node_options.parameter_overrides(params);
  ASSERT_THROW(VoxelCloudNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("config.min_point.x", -130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(VoxelCloudNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("config.min_point.y", -130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(VoxelCloudNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("config.min_point.z", -3.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(VoxelCloudNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("config.max_point.x", 130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(VoxelCloudNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("config.max_point.y", 130.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(VoxelCloudNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("config.max_point.z", 3.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(VoxelCloudNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("config.voxel_size.x", 1.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(VoxelCloudNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("config.voxel_size.y", 1.0);
  node_options.parameter_overrides(params);
  ASSERT_THROW(VoxelCloudNode{node_options}, rclcpp::ParameterTypeException);

  params.emplace_back("config.voxel_size.z", 1.0);
  node_options.parameter_overrides(params);
  ASSERT_NO_THROW(VoxelCloudNode{node_options});
}
