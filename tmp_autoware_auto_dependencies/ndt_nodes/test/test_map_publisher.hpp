// Copyright 2017-2019 the Autoware Foundation
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

#ifndef TEST_MAP_PUBLISHER_HPP_
#define TEST_MAP_PUBLISHER_HPP_

#include <ndt/ndt_map.hpp>
#include <pcl/io/pcd_io.h>
#include <gtest/gtest.h>
#include <geometry/spatial_hash_config.hpp>
#include <voxel_grid/config.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <Eigen/Core>
#include <vector>
#include <set>
#include <map>
#include "common/types.hpp"

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::PointXYZIF;
using autoware::common::types::PointXYZI;

PointXYZI get_point_from_vector(const Eigen::Vector3d & v)
{
  return PointXYZI{
    static_cast<float32_t>(v(0)),
    static_cast<float32_t>(v(1)),
    static_cast<float32_t>(v(2)),
    0.0};
}

class DenseNDTMapContext
{
protected:
  static constexpr int POINTS_PER_DIM{5U};
  // how much should the points diverge from the center. It's fixed as there's no randomness.
  static constexpr float32_t FIXED_DEVIATION{0.3f};
  using PointXYZ = geometry_msgs::msg::Point32;

// add the point `center` and 4 additional points in a fixed distance from the center
// resulting in 7 points with random but bounded covariance
  void add_cell(
    point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> & msg_wrapper,
    const Eigen::Vector3d & center, float64_t fixed_deviation)
  {
    msg_wrapper.push_back(get_point_from_vector(center));

    std::vector<Eigen::Vector3d> points;
    for (auto idx = 0U; idx < 3U; idx++) {
      for (auto mode = 0u; mode < 2u; mode++) {
        auto deviated_pt = center;
        if (mode == 0U) {
          deviated_pt(idx) += fixed_deviation;
        } else {
          deviated_pt(idx) -= fixed_deviation;
        }
        points.push_back(deviated_pt);
        msg_wrapper.push_back(get_point_from_vector(deviated_pt));
      }
    }
  }

  DenseNDTMapContext()
  : m_pc_wrapper{m_pc, "map"}
  {
    // Grid and spatial hash uses these boundaries. The setup allows for a grid of 125 cells: 5x5x5
    // where the centroid coordinates range from the integers 1 to 5 and the voxel size is 1
    m_min_point.x = 0.5F;
    m_min_point.y = 0.5F;
    m_min_point.z = 0.5F;
    m_max_point.x = 5.5F;
    m_max_point.y = 5.5F;
    m_max_point.z = 5.5F;
    m_voxel_size.x = 1.0F;
    m_voxel_size.y = 1.0F;
    m_voxel_size.z = 1.0F;
  }

  void build_pc(const autoware::perception::filters::voxel_grid::Config & cfg)
  {
    for (auto x = 1U; x <= POINTS_PER_DIM; ++x) {
      for (auto y = 1U; y <= POINTS_PER_DIM; ++y) {
        for (auto z = 1U; z <= POINTS_PER_DIM; ++z) {
          Eigen::Vector3d center{static_cast<float64_t>(x), static_cast<float64_t>(y),
            static_cast<float64_t>(z)};
          add_cell(m_pc_wrapper, center, static_cast<float64_t>(FIXED_DEVIATION));
          m_voxel_centers[cfg.index(center)] = center;
        }
      }
    }
  }

  uint32_t m_pc_idx{0U};
  sensor_msgs::msg::PointCloud2 m_pc;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> m_pc_wrapper;
  std::map<uint64_t, Eigen::Vector3d> m_voxel_centers;
  PointXYZ m_min_point;
  PointXYZ m_max_point;
  PointXYZ m_voxel_size;
  uint64_t m_capacity{1024U};
};

class MapPublisherTest : public DenseNDTMapContext, public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown()
  {
    rclcpp::shutdown();
  }
};


pcl::PointCloud<pcl::PointXYZI> from_pointcloud2(const sensor_msgs::msg::PointCloud2 & msg)
{
  pcl::PointCloud<pcl::PointXYZI> res{};
  point_cloud_msg_wrapper::PointCloud2View<autoware::common::types::PointXYZI>
  msg_view{msg};

  for (const auto & pt_in : msg_view) {
    pcl::PointXYZI pt;
    pt.x = pt_in.x;
    pt.y = pt_in.y;
    pt.z = pt_in.z;
    res.push_back(pt);
  }
  return res;
}

#endif  // TEST_MAP_PUBLISHER_HPP_
