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

#ifndef TEST_NDT_MAP_HPP_
#define TEST_NDT_MAP_HPP_

#include <ndt/ndt_map.hpp>
#include <geometry/spatial_hash_config.hpp>
#include <voxel_grid/config.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <Eigen/Core>
#include <vector>
#include <set>
#include <map>
#include <string>
#include "common/types.hpp"

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

using autoware::localization::ndt::DynamicNDTMap;
constexpr auto kNumConfigPoints = DynamicNDTMap::kNumConfigPoints;

sensor_msgs::msg::PointCloud2 make_pcl(
  const std::vector<sensor_msgs::msg::PointField> & fields,
  uint32_t height,
  uint32_t data_size,
  uint32_t row_step,
  uint32_t width,
  uint32_t point_step);

sensor_msgs::msg::PointCloud2 make_pcl(const std::vector<Eigen::Vector3d> & pts);


sensor_msgs::msg::PointField make_pf(
  std::string name, uint32_t offset, uint8_t datatype,
  uint32_t count);

void populate_pc(
  sensor_msgs::msg::PointCloud2 & pc,
  size_t num_points);
using PointXYZI = autoware::common::types::PointXYZI;

PointXYZI get_point_from_vector(const Eigen::Vector3d & v);

// add the point `center` and 4 additional points in a fixed distance from the center
// resulting in 7 points with random but bounded covariance
void add_cell(
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> & msg_wrapper,
  const Eigen::Vector3d & center, double fixed_deviation);

using PointXYZ = geometry_msgs::msg::Point32;

class DenseNDTMapContext
{
protected:
  static constexpr int POINTS_PER_DIM{5U};
  // how much should the points diverge from the center. It's fixed as there's no randomness.
  static constexpr float32_t FIXED_DEVIATION{0.3};

  DenseNDTMapContext();

  void build_pc(const autoware::perception::filters::voxel_grid::Config & cfg);

  sensor_msgs::msg::PointCloud2 m_pc;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> m_pc_wrapper;
  std::map<uint64_t, Eigen::Vector3d> m_voxel_centers;
  static constexpr std::uint32_t NUM_POINTS{50U};
  PointXYZ m_min_point;
  PointXYZ m_max_point;
  PointXYZ m_voxel_size;
  uint64_t m_capacity{1024U};
};


class NDTMapContext : protected DenseNDTMapContext {};

#endif  // TEST_NDT_MAP_HPP_
