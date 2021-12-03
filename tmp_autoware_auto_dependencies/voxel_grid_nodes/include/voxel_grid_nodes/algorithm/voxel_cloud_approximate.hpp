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

/// \file
/// \brief This file defines an instance of the VoxelCloudBase interface
#ifndef VOXEL_GRID_NODES__ALGORITHM__VOXEL_CLOUD_APPROXIMATE_HPP_
#define VOXEL_GRID_NODES__ALGORITHM__VOXEL_CLOUD_APPROXIMATE_HPP_

#include <voxel_grid_nodes/algorithm/voxel_cloud_base.hpp>

namespace autoware
{
namespace perception
{
namespace filters
{
namespace voxel_grid_nodes
{
namespace algorithm
{
/// \brief Instantiation of PointCloud2 VoxelCloudBase for ApproximateVoxels.
class VOXEL_GRID_NODES_PUBLIC VoxelCloudApproximate : public VoxelCloudBase
{
public:
  /// \brief Constructor
  /// \param[in] cfg Configuration struct for the voxel grid
  explicit VoxelCloudApproximate(const voxel_grid::Config & cfg);

  /// \brief Inserts points into the voxel grid data structure, overwrites internal header
  /// \param[in] msg A point cloud to insert into the voxel grid. Assumed to have the structure XYZI
  void insert(const sensor_msgs::msg::PointCloud2 & msg) override;

  /// \brief Get accumulated downsampled points. Internally resets the internal grid. Header is
  ///        taken from last insert
  /// \return The downsampled point cloud
  const sensor_msgs::msg::PointCloud2 & get() override;

private:
  sensor_msgs::msg::PointCloud2 m_cloud;
  voxel_grid::VoxelGrid<voxel_grid::ApproximateVoxel<PointXYZIF>> m_grid;
};  // VoxelCloudApproximate
}  // namespace algorithm
}  // namespace voxel_grid_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // VOXEL_GRID_NODES__ALGORITHM__VOXEL_CLOUD_APPROXIMATE_HPP_
