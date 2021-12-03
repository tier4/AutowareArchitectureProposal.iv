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

#include <cstring>

#include "lidar_utils/point_cloud_utils.hpp"
#include "point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp"
#include "voxel_grid_nodes/algorithm/voxel_cloud_centroid.hpp"

using autoware::common::lidar_utils::has_intensity_and_throw_if_no_xyz;

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
////////////////////////////////////////////////////////////////////////////////
VoxelCloudCentroid::VoxelCloudCentroid(const voxel_grid::Config & cfg)
: VoxelCloudBase(),
  m_cloud(),
  m_grid(cfg)
{
  // frame id is arbitrary, not the responsibility of this component
  using autoware::common::types::PointXYZI;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI>{m_cloud, "base_link"};
}

////////////////////////////////////////////////////////////////////////////////
void VoxelCloudCentroid::insert(
  const sensor_msgs::msg::PointCloud2 & msg)
{
  m_cloud.header = msg.header;

  // Verify the consistency of PointCloud msg
  const auto data_length = msg.width * msg.height * msg.point_step;
  if ((msg.data.size() != msg.row_step) || (data_length != msg.row_step)) {
    throw std::runtime_error("VoxelCloudCentroid: Malformed PointCloud2");
  }
  // Verify the point cloud format and assign correct point_step
  constexpr auto field_size = sizeof(decltype(autoware::common::types::PointXYZIF::x));
  auto point_step = 4U * field_size;
  if (!has_intensity_and_throw_if_no_xyz(msg)) {
    point_step = 3U * field_size;
  }

  // Iterate through the data, but skip intensity in case the point cloud does not have it.
  // For example:
  //
  // point_step = 4
  // x y z i a b c x y z i a b c
  // ^------       ^------
  for (std::size_t idx = 0U; idx < msg.data.size(); idx += msg.point_step) {
    PointXYZIF pt;
    //lint -e{925, 9110} Need to convert pointers and use bit for external API NOLINT
    (void)memmove(
      static_cast<void *>(&pt.x),
      static_cast<const void *>(&msg.data[idx]),
      point_step);
    m_grid.insert(pt);
  }
  // TODO(c.ho) overlay?
}

////////////////////////////////////////////////////////////////////////////////
const sensor_msgs::msg::PointCloud2 & VoxelCloudCentroid::get()
{
  using autoware::common::types::PointXYZI;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> modifier{m_cloud};
  modifier.clear();
  modifier.reserve(m_grid.size());

  for (const auto & it : m_grid) {
    const auto & pt = it.second.get();
    modifier.push_back(PointXYZI{pt.x, pt.y, pt.z, pt.intensity});
  }
  m_grid.clear();

  return m_cloud;
}
}  // namespace algorithm
}  // namespace voxel_grid_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware
