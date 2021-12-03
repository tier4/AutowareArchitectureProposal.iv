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

#include <ndt/ndt_map.hpp>
#include <ndt/utils.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <algorithm>
#include <string>

namespace autoware
{
namespace localization
{
namespace ndt
{
DynamicNDTMap::DynamicNDTMap(const Config & voxel_grid_config)
: m_grid{voxel_grid_config} {}

const std::string & DynamicNDTMap::frame_id() const noexcept
{
  return m_frame_id;
}

DynamicNDTMap::TimePoint DynamicNDTMap::stamp() const noexcept
{
  return m_stamp;
}

bool DynamicNDTMap::valid() const noexcept
{
  return (m_grid.size() > 0U) && (!m_frame_id.empty());
}

const DynamicNDTMap::ConfigPoint & DynamicNDTMap::cell_size() const noexcept
{
  return m_grid.cell_size();
}

void DynamicNDTMap::set(const sensor_msgs::msg::PointCloud2 & msg)
{
  m_grid.clear();
  insert(msg);
}

void DynamicNDTMap::insert(const sensor_msgs::msg::PointCloud2 & msg)
{
  using PointXYZI = autoware::common::types::PointXYZI;
  point_cloud_msg_wrapper::PointCloud2View<PointXYZI> msg_view{msg};

  for (const auto & point : msg_view) {
    m_grid.add_observation({point.x, point.y, point.z});
  }

  // try to stabilizie the covariance after inserting all the points
  for (auto & vx_it : m_grid) {
    auto & vx = vx_it.second;
    (void) vx.try_stabilize();
  }
  m_stamp = ::time_utils::from_message(msg.header.stamp);
  m_frame_id = msg.header.frame_id;
}

/// The resulting point cloud has the following fields: x, y, z, cov_xx, cov_xy, cov_xz, cov_yy,
/// cov_yz, cov_zz, cell_id.
/// \param msg_out Reference to the pointcloud message that will store
/// the serialized map data. The message will be initialized before use.
template<>
void DynamicNDTMap::serialize_as<StaticNDTMap>(sensor_msgs::msg::PointCloud2 & msg_out) const
{
  ndt::NdtMapCloudModifier msg_modifier{msg_out, frame_id()};

  msg_out.header.stamp = time_utils::to_message(m_stamp);

  const auto min_point = m_grid.config().get_min_point();
  const auto max_point = m_grid.config().get_max_point();
  const auto size = m_grid.config().get_voxel_size();

  // Serialize the configuration to be reconstructed. First 3 fields are used to store the
  // configuration and the remaining 6 fields are left unused so they are set to 0.0
  msg_modifier.push_back(
    PointWithCovariances{min_point.x, min_point.y, min_point.z,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  msg_modifier.push_back(
    PointWithCovariances{max_point.x, max_point.y, max_point.z,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  msg_modifier.push_back(
    PointWithCovariances{size.x, size.y, size.z,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  auto num_used_cells = 0U;
  for (const auto & vx_it : m_grid) {
    const auto & vx = vx_it.second;
    if (!vx.usable()) {
      // Voxel doesn't have enough points to be used in NDT
      continue;
    }

    const auto inv_covariance_opt = vx.inverse_covariance();
    if (!inv_covariance_opt) {
      // Voxel covariance is not invertible
      continue;
    }

    const auto & centroid = vx.centroid();
    const auto & inv_covariance = inv_covariance_opt.value();
    msg_modifier.push_back(
          {
            centroid(0U), centroid(1U), centroid(2U),
            inv_covariance(0U, 0U), inv_covariance(0U, 1U), inv_covariance(0U, 2U),
            inv_covariance(1U, 1U), inv_covariance(1U, 2U),
            inv_covariance(2U, 2U)
          });

    ++num_used_cells;
  }
}

const DynamicNDTMap::VoxelViewVector & DynamicNDTMap::cell(const Point & pt) const
{
  return m_grid.cell(pt);
}

const DynamicNDTMap::VoxelViewVector & DynamicNDTMap::cell(float32_t x, float32_t y, float32_t z)
const
{
  return cell(Point({x, y, z}));
}

std::size_t DynamicNDTMap::size() const noexcept
{
  return m_grid.size();
}

typename DynamicNDTMap::VoxelGrid::const_iterator DynamicNDTMap::begin() const noexcept
{
  return m_grid.cbegin();
}

typename DynamicNDTMap::VoxelGrid::const_iterator DynamicNDTMap::end() const noexcept
{
  return m_grid.cend();
}

void DynamicNDTMap::clear() noexcept
{
  m_grid.clear();
}

const std::string & StaticNDTMap::frame_id() const noexcept
{
  return m_frame_id;
}

StaticNDTMap::TimePoint StaticNDTMap::stamp() const noexcept
{
  return m_stamp;
}

bool StaticNDTMap::valid() const noexcept
{
  return m_grid && (m_grid->size() > 0U) && (!m_frame_id.empty());
}

const StaticNDTMap::ConfigPoint & StaticNDTMap::cell_size() const
{
  if (!m_grid) {
    throw std::runtime_error("Static ndt map was attempted to be used before a map was set.");
  }
  return m_grid->cell_size();
}

void StaticNDTMap::set(const sensor_msgs::msg::PointCloud2 & msg)
{
  if (m_grid) {
    m_grid->clear();
  }
  deserialize_from(msg);
  m_stamp = ::time_utils::from_message(msg.header.stamp);
  m_frame_id = msg.header.frame_id;
}

void StaticNDTMap::deserialize_from(const sensor_msgs::msg::PointCloud2 & msg)
{
  using PointXYZ = geometry_msgs::msg::Point32;
  constexpr auto num_config_fields = 3U;
  NdtMapCloudView msg_view{msg};
  if (msg_view.size() < num_config_fields) {
    throw std::runtime_error("StaticNDTMap: Point cloud representing the ndt map is empty.");
  }

  const auto map_size = msg_view.size() - num_config_fields;
  const auto & min_point = msg_view[0U];
  const auto & max_point = msg_view[1U];
  const auto & voxel_size = msg_view[2U];

  const Config config{
    PointXYZ{}.set__x(static_cast<float>(min_point.x)).set__y(static_cast<float>(min_point.y)).
    set__z(static_cast<float>(min_point.z)),
    PointXYZ{}.set__x(static_cast<float>(max_point.x)).set__y(static_cast<float>(max_point.y)).
    set__z(static_cast<float>(max_point.z)),
    PointXYZ{}.set__x(static_cast<float>(voxel_size.x)).set__y(static_cast<float>(voxel_size.y)).
    set__z(static_cast<float>(voxel_size.z)),
    map_size};

  // Either update the map config or initialize the map.
  if (m_grid) {
    m_grid->set_config(config);
  } else {
    m_grid.emplace(config);
  }

  for (auto it = std::next(msg_view.begin(), num_config_fields); it != msg_view.end(); ++it) {
    const auto & voxel_point = *it;
    const Point centroid{voxel_point.x, voxel_point.y, voxel_point.z};
    const auto voxel_idx = m_grid->index(centroid);

    Eigen::Matrix3d inv_covariance;
    inv_covariance <<
      voxel_point.icov_xx, voxel_point.icov_xy, voxel_point.icov_xz,
      voxel_point.icov_xy, voxel_point.icov_yy, voxel_point.icov_yz,
      voxel_point.icov_xz, voxel_point.icov_yz, voxel_point.icov_zz;
    const Voxel vx{centroid, inv_covariance};

    const auto insert_res = m_grid->emplace_voxel(voxel_idx, Voxel{centroid, inv_covariance});
    if (!insert_res.second) {
      // if a voxel already exist at this point, replace.
      insert_res.first->second = vx;
    }
  }
}
const StaticNDTMap::VoxelViewVector & StaticNDTMap::cell(const Point & pt) const
{
  if (!m_grid) {
    throw std::runtime_error("Static ndt map was attempted to be used before a map was set.");
  }
  return m_grid->cell(pt);
}

const StaticNDTMap::VoxelViewVector & StaticNDTMap::cell(float32_t x, float32_t y, float32_t z)
const
{
  return cell(Point({x, y, z}));
}

std::size_t StaticNDTMap::size() const
{
  if (!m_grid) {
    throw std::runtime_error("Static ndt map was attempted to be used before a map was set.");
  }
  return m_grid->size();
}

typename StaticNDTMap::VoxelGrid::const_iterator StaticNDTMap::begin() const
{
  if (!m_grid) {
    throw std::runtime_error("Static ndt map was attempted to be used before a map was set.");
  }
  return m_grid->cbegin();
}

typename StaticNDTMap::VoxelGrid::const_iterator StaticNDTMap::end() const
{
  if (!m_grid) {
    throw std::runtime_error("Static ndt map was attempted to be used before a map was set.");
  }
  return m_grid->cend();
}

void StaticNDTMap::clear()
{
  if (!m_grid) {
    throw std::runtime_error("Static ndt map was attempted to be used before a map was set.");
  }
  m_grid->clear();
}
}  // namespace ndt
}  // namespace localization
}  // namespace autoware
