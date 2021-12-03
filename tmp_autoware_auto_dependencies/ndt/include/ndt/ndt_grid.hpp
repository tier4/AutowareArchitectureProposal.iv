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

#ifndef NDT__NDT_GRID_HPP_
#define NDT__NDT_GRID_HPP_

#include <common/types.hpp>
#include <ndt/ndt_common.hpp>
#include <ndt/ndt_voxel_view.hpp>
#include <vector>
#include <limits>
#include <unordered_map>
#include <utility>
#include <string>

using autoware::common::types::float32_t;

namespace autoware
{
namespace localization
{
namespace ndt
{
/// \brief A voxel grid implementation for normal distribution transform
/// \tparam VoxelT Voxel type
template<typename VoxelT>
class NDTGrid
{
public:
  using Grid = std::unordered_map<uint64_t, VoxelT>;
  using Point = Eigen::Vector3d;
  using Config = autoware::perception::filters::voxel_grid::Config;
  using VoxelViewVector = std::vector<VoxelView<VoxelT>>;
  using ConfigPoint = std::decay_t<decltype(std::declval<Config>().get_min_point())>;

  /// Constructor
  /// \param voxel_grid_config Voxel grid config to configure the underlying voxel grid.
  explicit NDTGrid(const Config & voxel_grid_config)
  : m_config(voxel_grid_config), m_map(m_config.get_capacity())
  {
    m_output_vector.reserve(1U);
  }

  // Maps should be moved rather than being copied.
  NDTGrid(const NDTGrid &) = delete;

  NDTGrid & operator=(const NDTGrid &) = delete;

  // Explicitly declaring to default is needed since we explicitly deleted the copy methods.
  NDTGrid(NDTGrid &&) = default;

  NDTGrid & operator=(NDTGrid &&) = default;

  /// Lookup the cell at location.
  /// \param x x coordinate
  /// \param y y coordinate
  /// \param z z coordinate
  /// \return A vector containing the cell at given coordinates. A vector is used to support
  /// near-neighbour cell queries in the future.
  const VoxelViewVector & cell(float32_t x, float32_t y, float32_t z) const
  {
    return cell(Point({x, y, z}));
  }

  /// Lookup the cell at location.
  /// \param pt point to lookup
  /// \return A vector containing the cell at given coordinates. A vector is used to support
  /// near-neighbour cell queries in the future.
  const VoxelViewVector & cell(const Point & pt) const
  {
    // TODO(yunus.caliskan): revisit after multi-cell lookup support. #985
    m_output_vector.clear();
    const auto vx_it = m_map.find(m_config.index(pt));
    // Only return a voxel if it's occupied (i.e. has enough points to compute covariance.)
    if (vx_it != m_map.end() && vx_it->second.usable()) {
      m_output_vector.emplace_back(vx_it->second);
    }
    return m_output_vector;
  }

  /// Get size of the map
  /// \return Number of voxels in the map. This number includes the voxels that do not have
  /// enough numbers to be used yet.
  std::size_t size() const noexcept
  {
    return m_map.size();
  }

  /// Get size of the cell.
  /// \return A point representing the dimensions of the cell.
  const ConfigPoint & cell_size() const noexcept
  {
    return m_config.get_voxel_size();
  }

  /// \brief Returns an const iterator to the first element of the map
  /// \return Iterator
  typename Grid::const_iterator begin() const noexcept
  {
    return cbegin();
  }

  /// \brief Returns an iterator to the first element of the map
  /// \return Iterator
  typename Grid::iterator begin() noexcept
  {
    return m_map.begin();
  }

  /// \brief Returns a const iterator to the first element of the map
  /// \return Iterator
  typename Grid::const_iterator cbegin() const noexcept
  {
    return m_map.cbegin();
  }

  /// \brief Returns a const iterator to one past the last element of the map
  /// \return Iterator
  typename Grid::const_iterator end() const noexcept
  {
    return cend();
  }

  /// \brief Returns an iterator to one past the last element of the map
  /// \return Iterator
  typename Grid::iterator end() noexcept
  {
    return m_map.end();
  }

  /// \brief Returns a const iterator to one past the last element of the map
  /// \return Iterator
  typename Grid::const_iterator cend() const noexcept
  {
    return m_map.cend();
  }

  /// Clear all voxels in the map
  void clear() noexcept
  {
    m_map.clear();
  }

  /// Get voxel index given a point.
  /// \param pt point
  /// \return voxel index
  auto index(const Point & pt) const
  {
    return m_config.index(pt);
  }

  /// \brief Emplace a new voxel into the grid.
  /// \param args Arguments. An index and a voxel is expected
  /// \return See the return type of `unordered_map::emplace()`.
  template<typename ... Args>
  auto emplace_voxel(Args && ... args)
  {
    return m_map.emplace(std::forward<Args>(args)...);
  }

  /// \brief Add a point to its corresponding voxel in the grid.
  /// \param pt Point to be added
  void add_observation(const Point & pt)
  {
    m_map[index(pt)].add_observation(pt);
  }

  /// \brief Set the configuration
  /// \param config Config object to be set.
  void set_config(const Config & config)
  {
    m_config = config;
  }

  /// \brief Get the underlying voxel grid configuration
  /// \return Voxel grid configuration
  const Config & config() const
  {
    return m_config;
  }

private:
  mutable VoxelViewVector m_output_vector;
  Config m_config;
  Grid m_map;
};

}  // namespace ndt
}  // namespace localization

namespace common
{
namespace geometry
{
namespace point_adapter
{
/// Point adapters for eigen vector
/// These adapters are necessary for the VoxelGrid to know how to access
/// the coordinates from an eigen vector.
template<>
inline NDT_PUBLIC auto x_(const Eigen::Vector3d & pt)
{
  return static_cast<float32_t>(pt(0));
}

template<>
inline NDT_PUBLIC auto y_(const Eigen::Vector3d & pt)
{
  return static_cast<float32_t>(pt(1));
}

template<>
inline NDT_PUBLIC auto z_(const Eigen::Vector3d & pt)
{
  return static_cast<float32_t>(pt(2));
}

template<>
inline NDT_PUBLIC auto & xr_(Eigen::Vector3d & pt)
{
  return pt(0);
}

template<>
inline NDT_PUBLIC auto & yr_(Eigen::Vector3d & pt)
{
  return pt(1);
}

template<>
inline NDT_PUBLIC auto & zr_(Eigen::Vector3d & pt)
{
  return pt(2);
}

}  // namespace point_adapter
}  // namespace geometry
}  // namespace common
}  // namespace autoware
#endif  // NDT__NDT_GRID_HPP_
