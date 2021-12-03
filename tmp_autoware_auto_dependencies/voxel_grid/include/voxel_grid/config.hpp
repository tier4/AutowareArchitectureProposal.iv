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
/// \brief This file defines the configuration class for the voxel grid data structure

#ifndef VOXEL_GRID__CONFIG_HPP_
#define VOXEL_GRID__CONFIG_HPP_

#include <voxel_grid/visibility_control.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry/common_2d.hpp>
#include <common/types.hpp>
#include <cmath>

using autoware::common::types::float32_t;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace voxel_grid
{
// TODO(cvasfi) move this definition to a shared/common place

template<typename T>
inline T clamp(const T val, const T min, const T max)
{
  return (val < min) ? min : ((val > max) ? max : val);
}

using PointXYZ = geometry_msgs::msg::Point32;


/// \brief A configuration class for the VoxelGrid data structure, also includes some helper
///        functionality for computing indices and centroids of voxels.
class VOXEL_GRID_PUBLIC Config
{
public:
  static constexpr float32_t MIN_VOXEL_SIZE_M = 0.01F;
  /// \brief Constructor
  /// \param[in] min_point The minimum corner of the receptive field (box) for the voxel grid
  /// \param[in] max_point The maximum corner of the receptive field (box) for the voxel grid
  /// \param[in] voxel_size The size of each voxel
  /// \param[in] capacity The preallocated size of the voxel grid
  Config(
    const PointXYZ & min_point,
    const PointXYZ & max_point,
    const PointXYZ & voxel_size,
    const uint64_t capacity);
  /// \brief Gets the minimum corner of the voxel grid
  /// \return Fixed value
  const PointXYZ & get_min_point() const;
  /// \brief Gets the maximum corner of the voxel grid
  /// \return Fixed value
  const PointXYZ & get_max_point() const;
  /// \brief Gets the voxel size of the voxel grid
  /// \return Fixed value
  const PointXYZ & get_voxel_size() const;
  /// \brief Gets the capacity of the voxel grid
  /// \return Fixed value
  uint64_t get_capacity() const;
  /// \brief Computes index for a given point given the voxelgrid configuration parameters
  /// \param[in] pt The point for which the voxel index will be computed
  /// \return The index of the voxel for which this point will fall into
  /// \tparam PointT The point type taken. Assumed to have public float32_t members x, y, and z.
  ///                Also assumed to have a default constructor.
  template<typename PointT>
  uint64_t index(const PointT & pt) const
  {
    const uint64_t idx = static_cast<uint64_t>(
      std::floor(
        (clamp(
          common::geometry::point_adapter::x_(pt),
          m_min_point.x, m_max_point.x) - m_min_point.x) * m_voxel_size_inv.x));
    const uint64_t jdx = static_cast<uint64_t>(
      std::floor(
        (clamp(
          common::geometry::point_adapter::y_(pt),
          m_min_point.y, m_max_point.y) - m_min_point.y) * m_voxel_size_inv.y));
    const uint64_t kdx = static_cast<uint64_t>(
      std::floor(
        (clamp(
          common::geometry::point_adapter::z_(pt),
          m_min_point.z, m_max_point.z) - m_min_point.z) * m_voxel_size_inv.z));
    return idx + (jdx * m_y_stride) + (kdx * m_z_stride);
  }

  /// \brief Computes the centroid for a given voxel index
  /// \param[in] index The index for a given voxel
  /// \return A point for whom the x, y and z fields are filled out
  /// \tparam PointT The point type returned. Assumed to have public float32_t members x, y, and z.
  ///                Also assumed to have a default constructor.
  ///                Only the x, y, and z fields will be filled out.
  template<typename PointT>
  PointT centroid(const uint64_t index) const
  {
    // 'deserialize' indices
    const uint64_t zdx = index / m_z_stride;
    const uint64_t jdx = (index % m_z_stride);
    const uint64_t ydx = jdx / m_y_stride;
    const uint64_t xdx = jdx % m_y_stride;
    // compute centroid of voxel
    PointT pt;

    common::geometry::point_adapter::xr_(pt) =
      ((static_cast<float32_t>(xdx) + 0.5F) * m_voxel_size.x) + m_min_point.x;
    common::geometry::point_adapter::yr_(pt) =
      ((static_cast<float32_t>(ydx) + 0.5F) * m_voxel_size.y) + m_min_point.y;
    common::geometry::point_adapter::zr_(pt) =
      ((static_cast<float32_t>(zdx) + 0.5F) * m_voxel_size.z) + m_min_point.z;
    return pt;
  }

private:
  /// \brief Sanity check a range in a basis direction
  /// \return The number of voxels in the given basis direction (aka width in units of voxels)
  /// \param[in] min The lower bound in the specified basis direction
  /// \param[in] max The upper bound in the specified basis direction
  /// \param[in] size The voxel size in the specified basis direction
  uint64_t check_basis_direction(
    const float32_t min,
    const float32_t max,
    const float32_t size) const;

  PointXYZ m_min_point;
  PointXYZ m_max_point;
  PointXYZ m_voxel_size;
  PointXYZ m_voxel_size_inv;
  uint64_t m_y_stride;
  uint64_t m_z_stride;
  uint64_t m_capacity;
};  // class Config
}  // namespace voxel_grid
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // VOXEL_GRID__CONFIG_HPP_
