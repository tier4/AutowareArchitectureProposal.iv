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
/// \brief This file defines children and specializations of the Voxel class

#ifndef VOXEL_GRID__VOXELS_HPP_
#define VOXEL_GRID__VOXELS_HPP_

#include "common/types.hpp"
#include "voxel_grid/config.hpp"
#include "voxel_grid/visibility_control.hpp"
#include "voxel_grid/voxel.hpp"

namespace autoware
{
namespace perception
{
namespace filters
{
namespace voxel_grid
{

using autoware::common::types::PointXYZIF;
using autoware::common::types::float32_t;

/// \brief Default addition operator for a point type for use with CentroidVoxel
/// \tparam PointT Point type, assumed to have fields x, y, and z
/// \param[in] lhs Left hand operator
/// \param[in] rhs Right hand operator
/// \return lhs + rhs
template<typename PointT>
VOXEL_GRID_PUBLIC PointT operator+(const PointT & lhs, const PointT & rhs)
{
  PointT ret = lhs;
  ret.x += rhs.x;
  ret.y += rhs.y;
  ret.z += rhs.z;
  return ret;
}

/// \brief Default scalar multiplication operator for a point type for use with CentroidVoxel
/// \tparam PointT Point type, assumed to have fields x, y, and z
/// \param[in] lhs Left hand operator
/// \param[in] rhs Right hand operator, a floating point value
/// \return lhs * rhs
template<typename PointT>
VOXEL_GRID_PUBLIC PointT operator*(const PointT & lhs, const float32_t rhs)
{
  PointT ret = lhs;
  ret.x *= rhs;
  ret.y *= rhs;
  ret.z *= rhs;
  return ret;
}

/// \brief Addition operator for a PointXYZIF for use with CentroidVoxel
/// \param[in] lhs Left hand operator
/// \param[in] rhs Right hand operator
/// \return lhs + rhs
template<>
inline VOXEL_GRID_PUBLIC PointXYZIF operator+(
  //lint -e{9073} NOLINT This is a template specialization, not a token mismatch
  const PointXYZIF & lhs,
  //lint -e{9073} NOLINT This is a template specialization, not a token mismatch
  const PointXYZIF & rhs)
{
  PointXYZIF ret = lhs;
  ret.x += rhs.x;
  ret.y += rhs.y;
  ret.z += rhs.z;
  ret.intensity += rhs.intensity;
  return ret;
}
/// \brief Scalar multiplication operator for a PointXYZIF for use with CentroidVoxel
/// \param[in] lhs Left hand operator
/// \param[in] rhs Right hand operator, a floating point value
/// \return lhs * rhs
template<>
inline VOXEL_GRID_PUBLIC PointXYZIF operator*(
  //lint -e{9073} NOLINT This is a template specialization, not a token mismatch
  const PointXYZIF & lhs,
  const float32_t rhs)
{
  PointXYZIF ret = lhs;
  ret.x *= rhs;
  ret.y *= rhs;
  ret.z *= rhs;
  ret.intensity *= rhs;
  return ret;
}

/// \brief A specialization of the Voxel class, accumulates points to a moving centroid
/// \tparam PointT The point type, must have operator+ and operator*(float32_t) defined, and
///                float32_t members x, y, and z
///
/// A note on extending this to point types with additional fields: The default behavior only
/// updates the x, y, and z fields of the points. To get proper behavior for other types,
/// you can define operator+(PointT, PointT) and operator*(PointT, float32_t), in this namespace
/// or the global namespace to get proper resolution.
template<typename PointT>
class VOXEL_GRID_PUBLIC CentroidVoxel : public Voxel<PointT>
{
public:
  using Voxel<PointT>::Voxel;
  /// \brief Update the state of this voxel by incrementally updating the mean
  /// \param[in] pt The observed point
  void add_observation(const PointT & pt)
  {
    const float32_t last = static_cast<float32_t>(Voxel<PointT>::count());
    Voxel<PointT>::set_count(Voxel<PointT>::count() + 1U);
    const float32_t count_inv = 1.0F / static_cast<float32_t>(Voxel<PointT>::count());
    // Incremental update: u' = ((u * n) + x) / (n + 1), u = mean, x = obs, n = count
    const PointT tmp = ((Voxel<PointT>::get() * last) + pt) * count_inv;
    Voxel<PointT>::set_centroid(tmp);
  }
  /// \brief Use Config and index to set up any important information, in this case no-op
  /// \param[in] cfg The configuration object for the parent voxel grid
  /// \param[in] idx The index for this particular voxel
  //lint -e{9175} NOLINT this is to match a parent API
  void configure(const Config & cfg, const uint64_t idx)
  {
    (void)cfg;
    (void)idx;
  }
};  // class CentroidVoxel

/// \brief A specialization of the Voxel class, only returns centroid of voxel
/// \tparam PointT The point type, must have float32_t members x, y, and z
template<typename PointT>
class VOXEL_GRID_PUBLIC ApproximateVoxel : public Voxel<PointT>
{
public:
  using Voxel<PointT>::Voxel;
  /// \brief Update the state of the voxel, only increments internal counter
  /// \param[in] pt The observed point
  void add_observation(const PointT & pt)
  {
    Voxel<PointT>::set_count(Voxel<PointT>::count() + 1U);
    (void)pt;
  }
  /// \brief Use Config and index to set up any important information, in this case sets the
  ///        centroid
  /// \param[in] cfg The configuration object for the parent voxel grid
  /// \param[in] idx The index for this particular voxel
  void configure(const Config & cfg, const uint64_t idx)
  {
    Voxel<PointT>::set_centroid(cfg.centroid<PointT>(idx));
  }
};  // class ApproximateVoxel


}  // namespace voxel_grid
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // VOXEL_GRID__VOXELS_HPP_
