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

#include <common/types.hpp>
#include <limits>
#include "voxel_grid/voxel_grid.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace voxel_grid
{
////////////////////////////////////////////////////////////////////////////////
uint64_t Config::check_basis_direction(
  const float32_t min,
  const float32_t max,
  const float32_t size) const
{
  // check leaf size against lb to prevent division by zero
  //lint -e{1938} read only access is ok NOLINT
  if (MIN_VOXEL_SIZE_M > size) {
    throw std::domain_error("voxel_grid::Config leaf size smaller than MIN_VOXEL_SIZE_M");
  }
  if (min >= max) {
    throw std::domain_error("voxel_grid::Config: must have min < max");
  }
  // This family of checks is to ensure that you don't get weird casting effects due to huge
  // floating point values
  const float64_t dmax = static_cast<float64_t>(max);
  const float64_t dmin = static_cast<float64_t>(min);
  const float64_t width = (dmax - dmin) / static_cast<float64_t>(size);
  // This check is to ensure that you don't get weird casting effects due to huge
  // floating point values
  constexpr float64_t fltmax = static_cast<float64_t>(std::numeric_limits<float32_t>::max());
  if (fltmax <= width) {
    throw std::domain_error("voxel_grid::Config: voxel size approaching floating point limit");
  }
  return static_cast<uint64_t>(width);
}
////////////////////////////////////////////////////////////////////////////////
Config::Config(
  const PointXYZ & min_point,
  const PointXYZ & max_point,
  const PointXYZ & voxel_size,
  const uint64_t capacity)
: m_min_point(min_point),
  m_max_point(max_point),
  m_voxel_size(voxel_size),
  m_y_stride{check_basis_direction(min_point.x, max_point.x, voxel_size.x)},
  m_capacity(capacity)
{
  // tiny function to check if the multiplication of unsigned longs will overflow
  auto mul_will_overflow_u64 = [](const uint64_t x, const uint64_t y)
    {
      bool8_t retval = false;
      if ((x != 0U) && (y != 0U)) {
        retval = x > (UINT64_MAX / y);
      }
      return retval;
    };

  // precomputation for computational convenience
  m_voxel_size_inv.x = 1.0F / m_voxel_size.x;
  m_voxel_size_inv.y = 1.0F / m_voxel_size.y;
  m_voxel_size_inv.z = 1.0F / m_voxel_size.z;
  // More precomputation, check for uint overflow
  const uint64_t y_width = check_basis_direction(min_point.y, max_point.y, voxel_size.y);
  // This family of checks is to make sure that each voxel's index will be unique
  if (mul_will_overflow_u64(y_width, m_y_stride)) {
    throw std::domain_error("voxel_grid::Config: voxel index may overflow!");
  }
  m_z_stride = y_width * m_y_stride;
  const uint64_t z_width = check_basis_direction(min_point.z, max_point.z, voxel_size.z);
  if (mul_will_overflow_u64(m_z_stride, z_width)) {
    throw std::domain_error("voxel_grid::Config: voxel index may overflow!");
  }
  // small fudging to prevent weird boundary effects
  // (e.g (x=xmax, y) rolls index over to (x=0, y+1)
  //lint -e{1938} read only access is fine NOLINT
  m_max_point.x -= std::numeric_limits<float32_t>::epsilon();
  m_max_point.y -= std::numeric_limits<float32_t>::epsilon();
  m_max_point.z -= std::numeric_limits<float32_t>::epsilon();
}
////////////////////////////////////////////////////////////////////////////////
const PointXYZ & Config::get_min_point() const
{
  return m_min_point;
}
////////////////////////////////////////////////////////////////////////////////
const PointXYZ & Config::get_max_point() const
{
  return m_max_point;
}
////////////////////////////////////////////////////////////////////////////////
const PointXYZ & Config::get_voxel_size() const
{
  return m_voxel_size;
}
////////////////////////////////////////////////////////////////////////////////
uint64_t Config::get_capacity() const
{
  return m_capacity;
}

}  // namespace voxel_grid
}  // namespace filters
}  // namespace perception
}  // namespace autoware
