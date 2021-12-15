// Copyright 2017-2020 the Autoware Foundation, Arm Limited
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
/// \brief This file includes common type definition

// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef COMMON__TYPES_HPP_
#define COMMON__TYPES_HPP_

#include "float_comparisons.hpp"

#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <cstdint>
#include <limits>
#include <vector>

namespace autoware
{
namespace common
{
namespace types
{
// Aliases to conform to MISRA C++ Rule 3-9-2 (Directive 4.6 in MISRA C).
// Similarly, the stdint typedefs should be used instead of plain int, long etc. types.
// We don't currently require code to comply to MISRA, but we should try to where it is
// easily possible.
using bool8_t = bool;
using char8_t = char;
using uchar8_t = unsigned char;
// If we ever compile on a platform where this is not true, float32_t and float64_t definitions
// need to be adjusted.
static_assert(sizeof(float) == 4, "float is assumed to be 32-bit");
using float32_t = float;
static_assert(sizeof(double) == 8, "double is assumed to be 64-bit");
using float64_t = double;

/// pi = tau / 2
constexpr float32_t PI = 3.14159265359F;
/// pi/2
constexpr float32_t PI_2 = 1.5707963267948966F;
/// tau = 2 pi
constexpr float32_t TAU = 6.283185307179586476925286766559F;

struct PointXYZIF
{
  float32_t x{0};
  float32_t y{0};
  float32_t z{0};
  float32_t intensity{0};
  uint16_t id{0};
  static constexpr uint16_t END_OF_SCAN_ID = 65535u;
  friend bool operator==(const PointXYZIF & p1, const PointXYZIF & p2) noexcept
  {
    using autoware::common::helper_functions::comparisons::rel_eq;
    const auto epsilon = std::numeric_limits<float32_t>::epsilon();
    return rel_eq(p1.x, p2.x, epsilon) && rel_eq(p1.y, p2.y, epsilon) &&
           rel_eq(p1.z, p2.z, epsilon) && rel_eq(p1.intensity, p2.intensity, epsilon) &&
           (p1.id == p2.id);
  }
};

struct PointXYZF
{
  float32_t x{0};
  float32_t y{0};
  float32_t z{0};
  uint16_t id{0};
  static constexpr uint16_t END_OF_SCAN_ID = 65535u;
  friend bool operator==(const PointXYZF & p1, const PointXYZF & p2) noexcept
  {
    using autoware::common::helper_functions::comparisons::rel_eq;
    const auto epsilon = std::numeric_limits<float32_t>::epsilon();
    return rel_eq(p1.x, p2.x, epsilon) && rel_eq(p1.y, p2.y, epsilon) &&
           rel_eq(p1.z, p2.z, epsilon) && (p1.id == p2.id);
  }
};

struct PointXYZI
{
  float32_t x{0.0F};
  float32_t y{0.0F};
  float32_t z{0.0F};
  float32_t intensity{0.0F};
  friend bool operator==(const PointXYZI & p1, const PointXYZI & p2) noexcept
  {
    return helper_functions::comparisons::rel_eq(
             p1.x, p2.x, std::numeric_limits<float32_t>::epsilon()) &&

           helper_functions::comparisons::rel_eq(
             p1.y, p2.y, std::numeric_limits<float32_t>::epsilon()) &&

           helper_functions::comparisons::rel_eq(
             p1.z, p2.z, std::numeric_limits<float32_t>::epsilon()) &&

           helper_functions::comparisons::rel_eq(
             p1.intensity, p2.intensity, std::numeric_limits<float32_t>::epsilon());
  }
};

struct PointXYZIRADRT
{
  float32_t x{0.0F};
  float32_t y{0.0F};
  float32_t z{0.0F};
  float32_t intensity{0.0F};
  uint16_t ring{0U};
  float azimuth{0.0F};
  float distance{0.0F};
  uint8_t return_type{0U};
  double time_stamp{0.0};
  friend bool operator==(const PointXYZIRADRT & p1, const PointXYZIRADRT & p2) noexcept
  {
    return helper_functions::comparisons::rel_eq(
             p1.x, p2.x, std::numeric_limits<float32_t>::epsilon()) &&

           helper_functions::comparisons::rel_eq(
             p1.y, p2.y, std::numeric_limits<float32_t>::epsilon()) &&

           helper_functions::comparisons::rel_eq(
             p1.z, p2.z, std::numeric_limits<float32_t>::epsilon()) &&

           helper_functions::comparisons::rel_eq(
             p1.intensity, p2.intensity, std::numeric_limits<float32_t>::epsilon()) &&

           p1.ring == p2.ring &&

           helper_functions::comparisons::rel_eq(
             p1.azimuth, p2.azimuth, std::numeric_limits<float32_t>::epsilon()) &&
           helper_functions::comparisons::rel_eq(
             p1.distance, p2.distance, std::numeric_limits<float32_t>::epsilon()) &&

           p1.return_type == p2.return_type &&

           helper_functions::comparisons::rel_eq<double>(
             p1.time_stamp, p2.time_stamp, std::numeric_limits<float32_t>::epsilon());
  }
};

enum class PointIndex { X, Y, Z, Intensity, Ring, Azimuth, Distance, ReturnType, TimeStamp };

LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(azimuth);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(distance);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(return_type);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(time_stamp);

using PointXYZIRADRTGenerator = std::tuple<
  point_cloud_msg_wrapper::field_x_generator, point_cloud_msg_wrapper::field_y_generator,
  point_cloud_msg_wrapper::field_z_generator, point_cloud_msg_wrapper::field_intensity_generator,
  point_cloud_msg_wrapper::field_ring_generator, field_azimuth_generator, field_distance_generator,
  field_return_type_generator, field_time_stamp_generator>;

using PointBlock = std::vector<PointXYZIF>;
using PointPtrBlock = std::vector<const PointXYZIF *>;
/// \brief Stores basic configuration information, does some simple validity checking
static constexpr uint16_t POINT_BLOCK_CAPACITY = 512U;

// TODO(yunus.caliskan): switch to std::void_t when C++17 is available
/// \brief `std::void_t<> implementation
template <typename... Ts>
using void_t = void;
}  // namespace types
}  // namespace common
}  // namespace autoware

#endif  // COMMON__TYPES_HPP_
