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
/// \brief This file includes common functionality for 3D geometry, such as dot products

#ifndef GEOMETRY__COMMON_3D_HPP_
#define GEOMETRY__COMMON_3D_HPP_

#include <geometry/common_2d.hpp>

namespace autoware
{
namespace common
{
namespace geometry
{

/// \tparam T point type. Must have point adapters defined or have float members x, y and z
/// \brief compute p * q = p1 * q1 + p2 * q2 + p3 * 13
/// \param[in] pt first point
/// \param[in] q second point
/// \return 3d scalar dot product
template<typename T>
inline float32_t dot_3d(const T & pt, const T & q)
{
  using point_adapter::x_;
  using point_adapter::y_;
  using point_adapter::z_;
  return (x_(pt) * x_(q)) + (y_(pt) * y_(q) + z_(pt) * z_(q));
}

}  // namespace geometry
}  // namespace common
}  // namespace autoware

#endif  // GEOMETRY__COMMON_3D_HPP_
