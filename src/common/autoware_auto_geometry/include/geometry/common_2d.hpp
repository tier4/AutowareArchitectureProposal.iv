// Copyright 2017-2019 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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
/// \file
/// \brief This file includes common functionality for 2D geometry, such as dot products

#ifndef GEOMETRY__COMMON_2D_HPP_
#define GEOMETRY__COMMON_2D_HPP_

#include <common/types.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

using autoware::common::types::float32_t;
using autoware::common::types::bool8_t;

namespace autoware
{
namespace common
{
namespace geometry
{
template<typename T>
inline T clamp(const T val, const T min, const T max)
{
  return (val < min) ? min : ((val > max) ? max : val);
}

/// \brief Temporary namespace for point adapter methods, for use with nonstandard point types
namespace point_adapter
{
/// \brief Gets the x value for a point
/// \return The x value of the point
/// \param[in] pt The point
/// \tparam PointT The point type
template<typename PointT>
inline auto x_(const PointT & pt)
{
  return pt.x;
}
/// \brief Gets the y value for a point
/// \return The y value of the point
/// \param[in] pt The point
/// \tparam PointT The point type
template<typename PointT>
inline auto y_(const PointT & pt)
{
  return pt.y;
}
/// \brief Gets the z value for a point
/// \return The z value of the point
/// \param[in] pt The point
/// \tparam PointT The point type
template<typename PointT>
inline auto z_(const PointT & pt)
{
  return pt.z;
}
/// \brief Gets a reference to the x value for a point
/// \return A reference to the x value of the point
/// \param[in] pt The point
/// \tparam PointT The point type
template<typename PointT>
inline auto & xr_(PointT & pt)
{
  return pt.x;
}
/// \brief Gets a reference to the y value for a point
/// \return A reference to The y value of the point
/// \param[in] pt The point
/// \tparam PointT The point type
template<typename PointT>
inline auto & yr_(PointT & pt)
{
  return pt.y;
}
/// \brief Gets a reference to the z value for a point
/// \return A reference to the z value of the point
/// \param[in] pt The point
/// \tparam PointT The point type
template<typename PointT>
inline auto & zr_(PointT & pt)
{
  return pt.z;
}
}  // namespace point_adapter

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief compute whether line segment rp is counter clockwise relative to line segment qp
/// \param[in] pt shared point for both line segments
/// \param[in] r point to check if it forms a ccw angle
/// \param[in] q reference point
/// \return whether angle formed is ccw. Three collinear points is considered ccw
template<typename T>
inline bool8_t ccw(const T & pt, const T & q, const T & r)
{
  using point_adapter::x_;
  using point_adapter::y_;
  return (((x_(q) - x_(pt)) * (y_(r) - y_(pt))) - ((y_(q) - y_(pt)) * (x_(r) - x_(pt)))) <= 0.0F;
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief compute p x q = p1 * q2 - p2 * q1
/// \param[in] pt first point
/// \param[in] q second point
/// \return 2d cross product
template<typename T>
inline float32_t cross_2d(const T & pt, const T & q)
{
  using point_adapter::x_;
  using point_adapter::y_;
  return (x_(pt) * y_(q)) - (y_(pt) * x_(q));
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief compute p * q = p1 * q1 + p2 * q2
/// \param[in] pt first point
/// \param[in] q second point
/// \return 2d scalar dot product
template<typename T>
inline float32_t dot_2d(const T & pt, const T & q)
{
  using point_adapter::x_;
  using point_adapter::y_;
  return (x_(pt) * x_(q)) + (y_(pt) * y_(q));
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief Compute the 2d difference between two points, p - q
/// \param[in] p The left hand side
/// \param[in] q The right hand side
/// \return A point with the difference in the x and y fields, all other fields are default
///         initialized
template<typename T>
T minus_2d(const T & p, const T & q)
{
  T r;
  using point_adapter::x_;
  using point_adapter::y_;
  point_adapter::xr_(r) = x_(p) - x_(q);
  point_adapter::yr_(r) = y_(p) - y_(q);
  return r;
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief The unary minus or negation operator applied to a single point's 2d fields
/// \param[in] p The left hand side
/// \return A point with the negation in the x and y fields, all other fields are default
///         initialized
template<typename T>
T minus_2d(const T & p)
{
  T r;
  point_adapter::xr_(r) = -point_adapter::x_(p);
  point_adapter::yr_(r) = -point_adapter::y_(p);
  return r;
}
/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief The 2d addition operation, p + q
/// \param[in] p The left hand side
/// \param[in] q The right hand side
/// \return A point with the sum in the x and y fields, all other fields are default
///         initialized
template<typename T>
T plus_2d(const T & p, const T & q)
{
  T r;
  using point_adapter::x_;
  using point_adapter::y_;
  point_adapter::xr_(r) = x_(p) + x_(q);
  point_adapter::yr_(r) = y_(p) + y_(q);
  return r;
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief The scalar multiplication operation, p * a
/// \param[in] p The point value
/// \param[in] a The scalar value
/// \return A point with the scaled x and y fields, all other fields are default
///         initialized
template<typename T>
T times_2d(const T & p, const float32_t a)
{
  T r;
  point_adapter::xr_(r) = point_adapter::x_(p) * a;
  point_adapter::yr_(r) = point_adapter::y_(p) * a;
  return r;
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief solve p + t * u = q + s * v
///        Ref: https://stackoverflow.com/questions/563198/
///             whats-the-most-efficent-way-to-calculate-where-two-line-segments-intersect
/// \param[in] pt anchor point of first line
/// \param[in] u direction of first line
/// \param[in] q anchor point of second line
/// \param[in] v direction of second line
/// \return intersection point
/// \throw std::runtime_error if lines are (nearly) collinear or parallel
template<typename T>
inline T intersection_2d(const T & pt, const T & u, const T & q, const T & v)
{
  const float32_t num = cross_2d(minus_2d(pt, q), u);
  float32_t den = cross_2d(v, u);
  constexpr auto FEPS = std::numeric_limits<float32_t>::epsilon();
  if (fabsf(den) < FEPS) {
    if (fabsf(num) < FEPS) {
      // collinear case, anything is ok
      den = 1.0F;
    } else {
      // parallel case, no valid output
      throw std::runtime_error(
              "intersection_2d: no unique solution (either collinear or parallel)");
    }
  }
  return plus_2d(q, times_2d(v, num / den));
}


/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief rotate point given precomputed sin and cos
/// \param[inout] pt point to rotate
/// \param[in] cos_th precomputed cosine value
/// \param[in] sin_th precompined sine value
template<typename T>
inline void rotate_2d(T & pt, const float32_t cos_th, const float32_t sin_th)
{
  const float32_t x = point_adapter::x_(pt);
  const float32_t y = point_adapter::y_(pt);
  point_adapter::xr_(pt) = (cos_th * x) - (sin_th * y);
  point_adapter::yr_(pt) = (sin_th * x) + (cos_th * y);
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief rotate by radian angle th in z direction with ccw positive
/// \param[in] pt reference point to rotate
/// \param[in] th_rad angle by which to rotate point
/// \return rotated point
template<typename T>
inline T rotate_2d(const T & pt, const float32_t th_rad)
{
  T q(pt);  // It's reasonable to expect a copy constructor
  const float32_t s = sinf(th_rad);
  const float32_t c = cosf(th_rad);
  rotate_2d(q, c, s);
  return q;
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief compute q s.t. p T q, or p * q = 0
///        This is the equivalent of a 90 degree ccw rotation
/// \param[in] pt point to get normal point of
/// \return point normal to p (unnormalized)
template<typename T>
inline T get_normal(const T & pt)
{
  T q(pt);
  point_adapter::xr_(q) = -point_adapter::y_(pt);
  point_adapter::yr_(q) = point_adapter::x_(pt);
  return q;
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief get magnitude of x and y components:
/// \param[in] pt point to get magnitude of
/// \return magitude of x and y components together
template<typename T>
inline float32_t norm_2d(const T & pt)
{
  return sqrtf(dot_2d(pt, pt));
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief Compute the closest point on line segment p-q to point r
///        Based on equations from https://stackoverflow.com/a/1501725 and
///        http://paulbourke.net/geometry/pointlineplane/
/// \param[in] p First point defining the line segment
/// \param[in] q Second point defining the line segment
/// \param[in] r Reference point to find the closest point to
/// \return Closest point on line segment p-q to point r
template<typename T>
inline T closest_segment_point_2d(const T & p, const T & q, const T & r)
{
  const T qp = minus_2d(q, p);
  const float32_t len2 = dot_2d(qp, qp);
  T ret = p;
  if (len2 > std::numeric_limits<float32_t>::epsilon()) {
    const float32_t t = clamp(dot_2d(minus_2d(r, p), qp) / len2, 0.0F, 1.0F);
    ret = plus_2d(p, times_2d(qp, t));
  }
  return ret;
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief Compute the distance from line segment p-q to point r
/// \param[in] p First point defining the line segment
/// \param[in] q Second point defining the line segment
/// \param[in] r Reference point to find the distance from the line segment to
/// \return Distance from point r to line segment p-q
template<typename T>
inline float32_t point_line_segment_distance_2d(const T & p, const T & q, const T & r)
{
  const T pq_r = minus_2d(closest_segment_point_2d(p, q, r), r);
  return norm_2d(pq_r);
}
}  // namespace geometry
}  // namespace common
}  // namespace autoware

#endif  // GEOMETRY__COMMON_2D_HPP_
