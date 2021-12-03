// Copyright 2017-2018 the Autoware Foundation
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
/// \brief This file defines the core Voxel class

#ifndef VOXEL_GRID__VOXEL_HPP_
#define VOXEL_GRID__VOXEL_HPP_


#include <voxel_grid/visibility_control.hpp>

namespace autoware
{
namespace perception
{
namespace filters
{
/// \brief Resources relating to the voxel grid package
namespace voxel_grid
{

/// \brief A simple class to accumulate points for a voxel and emit the centroid
///
/// The VoxelGrid data structure assumes the voxel type subclasses from this class, and
/// adds one additional function with the signature void add_observation(const PointT & pt);
/// \tparam PointT The point type, must have float32_t members x, y, and z
template<typename PointT>
class VOXEL_GRID_PUBLIC Voxel
{
public:
  using point_t = PointT;
  /// \brief Default constructor, corresponds to an empty/uninitialized voxel
  Voxel()
  : m_num_points{0U}
  {
  }
  /// \brief Point constructor, voxel is assumed to be created based on this point
  explicit Voxel(const PointT & pt)
  : m_num_points(1U),
    m_centroid(pt)
  {
  }
  /// \brief Conversion operator to bool, pass through to occupied
  /// \return Whether or not the object is occuped
  explicit operator bool() const
  {
    return occupied();
  }
  /// \brief Conversion operator to PointT, pass through to get
  /// \return A copy of the underlying centroid
  /// \throw std::out_of_range If voxel is not occupied
  explicit operator PointT() const
  {
    return get();
  }

  /// \brief Whether or not this object has at least one point associated with it
  /// \return If the voxel has a point associated with it
  bool occupied() const
  {
    return count() > 0U;
  }

  /// \brief Resets the centroid for incremental updating
  void clear()
  {
    m_num_points = 0U;
  }

  /// \brief Emit the centroid
  /// \return A const reference to the centroid
  /// \throw std::out_of_range If voxel is not occupied
  const PointT & get() const
  {
    if (!occupied()) {
      throw std::out_of_range("Voxel: Cannot get point from an unoccupied voxel");
    }
    return m_centroid;
  }

  /// \brief Get the current number of points associated with this voxel
  /// \return The numbre of points
  uint32_t count() const
  {
    return m_num_points;
  }

protected:
  /// \brief Sets the count. This is to properly encapsulate the member variables. Intended to be
  ///        used by child classes.
  /// \param[in] next The new count to set the number points
  void set_count(const uint32_t next)
  {
    m_num_points = next;
  }

  /// \brief Sets the centroid. This is to properly encapsulate the member variables. Intended to be
  ///        used by child classes.
  /// \param[in] pt The new centroid
  void set_centroid(const PointT & pt)
  {
    m_centroid = pt;
  }

private:
  // TODO(c.ho) static_assert for better error messages
  uint32_t m_num_points;
  PointT m_centroid;
};  // class Voxel
}  // namespace voxel_grid
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // VOXEL_GRID__VOXEL_HPP_
