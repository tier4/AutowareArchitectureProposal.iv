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
/// \brief This file defines the voxel grid data structure for downsampling point clouds

#ifndef VOXEL_GRID__VOXEL_GRID_HPP_
#define VOXEL_GRID__VOXEL_GRID_HPP_

#include <voxel_grid/config.hpp>
#include <voxel_grid/voxels.hpp>
#include <common/types.hpp>
#include <forward_list>
#include <unordered_map>

using autoware::common::types::bool8_t;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace voxel_grid
{

/// \brief A voxel grid data structure for downsampling point clouds
/// \tparam VoxelT The underlying voxel type, assumed to be a child class of Voxel with the
///                addition of the add_observation(PointT) method
template<typename VoxelT>
class VOXEL_GRID_PUBLIC VoxelGrid
{
  using Grid = std::unordered_map<uint64_t, VoxelT>;
  using IT = typename Grid::const_iterator;
  using OutputQueue = std::forward_list<IT>;
  // TODO(c.ho) static assert for better error messages

public:
  /// \brief Constructor
  /// \param[in] cfg The configuration class
  explicit VoxelGrid(const Config & cfg)
  : m_config(cfg),
    m_map(m_config.get_capacity()),
    m_output_pool(m_config.get_capacity()),
    m_output(m_output_pool.get_allocator()),
    m_new_voxels_called(false),
    m_last_output_begin(m_output.end())
  {
  }

  using point_t = typename VoxelT::point_t;
  /// \brief Inserts a point into the voxel grid, may result in new voxels being activated
  /// \param[in] pt The point to insert
  void insert(const point_t & pt)
  {
    // Get voxel
    const uint64_t idx = m_config.index(pt);
    // Check capacity if it would be a new voxel
    if (m_map.end() == m_map.find(idx)) {
      if (capacity() <= size()) {
        throw std::length_error{"VoxelGrid: insertion would overrun capacity"};
      }
    }
    // TODO(c.ho) #2023 adding a new node allocates memory...
    VoxelT & vx = m_map[idx];
    // Add to new queue if newly activated, set up any stateful information
    if (!vx.occupied()) {
      // Set hint here, rationale:
      // Using some conditional constructor would require C++17 (my preferred solution)
      // In addition, doing the voxel centroid calculation for every point is wasted work
      // I in general agree that an `init` or `configure` method is poor style, but in this case
      // The details of the voxel should be mostly hidden from the user
      //lint -e{523} NOLINT This is to support multiple voxel implementations, see above
      vx.configure(m_config, idx);
      // TODO(c.ho) clean up logic
      // Add to output queue
      const auto it = m_output_pool.begin();
      *it = m_map.find(idx);
      m_output.splice_after(m_output.cbefore_begin(), m_output_pool, m_output_pool.before_begin());
      if (m_new_voxels_called) {
        m_last_output_begin = m_output.begin();
        m_new_voxels_called = false;
      }
    }
    // Add observation to voxel
    vx.add_observation(pt);
  }
  /// \brief Inserts many points into the voxel grid, dispatches to the core insert method.
  /// \tparam IT The iterator type
  /// \param[in] begin The starting iterator
  /// \param[in] end An iterator pointing one past the last element to be inserted.
  template<typename IT>
  void insert(const IT begin, const IT end)
  {
    for (IT it = begin; it != end; ++it) {
      insert(*it);
    }
  }
  // TODO(c.ho) operator[]? centroid(uint64_t) out here?

  /// \brief Return a list of iterators pointing to newly activated voxels. Voxels will be removed
  ///        from queue during a subsequent call to this method.
  /// \return A list of newly activated iterators pointing to index-voxel pairs, from most recently
  ///         activated to least recently activated voxels.
  ///
  /// An example of queueing behavior:
  /// insert 0, 1, 2, 3, 4
  /// new_voxels: 4->3->2->1->0
  /// insert 0, 3, 5
  /// new_voxels: 5
  const OutputQueue & new_voxels()
  {
    // Remove old outputs and put it into the pool
    m_output_pool.splice_after(
      m_output_pool.cbefore_begin(),
      m_output,
      m_last_output_begin,
      m_output.end());
    m_new_voxels_called = true;
    m_last_output_begin = m_output.before_begin();
    return m_output;
  }

  /// \brief Returns an iterator to the first element of the voxel grid
  /// \return Iterator
  IT begin() const
  {
    return cbegin();
  }
  /// \brief Returns an iterator to the first element of the voxel grid
  /// \return Iterator
  IT cbegin() const
  {
    return m_map.cbegin();
  }
  /// \brief Returns an iterator to one past the last element of the voxel grid
  /// \return Iterator
  IT end() const
  {
    return cend();
  }
  /// \brief Returns an iterator to one past the last element of the voxel grid
  /// \return Iterator
  IT cend() const
  {
    return m_map.cend();
  }
  /// \brief Resets the state of the voxel grid
  void clear()
  {
    // TODO(c.ho) clear deallocates nodes #2023
    m_output_pool.splice_after(m_output_pool.before_begin(), m_output);
    m_new_voxels_called = false;
    m_last_output_begin = m_output.end();
    m_map.clear();
  }
  /// \brief Returns the current size of the voxel grid
  std::size_t size() const
  {
    return m_map.size();
  }
  /// \brief Returns the preallocated capacity of the voxel grid
  /// \return The preallocated capacity
  std::size_t capacity() const
  {
    return m_config.get_capacity();
  }
  /// \brief Whether the voxel grid is empty
  /// \return True or false
  bool8_t empty() const
  {
    return m_map.empty();
  }

private:
  const Config m_config;
  Grid m_map;
  // Mechanisms to support output queueing in static memory
  OutputQueue m_output_pool;
  OutputQueue m_output;
  bool8_t m_new_voxels_called;
  typename OutputQueue::iterator m_last_output_begin;
};  // class VoxelGrid

}  // namespace voxel_grid
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // VOXEL_GRID__VOXEL_GRID_HPP_
