// Copyright 2021 the Autoware Foundation
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

#ifndef LIDAR_UTILS__CLUSTER_UTILS__SINGLE_CLUSTER_VIEW_HPP_
#define LIDAR_UTILS__CLUSTER_UTILS__SINGLE_CLUSTER_VIEW_HPP_

#include <autoware_auto_perception_msgs/msg/point_clusters.hpp>
#include <common/types.hpp>
#include <lidar_utils/visibility_control.hpp>

#include <memory>

namespace autoware
{
namespace common
{
namespace lidar_utils
{

///
/// @brief      Single cluster view that wraps a const reference to the clusters message and
///             provides convenient access functions and allows iterating over the points in a
///             single cluster
///
/// @warning    As always when using the view paradigm, the underlying container (a message in this
///             case) must outlive this view.
///
class LIDAR_UTILS_PUBLIC SingleClusterView
{
  using ClustersMsg = autoware_auto_perception_msgs::msg::PointClusters;
  using Point = ClustersMsg::_points_type::value_type;
  using PointConstIterator = ClustersMsg::_points_type::const_iterator;
  using PointConstReverseIterator = ClustersMsg::_points_type::const_reverse_iterator;

public:
  ///
  /// @brief      Constructs a new view from a message and the current cluster index.
  ///
  /// @param[in]  msg           The message reference that this class wraps
  /// @param[in]  border_index  The border index, aka index of the cluster in the clusters message
  ///
  explicit SingleClusterView(
    const ClustersMsg & msg,
    const std::uint32_t border_index) noexcept
  : m_msg{msg}, m_border_index{border_index} {}

  inline PointConstIterator cbegin() const noexcept
  {
    return m_msg.points.cbegin() + start_point_index();
  }

  inline PointConstIterator cend() const noexcept
  {
    return m_msg.points.cbegin() + m_msg.cluster_boundary[m_border_index];
  }

  inline PointConstIterator begin() const noexcept {return cbegin();}
  inline PointConstIterator end() const noexcept {return cend();}

  /// Allow direct access to the points in the current cluster
  inline const Point & operator[](const std::size_t point_index_in_cluster) const noexcept
  {
    return m_msg.points[start_point_index() + point_index_in_cluster];
  }

  /// Get number of points in this cluster.
  inline std::size_t size() const noexcept
  {
    return m_msg.cluster_boundary[m_border_index] - start_point_index();
  }

  /// Check if the cluster is empty, i.e., contains no points.
  inline types::bool8_t empty() const noexcept
  {
    return (m_msg.cluster_boundary[m_border_index] - start_point_index()) < 1U;
  }

private:
  inline std::uint32_t LIDAR_UTILS_LOCAL start_point_index() const noexcept
  {
    return m_border_index > 0U ? m_msg.cluster_boundary[m_border_index - 1U] : 0U;
  }

  const ClustersMsg & m_msg;
  std::uint32_t m_border_index;
};

}  // namespace lidar_utils
}  // namespace common
}  // namespace autoware

#endif  // LIDAR_UTILS__CLUSTER_UTILS__SINGLE_CLUSTER_VIEW_HPP_
