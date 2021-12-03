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

#ifndef LIDAR_UTILS__CLUSTER_UTILS__POINT_CLUSTERS_VIEW_HPP_
#define LIDAR_UTILS__CLUSTER_UTILS__POINT_CLUSTERS_VIEW_HPP_

#include <autoware_auto_perception_msgs/msg/point_clusters.hpp>
#include <common/types.hpp>
#include <lidar_utils/cluster_utils/point_clusters_msg_iterator.hpp>
#include <lidar_utils/visibility_control.hpp>

#include <memory>

namespace autoware
{
namespace common
{
namespace lidar_utils
{

///
/// @brief      Point clusters view that wraps a const reference to the message and allows iterating
///             over single clusters.
///
/// @details    This class makes use of PointClustersMsgIterator to iterate over single clusters,
///             which provides access to these single clusters through the usage of the
///             SingleClusterView class. That class, in turn, allows iterating over points in the
///             cluster.
///
class PointClustersView
{
  using ClustersMsg = autoware_auto_perception_msgs::msg::PointClusters;

public:
  /// @brief      Constructs a new view.
  explicit PointClustersView(const ClustersMsg & msg) noexcept
  : m_clusters{msg} {}

  SingleClusterView operator[](const std::uint32_t cluster_index) const noexcept
  {
    return SingleClusterView{m_clusters, cluster_index};
  }

  inline PointClustersMsgIterator cbegin() const noexcept
  {
    return PointClustersMsgIterator{m_clusters, 0UL};
  }
  inline PointClustersMsgIterator cend() const noexcept
  {
    return PointClustersMsgIterator{
      m_clusters, static_cast<std::uint32_t>(m_clusters.cluster_boundary.size())};
  }
  inline PointClustersMsgIterator begin() const noexcept {return cbegin();}
  inline PointClustersMsgIterator end() const noexcept {return cend();}

  inline std::size_t size() const noexcept {return m_clusters.cluster_boundary.size();}
  inline types::bool8_t empty() const noexcept {return m_clusters.cluster_boundary.size() < 1UL;}

private:
  const ClustersMsg & m_clusters;
};


}  // namespace lidar_utils
}  // namespace common
}  // namespace autoware

#endif  // LIDAR_UTILS__CLUSTER_UTILS__POINT_CLUSTERS_VIEW_HPP_
