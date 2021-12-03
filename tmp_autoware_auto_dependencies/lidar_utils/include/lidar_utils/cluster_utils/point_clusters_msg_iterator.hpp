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

#ifndef LIDAR_UTILS__CLUSTER_UTILS__POINT_CLUSTERS_MSG_ITERATOR_HPP_
#define LIDAR_UTILS__CLUSTER_UTILS__POINT_CLUSTERS_MSG_ITERATOR_HPP_

#include <lidar_utils/visibility_control.hpp>

#include <autoware_auto_perception_msgs/msg/point_clusters.hpp>
#include <common/types.hpp>
#include <lidar_utils/cluster_utils/single_cluster_view.hpp>

#include <memory>

namespace autoware
{
namespace common
{
namespace lidar_utils
{

///
/// @brief      This class describes an iterator over the PointClusters message.
///
/// @details    This class stores an instance of a SingleClusterView and changes to which cluster
///             that view points when the value of this iterator is incremented or decremented.
///
/// @warning    As always when using iterators, the underlying container (a message in this case)
///             must outlive this iterator.
///
class LIDAR_UTILS_PUBLIC PointClustersMsgIterator
{
  using ClustersMsg = autoware_auto_perception_msgs::msg::PointClusters;

public:
  using iterator_category = std::random_access_iterator_tag;
  using value_type = SingleClusterView;
  using difference_type = std::ptrdiff_t;
  using reference = const SingleClusterView &;
  using pointer = const SingleClusterView * const;

  ///
  /// @brief      Constructs a new iterator.
  ///
  /// @param[in]  msg           The message to iterate over
  /// @param[in]  border_index  The current border index aka current cluster index
  ///
  explicit PointClustersMsgIterator(
    const ClustersMsg & msg,
    const std::uint32_t border_index) noexcept
  : m_msg{msg}, m_border_index{border_index} {}

  inline SingleClusterView operator*() const noexcept
  {
    return SingleClusterView{m_msg, m_border_index};
  }

  inline PointClustersMsgIterator & operator++() noexcept
  {
    m_border_index++;
    return *this;
  }

  inline PointClustersMsgIterator operator++(int) noexcept
  {
    PointClustersMsgIterator past = *this;
    ++(*this);
    return past;
  }

  inline PointClustersMsgIterator & operator--() noexcept
  {
    m_border_index--;
    return *this;
  }

  inline PointClustersMsgIterator operator--(int) noexcept
  {
    PointClustersMsgIterator past = *this;
    --(*this);
    return past;
  }

  inline bool operator==(const PointClustersMsgIterator & other) const noexcept
  {
    return (&m_msg == &other.m_msg) && (m_border_index == other.m_border_index);
  }

  inline bool operator!=(const PointClustersMsgIterator & other) const noexcept
  {
    return !(*this == other);
  }

private:
  const ClustersMsg & m_msg;
  std::uint32_t m_border_index;
};

}  // namespace lidar_utils
}  // namespace common
}  // namespace autoware

#endif  // LIDAR_UTILS__CLUSTER_UTILS__POINT_CLUSTERS_MSG_ITERATOR_HPP_
