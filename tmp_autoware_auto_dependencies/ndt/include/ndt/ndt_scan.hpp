// Copyright 2019 the Autoware Foundation
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

#ifndef NDT__NDT_SCAN_HPP_
#define NDT__NDT_SCAN_HPP_

#include <common/types.hpp>
#include <helper_functions/crtp.hpp>
#include <ndt/visibility_control.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <time_utils/time_utils.hpp>

#include <Eigen/Core>
#include <vector>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace localization
{
namespace ndt
{
/// CRTP Base class defining the required minimal API of an NDTScan.
/// \tparam Derived Derived class
/// \tparam NDTUnit The unit representing a single element within the scan.
/// \tparam IteratorT The type of iterator to iterate the scan.
template<typename Derived, typename NDTUnit, typename IteratorT>
class NDTScanBase : public common::helper_functions::crtp<Derived>
{
public:
  using TimePoint = std::chrono::system_clock::time_point;

  /// Get iterator pointing to the beginning of the internal container.
  /// \return Begin iterator.
  IteratorT begin() const
  {
    return this->impl().begin_();
  }

  /// Get iterator pointing to the end of the internal container.
  /// \return End iterator.
  IteratorT end() const
  {
    return this->impl().end_();
  }

  /// Clear the states and the internal cache of the scan.
  void clear()
  {
    return this->impl().clear_();
  }

  /// Check if there is any data in the scan.
  /// \return True if the internal container is empty.
  bool8_t empty()
  {
    return this->impl().empty_();
  }

  /// Insert a point cloud into the NDTScan. This is the step where the pointcloud is
  /// converted into the ndt scan representation.
  /// \param msg Point cloud to insert.
  void insert(const sensor_msgs::msg::PointCloud2 & msg)
  {
    this->impl().insert_(msg);
  }

  /// Number of points inside the scan.
  /// \return Number of points
  std::size_t size() const
  {
    return this->impl().size_();
  }

  TimePoint stamp()
  {
    return this->impl().stamp_();
  }
};

/// Represents a lidar scan in a P2D optimization problem. It is a wrapper around an
/// std::vector<Eigen::Vector3d>
class NDT_PUBLIC P2DNDTScan : public NDTScanBase<P2DNDTScan,
    Eigen::Vector3d, std::vector<Eigen::Vector3d>::const_iterator>
{
public:
  using Container = std::vector<Eigen::Vector3d>;
  using iterator = Container::const_iterator;

  // Make sure the given iterator type in the template is compatible with the used container.
  // container should have `iterator` type/alias defined.
  static_assert(
    std::is_same<decltype(std::declval<NDTScanBase>().begin()), iterator>::value,
    "P2DNDTScan: The iterator type parameter should match the "
    "iterator of the container.");

  /// Constructor
  /// \param msg Point cloud message to initialize this scan with.
  /// \param capacity Capacity of the scan. It should be configured according to the max. expected
  /// point cloud message size from the lidar.
  P2DNDTScan(
    const sensor_msgs::msg::PointCloud2 & msg,
    std::size_t capacity)
  {
    m_points.reserve(capacity);
    insert_(msg);
  }

  // Scans should be moved rather than being copied.
  P2DNDTScan(const P2DNDTScan &) = delete;
  P2DNDTScan & operator=(const P2DNDTScan &) = delete;

  // Explicitly declaring to default is needed since we explicitly deleted the copy methods.
  P2DNDTScan(P2DNDTScan &&) = default;
  P2DNDTScan & operator=(P2DNDTScan &&) = default;

  /// Constructor
  /// \param capacity Capacity of the scan. It should be configured according to the max. expected
  /// point cloud message size from the lidar.
  explicit P2DNDTScan(std::size_t capacity)
  {
    m_points.reserve(capacity);
  }

  /// Insert a point cloud into the NDTScan. This is the step where the pointcloud is
  /// converted into the ndt scan representation.
  /// \param msg Point cloud to insert.
  void insert_(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if (!m_points.empty()) {
      m_points.clear();
    }

    m_stamp = ::time_utils::from_message(msg.header.stamp);

    constexpr auto container_full_error = "received a lidar scan with more points than the "
      "ndt scan representation can contain. Please re-configure the scan"
      "representation accordingly.";

    if (msg.width > m_points.capacity()) {
      throw std::length_error(container_full_error);
    }
    using autoware::common::types::PointXYZ;
    point_cloud_msg_wrapper::PointCloud2View<PointXYZ> msg_view{msg};
    for (const auto & point : msg_view) {
      m_points.emplace_back(point.x, point.y, point.z);
    }
  }

  /// Get iterator pointing to the beginning of the internal container.
  /// \return Begin iterator.
  iterator begin_() const
  {
    return m_points.cbegin();
  }

  /// Get iterator pointing to the end of the internal container.
  /// \return End iterator.
  iterator end_() const
  {
    return m_points.cend();
  }

  /// Check if there is any data in the scan.
  /// \return True if the internal container is empty.
  bool8_t empty_()
  {
    return m_points.empty();
  }

  /// Clear the states and the internal cache of the scan.
  void clear_()
  {
    m_points.clear();
  }

  /// Number of points inside the scan.
  /// \return Number of points
  std::size_t size_() const
  {
    return m_points.size();
  }

  TimePoint stamp_()
  {
    return m_stamp;
  }

private:
  Container m_points;
  NDTScanBase::TimePoint m_stamp{};
};

}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // NDT__NDT_SCAN_HPP_
