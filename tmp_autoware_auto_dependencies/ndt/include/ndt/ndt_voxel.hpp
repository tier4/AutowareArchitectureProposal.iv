// Copyright 2019-2021 the Autoware Foundation
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

#ifndef NDT__NDT_VOXEL_HPP_
#define NDT__NDT_VOXEL_HPP_

#include <common/types.hpp>
#include <experimental/optional>
#include <ndt/ndt_common.hpp>
#include <voxel_grid/voxels.hpp>

#include <Eigen/Core>

using autoware::common::types::bool8_t;

namespace autoware
{
namespace localization
{
namespace ndt
{

enum class Invertibility
{
  INVERTIBLE,
  NOT_INVERTIBLE,
  UNKNOWN
};

/// Dynamic Voxel implementation for the NDT map. A dynamic voxel updates its state with each added
/// observation and hence it is to be only used when a raw point cloud is being
/// transformed into the ndt map representation.
class NDT_PUBLIC DynamicNDTVoxel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Point = Eigen::Vector3d;
  using Cov = Eigen::Matrix3d;
  DynamicNDTVoxel();

  // TODO(yunus.caliskan): make this configurable.
  // Number of points a voxel should have to count as occupied. Set to the dimension of a 3D point.
  static constexpr uint32_t NUM_POINT_THRESHOLD = 3U;

  /// Add a point to the cell, update the centroid and covariance. This function
  /// Uses Welford's online algorithm:
  /// https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
  /// \param pt Point to add to the voxel.
  void add_observation(const Point & pt);

  /// Try to stabilize the covariance
  /// \return True if stabilization succeeds and covariance is invertible
  bool8_t try_stabilize();

  /// Check if the cell contains enough points to be used in ndt matching
  /// \return True if cell has more points than NUM_POINT_THRESHOLD
  bool8_t usable() const noexcept;

  /// Returns the covariance of the points in the voxel. If not all points are used to update the
  /// covariance or the cell does not have enough points for covariance calculation,
  /// throws an error.
  /// \return covariance of the cell
  const Cov & covariance() const;

  /// Returns the inverse covariance calculated from the covariance. If the covariance is not
  /// invertible, throws an error
  /// \return inverse covariance of the cell
  std::experimental::optional<Cov> inverse_covariance() const;

  /// Returns the mean of the points in the cell. Throw if the cell does not have enough points.
  /// \return centroid of the cell
  const Point & centroid() const;

  /// Get number of points residing in the voxel.
  /// \return Number of points.
  uint64_t count() const noexcept;

private:
  Point m_centroid;
  Cov m_M2;  // Used in covariance computation.
  Cov m_covariance;
  Invertibility m_invertible{Invertibility::UNKNOWN};
  uint64_t m_num_points{0U};
};

/// Static Voxel implementation for the NDT map. A static voxel is used to represent a pre-computed
/// ndt cell, hence it doesn't contain any logic for updating its states.
class NDT_PUBLIC StaticNDTVoxel
{
public:
  using Point = Eigen::Vector3d;
  using Cov = Eigen::Matrix3d;
  /// Initialize an empty voxel
  StaticNDTVoxel();

  /// Initialize a voxel given the centroid and the covariance.
  /// \param centroid Centroid of the voxel.
  /// \param inv_covariance Covariance of the voxel.
  StaticNDTVoxel(const Point & centroid, const Cov & inv_covariance);

  /// Calculates and returns the covariance of the points in the voxel. Throw if voxel is empty.
  /// \return covariance of the cell
  Cov covariance() const;
  /// Returns the mean of the points in the cell. Throw if voxel is empty.
  /// \return centroid of the cell
  const Point & centroid() const;

  /// Returns the inverse covariance of the points in the voxel. Throw if voxel is empty.
  /// \return inverse covariance of the cell
  const Cov & inverse_covariance() const;

  /// Check if the cell is occupied and can be used in ndt matching
  /// \return True if cell is occupied
  bool8_t usable() const noexcept;

private:
  Point m_centroid;
  Cov m_inv_covariance;
  bool8_t m_occupied{false};
};
}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // NDT__NDT_VOXEL_HPP_
