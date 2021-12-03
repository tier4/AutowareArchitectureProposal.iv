// Copyright 2020 the Autoware Foundation
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

#ifndef NDT__NDT_VOXEL_VIEW_HPP_
#define NDT__NDT_VOXEL_VIEW_HPP_

#include <ndt/ndt_voxel.hpp>

namespace autoware
{
namespace localization
{
namespace ndt
{

template<typename VoxelT>
class NDT_PUBLIC VoxelView;

/// Base CRTP interface for the voxel view. It assumes a cached
/// centroid and inverse covariance are provided by the implementation. This interface
/// does not contain a covariance access function as it's not directly needed for the
/// gaussian term shared in ndt optimization problems.
/// It should be noted that, as this is purely a view of an existing voxel,
/// it should not outlive the voxel it is referring to.
/// \tparam VoxelT Type of voxel to view.
/// \tparam Derived Implementation. Expected to be of `type VoxelView<VoxelT>`.
template<typename VoxelT, typename Derived>
class NDT_PUBLIC VoxelViewBase : public common::helper_functions::crtp<Derived>
{
public:
  using Point = Eigen::Vector3d;
  using Cov = Eigen::Matrix3d;
  explicit VoxelViewBase(const VoxelT & vx)
  : m_data_ref{vx} {}

  const Point & centroid() const
  {
    return this->impl().centroid_();
  }
  const Cov & inverse_covariance() const
  {
    return this->impl().inverse_covariance_();
  }
  bool8_t usable() const noexcept
  {
    return this->impl().usable_();
  }
  const VoxelT & get() const noexcept
  {
    return m_data_ref;
  }

private:
  const VoxelT & m_data_ref;
};

/// VoxelViewBase implementation for `StaticNDTVoxel`. It's just a pure wrapper.
template<>
class NDT_PUBLIC VoxelView<StaticNDTVoxel>
  : public VoxelViewBase<StaticNDTVoxel, VoxelView<StaticNDTVoxel>>
{
public:
  using Point = Eigen::Vector3d;
  using Cov = Eigen::Matrix3d;
  using Base = VoxelViewBase<StaticNDTVoxel, VoxelView<StaticNDTVoxel>>;
  explicit VoxelView(const StaticNDTVoxel & voxel);

  const Cov & inverse_covariance_() const;
  const Point & centroid_() const;
  bool8_t usable_() const noexcept;

private:
  bool8_t m_usable;
};


/// VoxelViewBase implementation for `DynamicNDTVoxel`. On construction, it will compute the
/// covariance. If the cell is not usable, the value returned by `inverse_covariance_` will
/// be have an invalid value, thus the user should always first check the usability of a dynamic
/// voxel view before querying its inverse covariance.
template<>
class NDT_PUBLIC VoxelView<DynamicNDTVoxel>
  : public VoxelViewBase<DynamicNDTVoxel, VoxelView<DynamicNDTVoxel>>
{
public:
  using Base = VoxelViewBase<DynamicNDTVoxel, VoxelView<DynamicNDTVoxel>>;
  using Point = Eigen::Vector3d;
  using Cov = Eigen::Matrix3d;
  explicit VoxelView(const DynamicNDTVoxel & voxel);

  const Cov & inverse_covariance_() const;
  const Point & centroid_() const;
  bool8_t usable_() const noexcept;

private:
  Cov m_inverse_covariance;
  bool8_t m_usable{true};
};


}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // NDT__NDT_VOXEL_VIEW_HPP_
