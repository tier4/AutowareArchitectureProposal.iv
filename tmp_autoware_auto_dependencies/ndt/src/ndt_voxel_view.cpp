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

#include <ndt/ndt_voxel_view.hpp>
#include <utility>

namespace autoware
{
namespace localization
{
namespace ndt
{

using StaticView = VoxelView<StaticNDTVoxel>;
using DynamicView = VoxelView<DynamicNDTVoxel>;

StaticView::VoxelView(const StaticNDTVoxel & voxel) : Base(voxel), m_usable{voxel.usable()} {}

const StaticView::Cov & StaticView::inverse_covariance_() const
{
  return this->get().inverse_covariance();
}

const StaticView::Point & StaticView::centroid_() const
{
  return this->get().centroid();
}

bool8_t StaticView::usable_() const noexcept
{
  return m_usable;
}

DynamicView::VoxelView(const DynamicNDTVoxel & voxel) : Base{voxel}, m_usable{voxel.usable()} {
  if (m_usable) {
    auto res = voxel.inverse_covariance();
    if (res) {
      m_inverse_covariance = std::move(res.value());
    } else {
      m_usable = false;
    }
  }
}

const DynamicView::Cov & DynamicView::inverse_covariance_() const
{
  if (!m_usable) {
    throw std::runtime_error("Cannot get inverse covariance from an invalid voxel.");
  }
  return m_inverse_covariance;
}

const DynamicView::Point & DynamicView::centroid_() const
{
  return this->get().centroid();
}

bool8_t DynamicView::usable_() const noexcept
{
  return m_usable;
}
}  // namespace ndt
}  // namespace localization
}  // namespace autoware
