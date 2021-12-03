// Copyright 2021 Apex.AI, Inc.
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

#ifndef NDT__CONSTRAINTS_HPP_
#define NDT__CONSTRAINTS_HPP_

#include <helper_functions/template_utils.hpp>
#include <string>
#include <type_traits>
#include <vector>

namespace autoware
{
namespace localization
{
namespace ndt
{
enum class Requires {};

namespace traits
{
/// \brief This constraint expresses the map interface requirements for the localizer
/// \tparam MapT
template<typename MapT>
struct LocalizationMapConstraint
{
  /// \brief  This expression requires a method that returns a const reference to the frame ID
  /// of the map.
  /// \return Map frame ID.
  template<typename Map>
  using call_frame_id = decltype(std::declval<Map>().frame_id());

  /// \brief  This expression requires a method that returns the (std::chrono) timestamp of the
  /// \return Map stamp.
  template<typename Map>
  using call_stamp = decltype(std::declval<Map>().stamp());

  static_assert(
    common::helper_functions::expression_valid_with_return<call_frame_id, MapT,
    const std::string &>::value,
    "The map should provide a `frame_id()` method");

  static_assert(
    common::helper_functions::expression_valid_with_return<call_stamp, MapT,
    std::chrono::system_clock::time_point>::value,
    "The map should provide a `stamp()` method");

  static constexpr Requires value{};
};

/// \brief This constraint expresses the map interface requirements for the localizer
/// \tparam MapT
template<typename MapT>
struct P2DNDTOptimizationMapConstraint
{
  /// \brief The map type should expose the voxel type.
  using Voxel = typename MapT::Voxel;
  using VoxelViewVector = std::vector<VoxelView<Voxel>>;
  using Point = Eigen::Vector3d;


  /// \brief  This expression requires a method that looks up the cells at the given location.
  /// \return Map frame ID.
  template<typename Map>
  using call_cell = decltype(std::declval<Map>().cell(std::declval<const Point &>()));

  /// \brief  This expression requires a method that returns the (std::chrono) timestamp of the
  /// \return Map frame ID.
  template<typename Map>
  using call_cell_size = decltype(std::declval<Map>().cell_size());

  static_assert(
    common::helper_functions::expression_valid_with_return<call_cell, MapT,
    const VoxelViewVector &>::value,
    "The map should provide a `cell(...)` method");

  static_assert(
    common::helper_functions::expression_valid_with_return<call_cell_size, MapT,
    const perception::filters::voxel_grid::PointXYZ &>::value,
    "The map should provide a `cell_size()` method");

  static constexpr Requires value{};
};

// Define static members
template<typename T>
constexpr Requires LocalizationMapConstraint<T>::value;
template<typename T>
constexpr Requires P2DNDTOptimizationMapConstraint<T>::value;
}  // namespace traits
}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // NDT__CONSTRAINTS_HPP_
