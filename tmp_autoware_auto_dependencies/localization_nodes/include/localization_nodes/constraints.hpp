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

#ifndef LOCALIZATION_NODES__CONSTRAINTS_HPP_
#define LOCALIZATION_NODES__CONSTRAINTS_HPP_

#include <helper_functions/template_utils.hpp>
#include <localization_common/optimized_registration_summary.hpp>
#include <type_traits>
#include <string>

namespace autoware
{
namespace localization
{
namespace localization_nodes
{
/// \brief Helper class for concepts-like API.
enum class Requires
{
  Dummy
};
namespace traits
{
/// \brief This struct specifies the interface requirements of `MapT` given `MapMsgT`
/// parameter of a relative localizer node instantiation. Please refer to the documentation of
/// the expressions  within the constraint to understand the API requirements better.
/// The constraint expresses the constraints as static assertions, hence if the constraint
/// is not satisfied, the compilation will not succeed. To be able to use the constraint,
/// either include in the list of template parameters via the `Requires` keyword as:
/// `Requires = MapConstraint<T1, T2>` or simply instantiate it anywhere  in the code as
/// `MapConstraint<T1, T2> constraint{};`
/// \tparam MapT Localization map type.
/// \tparam MapMsgT Serialized map message type.
template<typename MapT, typename MapMsgT>
struct MapConstraint
{
  /// \brief This expression requires a method that sets the map for a given message that contains
  /// the data.
  /// \param[in] msg Message instance containing the map data.
  template<typename Map>
  using call_set = decltype(std::declval<Map>().set(std::declval<const MapMsgT &>()));

  /// \brief  This expression requires a method that returns a const reference to the frame ID
  /// of the map.
  /// \return Map frame ID.
  template<typename Map>
  using call_frame_id = decltype(std::declval<Map>().frame_id());

  /// \brief This expression requires a method that returns the validity of the map.
  /// \return True if the map is valid and usable.
  template<typename Map>
  using call_valid = decltype(std::declval<Map>().valid());

  static_assert(
    common::helper_functions::expression_valid<call_set, MapT>::value,
    "The map should provide a `set(msg)` method");
  static_assert(
    common::helper_functions::expression_valid_with_return<call_frame_id, MapT,
    const std::string &>::value,
    "The map should provide a `frame_id()` method");
  static_assert(
    common::helper_functions::expression_valid_with_return<call_valid, MapT, bool>::value,
    "The map should provide a `valid(msg)` method");
  static constexpr Requires value{};
};

/// \brief This struct specifies the interface requirements of `LocalizerT`, given the `InputT`,
/// `MapT` and  `SummaryT` parameters of a relative localizer node. Please refer to the
/// documentation of the expressions within the constraint to understand the API requirements
/// better. The constraint expresses the constraints as static assertions, hence if the
/// constraint is not satisfied, the compilation will not succeed. To be able to use the
/// constraint, either include in the list of template parameters via the `Requires` keyword as:
/// `Requires = LocalizerConstraint<T1, T2, T3>` or simply instantiate it anywhere in the code as
/// `LocalizerConstraint<T1, T2, T3> constraint{};`
/// \tparam LocalizerT
/// \tparam InputT
/// \tparam MapT
/// \tparam SummaryT
template<typename LocalizerT, typename InputT, typename MapT,
  typename SummaryT = localization_common::OptimizedRegistrationSummary>
struct LocalizerConstraint
{
  /// \brief This expression requires a method that registers a measurement to the current map and
  /// returns the estimated pose of the vehicle and its validity.
  /// \param[in] msg Measurement message to register.
  /// \param[in] transform_initial Initial guess of the pose to initialize the localizer with
  /// in iterative processes like solving optimization problems.
  /// \param[in] map Map to register.
  /// \param[out] summary Raw pointer to the registration summary for optional summary filling.
  /// \return Pose estimate after registration.
  template<typename Localizer>
  using call_register = decltype(std::declval<Localizer>().register_measurement(
      std::declval<InputT>(),
      std::declval<geometry_msgs::msg::TransformStamped>(),
      std::declval<MapT>(),
      std::declval<SummaryT * const>())
  );

  static_assert(
    common::helper_functions::expression_valid_with_return<
      call_register, LocalizerT, geometry_msgs::msg::PoseWithCovarianceStamped>::value,
    "The localizer should provide a valid  `register_measurement(...)` method.");
  static constexpr Requires value{};
};
// External definition of static members:
template<typename MapT, typename MapMsgT>
constexpr Requires MapConstraint<MapT, MapMsgT>::value;
template<typename LocalizerT, typename InputT, typename MapT, typename SummaryT>
constexpr Requires LocalizerConstraint<LocalizerT, InputT, MapT, SummaryT>::value;
}  // namespace traits
}  // namespace localization_nodes
}  // namespace localization
}  // namespace autoware
#endif  // LOCALIZATION_NODES__CONSTRAINTS_HPP_
