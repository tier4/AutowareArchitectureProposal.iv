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
// Developed by Apex.AI, Inc.

#include <common/visibility_control.hpp>

#include <cstdint>
#include <tuple>
#include <type_traits>

#ifndef COMMON__TYPE_TRAITS_HPP_
#define COMMON__TYPE_TRAITS_HPP_

namespace autoware
{
namespace common
{
namespace type_traits
{
/// Find an index of a type in a tuple
template<class QueryT, class TupleT>
struct COMMON_PUBLIC index
{
  static_assert(!std::is_same<TupleT, std::tuple<>>::value, "Could not find QueryT in given tuple");
};

/// Specialization for a tuple that starts with the HeadT type. End of recursion.
template<class HeadT, class ... Tail>
struct COMMON_PUBLIC index<HeadT, std::tuple<HeadT, Tail...>>
  : std::integral_constant<std::int32_t, 0> {};

/// Specialization for a tuple with a type different to QueryT that calls the recursive step.
template<class QueryT, class HeadT, class ... Tail>
struct COMMON_PUBLIC index<QueryT, std::tuple<HeadT, Tail...>>
  : std::integral_constant<std::int32_t, 1 + index<QueryT, std::tuple<Tail...>>::value> {};

///
/// @brief      Visit every element in a tuple.
///
///             This specialization indicates the end of the recursive tuple traversal.
///
/// @tparam     I         Current index.
/// @tparam     Callable  Callable type, usually a lambda with one auto input parameter.
/// @tparam     TypesT    Types in the tuple.
///
/// @return     Does not return anything. Capture variables in a lambda to return any values.
///
template<std::size_t I = 0UL, typename Callable, typename ... TypesT>
COMMON_PUBLIC inline typename std::enable_if_t<I == sizeof...(TypesT)>
visit(std::tuple<TypesT...> &, Callable) {}
/// @brief      Same as the previous specialization but for const tuple.
template<std::size_t I = 0UL, typename Callable, typename ... TypesT>
COMMON_PUBLIC inline typename std::enable_if_t<I == sizeof...(TypesT)>
visit(const std::tuple<TypesT...> &, Callable) {}

///
/// @brief      Visit every element in a tuple.
///
///             This specialization is used to apply the callable to an element of a tuple and
///             recursively call this function on the next one.
///
/// @param      tuple     The tuple instance
/// @param[in]  callable  A callable, usually a lambda with one auto input parameter.
///
/// @tparam     I         Current index.
/// @tparam     Callable  Callable type, usually a lambda with one auto input parameter.
/// @tparam     TypesT    Types in the tuple.
///
/// @return     Does not return anything. Capture variables in a lambda to return any values.
///
template<std::size_t I = 0UL, typename Callable, typename ... TypesT>
COMMON_PUBLIC inline typename std::enable_if_t<I != sizeof...(TypesT)>
visit(std::tuple<TypesT...> & tuple, Callable callable)
{
  callable(std::get<I>(tuple));
  visit<I + 1UL, Callable, TypesT...>(tuple, callable);
}
/// @brief      Same as the previous specialization but for const tuple.
template<std::size_t I = 0UL, typename Callable, typename ... TypesT>
COMMON_PUBLIC inline typename std::enable_if_t<I != sizeof...(TypesT)>
visit(const std::tuple<TypesT...> & tuple, Callable callable)
{
  callable(std::get<I>(tuple));
  visit<I + 1UL, Callable, TypesT...>(tuple, callable);
}

/// @brief      A class to compute a conjunction over given traits.
template<class ...>
struct COMMON_PUBLIC conjunction : std::true_type {};
/// @brief      A conjunction of another type shall derive from that type.
template<class TraitT>
struct COMMON_PUBLIC conjunction<TraitT>: TraitT {};
template<class TraitT, class ... TraitsTs>
struct COMMON_PUBLIC conjunction<TraitT, TraitsTs...>
  : std::conditional_t<static_cast<bool>(TraitT::value), conjunction<TraitsTs...>, TraitT> {};

}  // namespace type_traits
}  // namespace common
}  // namespace autoware

#endif  // COMMON__TYPE_TRAITS_HPP_
