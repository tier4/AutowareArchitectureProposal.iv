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

#ifndef OPTIMIZATION__LINE_SEARCH__FIXED_LINE_SEARCH_HPP_
#define OPTIMIZATION__LINE_SEARCH__FIXED_LINE_SEARCH_HPP_

#include <optimization/line_search/line_search.hpp>

#include <limits>

namespace autoware
{
namespace common
{
namespace optimization
{

/// Class to use a fixed step length during optimization.
class OPTIMIZATION_PUBLIC FixedLineSearch : public LineSearch<FixedLineSearch>
{
public:
  /// Constructor.
  /// \param step Fixed step to be used.
  explicit FixedLineSearch(const StepT step = std::numeric_limits<StepT>::min())
  : LineSearch{step} {}

  /// Returns directly the pre-set (maximum) step length
  /// \return The fixed step length.
  template<typename DomainValueT, typename OptimizationProblemT>
  DomainValueT compute_next_step(
    const DomainValueT &, DomainValueT & step_direction,
    OptimizationProblemT &) const noexcept
  {
    return get_step_max() * step_direction.normalized();
  }
};
}  // namespace optimization
}  // namespace common
}  // namespace autoware

#endif  // OPTIMIZATION__LINE_SEARCH__FIXED_LINE_SEARCH_HPP_
