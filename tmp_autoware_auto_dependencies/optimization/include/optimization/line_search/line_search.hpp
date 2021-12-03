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

#ifndef OPTIMIZATION__LINE_SEARCH__LINE_SEARCH_HPP_
#define OPTIMIZATION__LINE_SEARCH__LINE_SEARCH_HPP_

#include <helper_functions/crtp.hpp>
#include <optimization/visibility_control.hpp>
#include <limits>
#include <cmath>
#include <algorithm>

namespace autoware
{
namespace common
{
namespace optimization
{
/// Base class (CRTP) to mange the step length during optimization.
template<typename Derived>
class OPTIMIZATION_PUBLIC LineSearch : public common::helper_functions::crtp<Derived>
{
public:
  // TODO(igor): should we force this to float? We mostly use 64 bit types in optimization.
  using StepT = float_t;

  /// Constructor.
  /// \param step_max Maximum step length. By default initialized to the minimum value.
  explicit LineSearch(const StepT step_max = std::numeric_limits<StepT>::min())
  {
    m_step_max = step_max;
  }

  /// Computes the optimal step for the optimization optimization_problem
  /// \tparam DomainValueT Parameter type.
  /// \tparam OptimizationProblemT Optimization optimization_problem type. Must be an
  /// implementation of `common::optimization::OptimizationProblem`.
  /// \param x0 Initial x value to do the line searching for.
  /// \param initial_step Initial step to start the search for the optimal step.
  /// \param optimization_problem optimization problem.
  /// \return Optimal step.
  template<typename DomainValueT, typename OptimizationProblemT>
  inline DomainValueT compute_next_step(
    const DomainValueT & x0, const DomainValueT & initial_step,
    OptimizationProblemT & optimization_problem)
  {
    return this->impl().compute_next_step_(x0, initial_step, optimization_problem);
  }

  /// Getter for the maximum step length
  /// \return The maximum step length.
  inline StepT get_step_max() const noexcept
  {
    return m_step_max;
  }

  /// Setter for the maximum step length
  /// \param step_max the new maximal step length
  inline void set_step_max(const StepT step_max) noexcept
  {
    m_step_max = step_max;
  }

private:
  StepT m_step_max;
};

}  // namespace optimization
}  // namespace common
}  // namespace autoware

#endif  // OPTIMIZATION__LINE_SEARCH__LINE_SEARCH_HPP_
