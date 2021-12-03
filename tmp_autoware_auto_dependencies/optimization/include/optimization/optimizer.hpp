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

#ifndef OPTIMIZATION__OPTIMIZER_HPP_
#define OPTIMIZATION__OPTIMIZER_HPP_

#include <optimization/optimizer_options.hpp>
#include <optimization/line_search/line_search.hpp>
#include <Eigen/SVD>

namespace autoware
{
namespace common
{
namespace optimization
{
// Optimization solver base class(CRTP) for a given optimization problem.
template<typename Derived>
class OPTIMIZATION_PUBLIC Optimizer : public common::helper_functions::crtp<Derived>
{
public:
  /// Solves `x_out` for an objective `optimization_problem` and an initial value `x0`
  /// \tparam OptimizationProblemT Optimization problem type. Must be an
  /// implementation of `common::optimization::OptimizationProblem`.
  /// \tparam DomainValueT Type of the parameter
  /// \tparam EigenSolverT Type of eigen solver to be used internallt for solving the
  /// necessary linear equations. By default set to `Eigen::LDLT`.
  /// \param optimization_problem optimization_problem optimization objective
  /// \param x0 initial value
  /// \param x_out optimized value
  /// \return summary object
  template<
    typename OptimizationProblemT,
    typename DomainValueT,
    typename EigenSolverT = Eigen::LDLT<typename OptimizationProblemT::Hessian>>
  OptimizationSummary solve(
    OptimizationProblemT & optimization_problem,
    const DomainValueT & x0,
    DomainValueT & x_out)
  {
    return this->impl().template solve_<OptimizationProblemT, DomainValueT, EigenSolverT>(
      optimization_problem, x0, x_out);
  }
};

}  // namespace optimization
}  // namespace common
}  // namespace autoware

#endif  // OPTIMIZATION__OPTIMIZER_HPP_
