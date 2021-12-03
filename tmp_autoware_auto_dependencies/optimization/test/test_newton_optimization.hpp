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

#ifndef TEST_NEWTON_OPTIMIZATION_HPP_
#define TEST_NEWTON_OPTIMIZATION_HPP_

#include <common/types.hpp>
#include <optimization/newtons_method_optimizer.hpp>

using autoware::common::types::float64_t;

namespace autoware
{
namespace common
{
namespace optimization
{
using Vector1D = Eigen::Matrix<float64_t, 1U, 1U>;

/// This is the expression for `y = (x + A)^N + B`.
/// If N is even, then the expression is convex with a global minima at -A.
class Polynomial1DObjective : public Expression<Polynomial1DObjective, Vector1D, 1U, 1U>
{
public:
  // getting aliases from the base class.
  using ExpressionT = Expression<Polynomial1DObjective, Eigen::Matrix<float64_t, 1U, 1U>, 1U, 1U>;
  using DomainValue = typename ExpressionT::DomainValue;
  using Value = typename ExpressionT::Value;
  using Jacobian = typename ExpressionT::Jacobian;
  using Hessian = typename ExpressionT::Hessian;
  using JacobianRef = Eigen::Ref<Jacobian>;
  using HessianRef = Eigen::Ref<Hessian>;

  Polynomial1DObjective(float64_t A_, int32_t N_, float64_t B_)
  : A{A_}, B{B_}, N{N_}, solution{-A_}, convex{((N_ % 2) == 0)} {}
  /// Get the result of an expression for a given parameter value.
  /// \param x Parameter value
  /// \return Evaluated score
  Value score_(const DomainValue & x)
  {
    return std::pow(x(0, 0) + A, N) + B;
  }

  /// Get the jacobian at a given parameter value.
  /// \param x Parameter value.
  /// \param out Evaluated jacobian matrix.
  void jacobian_(const DomainValue & x, JacobianRef out)
  {
    out(0, 0) = static_cast<float64_t>(N) * (std::pow( (x(0, 0) + A), N - 1));
  }

  /// Get the hessian at a given parameter value.
  /// \param x Parameter value.
  /// \param out Evaluated hessian matrix.
  void hessian_(const DomainValue & x, HessianRef out)
  {
    out(0, 0) = static_cast<float64_t>(N) * (std::pow(x(0, 0) + A, N - 2));
  }

  float64_t A;
  float64_t B;
  int32_t N;
  float64_t solution;
  bool convex;
};

class Polynomial1DOptimizationProblem : public
  UnconstrainedOptimizationProblem<Polynomial1DObjective, Eigen::Matrix<float64_t, 1U, 1U>, 1U>
{
public:
  using UnconstrainedOptimizationProblem::UnconstrainedOptimizationProblem;

  Polynomial1DObjective get_objective()
  {
    return objective();
  }
};

template<typename OptimmizationOptionsT, typename LineSearchT>
struct Polynomial1dParam
{
  Polynomial1DOptimizationProblem problem;
  Polynomial1DObjective::DomainValue x0;
  OptimmizationOptionsT options;
  LineSearchT line_searcher;
  TerminationType expected_termination;
};

}  // namespace optimization
}  // namespace common
}  // namespace autoware

#endif  // TEST_NEWTON_OPTIMIZATION_HPP_
