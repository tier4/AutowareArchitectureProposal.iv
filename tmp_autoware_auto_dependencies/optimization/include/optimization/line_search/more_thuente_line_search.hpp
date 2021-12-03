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


// This file contains modified code from the following open source projects
// published under the licenses listed below:
//
// Software License Agreement (BSD License)
//
//  Point Cloud Library (PCL) - www.pointclouds.org
//  Copyright (c) 2010-2011, Willow Garage, Inc.
//  Copyright (c) 2012-, Open Perception, Inc.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   * Neither the name of the copyright holder(s) nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

#ifndef OPTIMIZATION__LINE_SEARCH__MORE_THUENTE_LINE_SEARCH_HPP_
#define OPTIMIZATION__LINE_SEARCH__MORE_THUENTE_LINE_SEARCH_HPP_

#include <optimization/line_search/line_search.hpp>
#include <optimization/utils.hpp>
#include <helper_functions/float_comparisons.hpp>

#include <limits>
#include <algorithm>
#include <utility>

namespace autoware
{
namespace common
{
namespace comp = helper_functions::comparisons;
namespace optimization
{

namespace detail
{
/// This value is used in More-Thuente paper without explanation (in the paper: Section 4, Case 3).
constexpr common::types::float32_t kDelta = 0.66F;
}  // namespace detail

///
/// @brief      This class describes a More-Thuente line search as presented in the paper
///             "Line Search Algorithms with Guaranteed Sufficient Decrease" by Jorge J. More and
///             David J. Thuente.
///
///             One notable change here is that this class will automatically detect if we are
///             trying to solve a minimization or the maximization problem by evaluating
///             \f$\phi^\prime(0)\f$. If
///             \f$\phi^\prime(0) < 0\f$ then we follow the suggestions in the paper exactle.
///             Otherwise, we assume we are dealing with a dual maximization problem and will flip
///             the objective function
///             \f$\phi\f$, thus making it again a minimization problem.
///
class OPTIMIZATION_PUBLIC MoreThuenteLineSearch : public LineSearch<MoreThuenteLineSearch>
{
public:
  ///
  /// @brief      An enum that defines the direction of optimization.
  ///
  enum class OptimizationDirection
  {
    kMinimization,
    kMaximization
  };

  ///
  /// @brief      Constructs a new instance.
  ///
  /// @param[in]  max_step                The maximum step that the process is allowed to make
  /// @param[in]  min_step                The minimum step that the process is allowed to make
  /// @param[in]  optimization_direction  The optimization direction
  /// @param[in]  mu                      Constant
  ///             \f$\mu\f$ (eq. 1.1), that forces a sufficient decrease of the function.
  /// @param[in]  eta                     Constant
  ///             \f$\eta\f$ (eq. 1.2), that forces the curvature condition.
  /// @param[in]  max_iterations          Maximum allowed iterations.
  ///
  explicit MoreThuenteLineSearch(
    const StepT max_step,
    const StepT min_step,
    const OptimizationDirection optimization_direction = OptimizationDirection::kMinimization,
    const StepT mu = 1.e-4F,
    const StepT eta = 0.1F,  // Default value suggested in Section 5 of the paper.
    const std::int32_t max_iterations = 10)
  : LineSearch{max_step},
    m_step_min{min_step},
    m_optimization_direction{optimization_direction},
    m_mu{mu},
    m_eta{eta},
    m_max_iterations{max_iterations}
  {
    if (min_step < 0.0F) {throw std::domain_error("Min step cannot be negative.");}
    if (max_step < min_step) {throw std::domain_error("Max step cannot be smaller than min step.");}
    if (mu < 0.0F || mu > 1.0F) {throw std::domain_error("mu must be in (0, 1).");}
    if (eta < 0.0F || eta > 1.0F) {throw std::domain_error("eta must be in (0, 1).");}
    if (max_iterations < 1) {throw std::domain_error("Less than 1 iteration is not allowed.");}
    m_compute_mode.set_score().set_jacobian();
  }

  ///
  /// @brief      Calculates the next step.
  ///
  /// @param[in]  x0                    Starting argument.
  /// @param[in]  initial_step          Initial step to initiate the search.
  /// @param      optimization_problem  The optimization problem for generating values of function
  ///                                   denoted as f in the paper.
  ///
  /// @tparam     DomainValueT          Type of values of the domain of the function f.
  /// @tparam     OptimizationProblemT  The type of the optimization problem that provides values of
  ///                                   function f evaluated on a domain point.
  ///
  /// @return     The new step to make in order to optimize the function.
  ///
  /// @note       If the length of initial_step is smaller than the m_step_min then an unmodified
  ///             initial_step will be returned.
  ///
  template<typename DomainValueT, typename OptimizationProblemT>
  DomainValueT compute_next_step_(
    const DomainValueT & x0,
    const DomainValueT & initial_step,
    OptimizationProblemT & optimization_problem);

private:
  /// Struct to represent an interval between unsorted points a_l and a_u.
  struct Interval
  {
    StepT a_l;
    StepT a_u;
  };

  /// This function is represented by letter phi in the paper (eq. 1.3).
  /// For an underlying function f and step a_t > 0 this becomes:
  ///   phi(a_t) = f(x + a_t * p)
  template<typename OptimizationProblemT>
  class ObjectiveFunction;

  ///
  /// This class describes an auxiliary function denoted as psi in the paper (just before eq. 2.1)
  /// For an objective function phi, constant mu in (0, 1), and step a_t > 0 this becomes:
  ///   psi(a_t) = phi(a_t) - mu * phi(a_t).derivative * a_t
  template<typename ObjectiveFunctionT>
  class AuxiliaryFunction;

  // Find the next step as described in section 4 of the paper.
  template<typename FunctionValueT>
  StepT find_next_step_length(
    const FunctionValueT & f_t, const FunctionValueT & f_l, const FunctionValueT & f_u);

  // Find the next [a_l, a_u] interval as described in the "Updating Algorithm" with function psi
  // and "Modifier Updating Algorithm" with function phi.
  template<typename FunctionValueT>
  Interval update_interval(
    const FunctionValueT & f_t, const FunctionValueT & f_l, const FunctionValueT & f_u);

  StepT m_step_min{};
  OptimizationDirection m_optimization_direction;
  ComputeMode m_compute_mode{};
  StepT m_mu{};
  StepT m_eta{};
  std::int32_t m_max_iterations{};
};

template<typename DomainValueT, typename OptimizationProblemT>
DomainValueT MoreThuenteLineSearch::compute_next_step_(
  const DomainValueT & x0, const DomainValueT & initial_step,
  OptimizationProblemT & optimization_problem)
{
  auto a_t = std::min(static_cast<StepT>(initial_step.norm()), get_step_max());
  if (a_t < m_step_min) {
    // We don't want to perform the line search as the initial step is out of allowed bounds. We
    // assume that the optimizer knows what it is doing and return the initial_step unmodified.
    return initial_step;
  }
  // Function phi as defined in eq. 1.3
  using FunctionPhi = ObjectiveFunction<OptimizationProblemT>;
  // Function phi as defined right before eq. 2.1
  using FunctionPsi = AuxiliaryFunction<FunctionPhi>;
  FunctionPhi phi{x0, initial_step, optimization_problem, m_optimization_direction};
  FunctionPsi psi{phi, m_mu};


  Interval interval{m_step_min, get_step_max()};
  const auto phi_0 = phi(0.0F);
  auto phi_t = phi(a_t);
  auto psi_t = psi(a_t);
  auto f_l = psi(interval.a_l);
  auto f_u = psi(interval.a_u);

  using ValueT = typename OptimizationProblemT::Value;

  bool use_auxiliary_function = true;
  // Follows the "Search Algorithm" as presented in the paper.
  for (auto step_iterations = 0; step_iterations < m_max_iterations; ++step_iterations) {
    constexpr decltype(psi_t.value) ZERO{};
    if ((psi_t.value <= ZERO) &&
      (std::abs(phi_t.derivative) <= static_cast<ValueT>(m_eta) * std::abs(phi_0.derivative)))
    {
      // We reached the termination condition as the step satisfies the strong Wolfe conditions (the
      // ones in the if condition). This means we have converged and are ready to return the found
      // step.
      break;
    }

    // Pick next step size by interpolating either phi or psi depending on which update algorithm is
    // currently being used.
    if (use_auxiliary_function) {
      a_t = find_next_step_length(psi_t, f_l, f_u);
    } else {
      a_t = find_next_step_length(phi_t, f_l, f_u);
    }
    if (a_t < m_step_min || std::isnan(a_t)) {
      // This can happen if we are closer than the minimum step to the optimum. We don't want to do
      // anything in this case.
      a_t = 0.0F;
      break;
    }
    phi_t = phi(a_t);
    psi_t = psi(a_t);

    // Decide if we want to switch to using a "Modified Updating Algorithm" (shown after theorem 3.2
    // in the paper) by switching from using function psi to using function phi. The decision
    // follows the logic in the paragraph right before theorem 3.3 in the paper.
    if (use_auxiliary_function && (psi_t.value <= 0.0 && psi_t.derivative > 0.0)) {
      use_auxiliary_function = false;
      // We now want to switch to using phi so compute the required values.
      f_l = phi(interval.a_l);
      f_u = phi(interval.a_u);
    }

    if (use_auxiliary_function) {
      // Update the interval that will be used to generate the next step using the
      // "Updating Algorithm" (right after theorem 2.1 in the paper).
      interval = update_interval(psi_t, f_l, f_u);
      f_l = psi(interval.a_l);
      f_u = psi(interval.a_u);
    } else {
      // Update the interval that will be used to generate the next step using the
      // "Modified Updating Algorithm" (right after theorem 3.2 in the paper).
      interval = update_interval(phi_t, f_l, f_u);
      f_l = phi(interval.a_l);
      f_u = phi(interval.a_u);
    }
    constexpr auto EPS = std::numeric_limits<StepT>::epsilon();
    if (comp::approx_eq(interval.a_u, interval.a_l, m_step_min, EPS)) {
      // The interval has converged to a point so we can stop here.
      a_t = interval.a_u;
      break;
    }
  }
  return a_t * phi.get_step_direction();
}

template<typename OptimizationProblemT>
class MoreThuenteLineSearch::ObjectiveFunction
{
  using ValueT = typename OptimizationProblemT::Value;
  using JacobianT = typename OptimizationProblemT::Jacobian;
  using DomainValueT = typename OptimizationProblemT::DomainValue;

public:
  /// A utility struct that holds the argument, value and derivative of a function.
  struct FunctionValue
  {
    StepT argument;
    ValueT value;
    ValueT derivative;
  };

  ObjectiveFunction(
    const DomainValueT & starting_state,
    const DomainValueT & initial_step,
    OptimizationProblemT & underlying_function,
    const OptimizationDirection direction)
  : m_starting_state{starting_state},
    m_step_direction{initial_step.normalized()},
    m_underlying_function{underlying_function}
  {
    m_compute_mode.set_score().set_jacobian();
    m_underlying_function.evaluate(m_starting_state, m_compute_mode);
    m_underlying_function.jacobian(m_starting_state, m_underlying_function_jacobian);
    const auto derivative = m_underlying_function_jacobian.dot(m_step_direction);
    switch (direction) {
      case OptimizationDirection::kMinimization:
        if (derivative > ValueT{0.0}) {
          m_step_direction *= -1.0;
        }
        break;
      case OptimizationDirection::kMaximization:
        if (derivative < ValueT{0.0}) {
          m_step_direction *= -1.0;
        }
        // The function phi must have a derivative < 0 following the introduction of the
        // More-Thuente paper. In case we want to solve a maximization problem, the derivative will
        // be positive and we need to make a dual problem from it by flipping the values of phi.
        m_multiplier = ValueT{-1.0};
        break;
    }
  }

  /// Get the value of phi for a given step.
  FunctionValue operator()(const StepT & step_size)
  {
    if (step_size < StepT{0.0}) {throw std::runtime_error("Step cannot be negative");}
    const auto current_state = m_starting_state + step_size * m_step_direction;
    m_underlying_function.evaluate(current_state, m_compute_mode);
    m_underlying_function.jacobian(current_state, m_underlying_function_jacobian);
    return {
      step_size,
      m_multiplier * m_underlying_function(current_state),
      m_multiplier * m_underlying_function_jacobian.dot(m_step_direction)};
  }

  /// Get the step direction.
  const DomainValueT & get_step_direction() const noexcept {return m_step_direction;}

private:
  DomainValueT m_starting_state;
  DomainValueT m_step_direction;
  OptimizationProblemT & m_underlying_function;
  ComputeMode m_compute_mode{};
  JacobianT m_underlying_function_jacobian;
  ValueT m_multiplier{1.0};
};


template<typename ObjectiveFunctionT>
class MoreThuenteLineSearch::AuxiliaryFunction
{
  using FunctionValue = typename ObjectiveFunctionT::FunctionValue;
  using ValueT = decltype(FunctionValue::value);

public:
  /// Constructs a new instance of function psi.
  AuxiliaryFunction(ObjectiveFunctionT & objective_function, const StepT & mu)
  : m_objective_function{objective_function},
    m_mu{mu},
    m_initial_objective_function_value{objective_function(0.0F)} {}

  /// Get the value of psi for a given step.
  FunctionValue operator()(const StepT & step_size)
  {
    const auto & objective_function_value = m_objective_function(step_size);
    const auto value =
      objective_function_value.value -
      m_initial_objective_function_value.value -
      static_cast<ValueT>(m_mu) * static_cast<ValueT>(step_size) *
      objective_function_value.derivative;
    const auto derivative =
      objective_function_value.derivative - static_cast<ValueT>(m_mu) *
      m_initial_objective_function_value.derivative;
    return {step_size, value, derivative};
  }

private:
  ObjectiveFunctionT & m_objective_function;
  StepT m_mu{};
  FunctionValue m_initial_objective_function_value{};
  FunctionValue m_value{};
};

template<typename FunctionValueT>
MoreThuenteLineSearch::StepT MoreThuenteLineSearch::find_next_step_length(
  const FunctionValueT & f_t, const FunctionValueT & f_l, const FunctionValueT & f_u)
{
  using ValueT = decltype(FunctionValueT::value);

  if (std::isnan(f_t.argument) || std::isnan(f_l.argument) || std::isnan(f_u.argument)) {
    throw std::runtime_error("Got nan values in the step computation function.");
  }
  constexpr auto kValueEps = 0.00001;
  constexpr auto kStepEps = 0.00001F;
  // A lambda to calculate the minimizer of the cubic that interpolates f_a, f_a_derivative, f_b and
  // f_b_derivative on [a, b]. Equation 2.4.52 [Sun, Yuan 2006]
  const auto find_cubic_minimizer = [kStepEps](const auto & f_a, const auto & f_b) -> StepT {
      if (comp::approx_eq(f_a.argument, f_b.argument, kStepEps, kStepEps)) {
        return f_a.argument;
      }
      const auto z = static_cast<ValueT>(3.0) * (f_a.value - f_b.value) /
        (static_cast<ValueT>(f_b.argument) - static_cast<ValueT>(f_a.argument)) + f_a.derivative +
        f_b.derivative;
      const auto w = std::sqrt(z * z - f_a.derivative * f_b.derivative);
      // Equation 2.4.56 [Sun, Yuan 2006]
      return static_cast<StepT>(
        static_cast<ValueT>(f_b.argument) -
        (static_cast<ValueT>(f_b.argument) - static_cast<ValueT>(f_a.argument)) *
        (f_b.derivative + w - z) /
        (f_b.derivative - f_a.derivative + static_cast<ValueT>(2.0) * w));
    };

  // A lambda to calculate the minimizer of the quadratic that interpolates f_a, f_b and f'_a
  const auto find_a_q = [kStepEps](
    const FunctionValueT & f_a, const FunctionValueT & f_b) -> StepT {
      if (comp::approx_eq(f_a.argument, f_b.argument, kStepEps, kStepEps)) {
        return f_a.argument;
      }
      return
        static_cast<StepT>(
        static_cast<ValueT>(f_a.argument) + static_cast<ValueT>(0.5) *
        (static_cast<ValueT>(f_b.argument) - static_cast<ValueT>(f_a.argument)) *
        (static_cast<ValueT>(f_b.argument) - static_cast<ValueT>(f_a.argument)) *
        f_a.derivative /
        (f_a.value - f_b.value +
        (static_cast<ValueT>(f_b.argument) - static_cast<ValueT>(f_a.argument)) *
        f_a.derivative));
    };

  // A lambda to calculate the minimizer of the quadratic that interpolates f'_a, and f'_b
  const auto find_a_s = [kStepEps](
    const FunctionValueT & f_a, const FunctionValueT & f_b) -> StepT {
      if (comp::approx_eq(f_a.argument, f_b.argument, kStepEps, kStepEps)) {
        return f_a.argument;
      }
      return static_cast<StepT>(
        static_cast<ValueT>(f_a.argument) +
        (static_cast<ValueT>(f_b.argument) - static_cast<ValueT>(f_a.argument)) * f_a.derivative /
        (f_a.derivative - f_b.derivative));
    };

  // We cover here all the cases presented in the More-Thuente paper in section 4.
  if (f_t.value > f_l.value) {  // Case 1 from section 4.
    const auto a_c = find_cubic_minimizer(f_l, f_t);
    const auto a_q = find_a_q(f_l, f_t);
    if (std::fabs(a_c - f_l.argument) < std::fabs(a_q - f_l.argument)) {
      return a_c;
    } else {
      return 0.5F * (a_q + a_c);
    }
  } else if (f_t.derivative * f_l.derivative < 0) {  // Case 2 from section 4.
    const auto a_c = find_cubic_minimizer(f_l, f_t);
    const auto a_s = find_a_s(f_l, f_t);
    if (std::fabs(a_c - f_t.argument) >= std::fabs(a_s - f_t.argument)) {
      return a_c;
    } else {
      return a_s;
    }
  } else if (comp::abs_lte(std::abs(f_t.derivative), std::abs(f_l.derivative), kValueEps)) {
    // Case 3 from section 4.
    const auto a_c = find_cubic_minimizer(f_l, f_t);
    const auto a_s = find_a_s(f_l, f_t);
    if (std::fabs(a_c - f_t.argument) < std::fabs(a_s - f_t.argument)) {
      return std::min(
        f_t.argument + detail::kDelta * (f_u.argument - f_t.argument),
        static_cast<StepT>(a_c));
    } else {
      return std::max(
        f_t.argument + detail::kDelta * (f_u.argument - f_t.argument),
        static_cast<StepT>(a_s));
    }
  } else {  // Case 4 from section 4.
    return find_cubic_minimizer(f_t, f_u);
  }
}

template<typename FunctionValueT>
MoreThuenteLineSearch::Interval MoreThuenteLineSearch::update_interval(
  const FunctionValueT & f_t, const FunctionValueT & f_l, const FunctionValueT & f_u)
{
  using ValueT = decltype(FunctionValueT::value);

  // Following either "Updating Algorithm" or "Modifier Updating Algorithm" depending on the
  // provided function f (can be psi or phi).
  if (f_t.value > f_l.value) {
    return {f_l.argument, f_t.argument};  // case a
  } else if (f_t.derivative * static_cast<ValueT>(f_t.argument - f_l.argument) < 0) {
    return {f_t.argument, f_u.argument};  // case b
  } else if (f_t.derivative * static_cast<ValueT>(f_t.argument - f_l.argument) > 0) {
    return {f_t.argument, f_l.argument};  // case c
  }
  // Converged to a point.
  return {f_t.argument, f_t.argument};
}

}  // namespace optimization
}  // namespace common
}  // namespace autoware


#endif  // OPTIMIZATION__LINE_SEARCH__MORE_THUENTE_LINE_SEARCH_HPP_
