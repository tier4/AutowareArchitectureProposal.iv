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

#ifndef OPTIMIZATION__OPTIMIZATION_PROBLEM_HPP_
#define OPTIMIZATION__OPTIMIZATION_PROBLEM_HPP_

#include <common/types.hpp>
#include <optimization/visibility_control.hpp>
#include <optimization/utils.hpp>
#include <helper_functions/crtp.hpp>
#include <Eigen/Core>
#include <vector>
#include <cstddef>
#include <memory>
#include <tuple>
#include <utility>

using autoware::common::types::bool8_t;

namespace autoware
{
namespace common
{
namespace optimization
{
/// CRTP base class for a mathematical expression.
/// \tparam Derived Implementation class. Must implement `score_(..)`, `jacobian_()` and
/// `hessian(..)`.
/// \tparam DomainValueT Parameter type.
/// \tparam NumJacobianColsT Number of columns in the jacobian matrix.
/// \tparam NumVarsT Number of variables.
template<typename Derived, typename DomainValueT, int NumJacobianColsT, int NumVarsT>
class OPTIMIZATION_PUBLIC Expression : public common::helper_functions::crtp<Derived>
{
public:
  static constexpr auto NumJacobianCols = NumJacobianColsT;
  static constexpr auto NumVars = NumVarsT;
  using DomainValue = DomainValueT;
  using Value = autoware::common::types::float64_t;
  using Jacobian = Eigen::Matrix<Value, NumVars, NumJacobianCols>;
  using Hessian = Eigen::Matrix<Value, NumVars, NumVars>;
  using JacobianRef = Eigen::Ref<Jacobian>;
  using HessianRef = Eigen::Ref<Hessian>;

  /// Get the result of an expression for a given parameter value.
  /// \param x Parameter value
  /// \return Evaluated score
  Value operator()(const DomainValue & x)
  {
    return this->impl().score_(x);
  }

  /// Get the jacobian at a given parameter value.
  /// \param x Parameter value.
  /// \param out Evaluated jacobian matrix.
  void jacobian(const DomainValue & x, JacobianRef out)
  {
    this->impl().jacobian_(x, out);
  }

  /// Get the hessian at a given parameter value.
  /// \param x Parameter value.{
  /// \param out Evaluated hessian matrix.
  void hessian(const DomainValue & x, HessianRef out)
  {
    this->impl().hessian_(x, out);
  }

  /// Pre-compute and cache the score/jacobian/hessian values. Which terms are to be computed is
  /// specified with the `ComputeMode` argument. By default, this function does nothing. You can
  /// implement and hide this function in your implementation to allow for some caching behavior.
  /// Though it is encouraged to use `CachedExpression` which already handles a big portion of the
  /// cache state management.
  /// \param x Parameter value.
  /// \param mode Computation mode. Which terms (score, jacobian, hessian) are to be computed
  /// can be set within this element.
  void evaluate(const DomainValue & x, const ComputeMode & mode)
  {
    (void)x;
    (void)mode;
  }
};

/// Expression class for cases when pre-evaluating elements or computing elements together
/// may be more efficient. This class implements the necessary boilerplate to manage the cache
/// state book-keeping.
/// This class implements `score_(..)`, `jacobian_(..)` and `hessian_(..)`; hence the
/// implementation must only implement `evaluate_(..)`
/// \tparam Derived CRTP implementation class. This class should implement `evaluate_()`.
/// \tparam DomainValueT Parameter type.
/// \tparam NumJacobianColsT Number of columns in the jacobian matrix.
/// \tparam NumVarsT Number of variables.
/// \tparam ComparatorT Comparator function for `DomainValueT`
template<typename Derived, typename DomainValueT, int NumJacobianColsT, int NumVarsT,
  typename ComparatorT>
class CachedExpression : public
  Expression<CachedExpression<Derived, DomainValueT,
    NumJacobianColsT, NumVarsT, ComparatorT>,
    DomainValueT, NumJacobianColsT, NumVarsT>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr auto NumJacobianCols = NumJacobianColsT;
  static constexpr auto NumVars = NumVarsT;
  using DomainValue = DomainValueT;
  using Value = autoware::common::types::float64_t;
  using Jacobian = Eigen::Matrix<Value, NumVars, NumJacobianCols>;
  using Hessian = Eigen::Matrix<Value, NumVars, NumVars>;
  using JacobianRef = Eigen::Ref<Eigen::Matrix<Value, NumVars, NumJacobianCols>>;
  using HessianRef = Eigen::Ref<Eigen::Matrix<Value, NumVars, NumVars>>;

  explicit CachedExpression(const ComparatorT & comparator = ComparatorT())
  : m_score{0.0}, m_jacobian(Jacobian::Zero()), m_hessian{Hessian::Zero()},
    m_cache_state(comparator) {}

  Value score_(const DomainValue & x)
  {
    if (!m_cache_state.is_cached(x, common::optimization::ExpressionTerm::SCORE)) {
      evaluate(x, ComputeMode{}.set_score());
    }
    return m_score;
  }

  void jacobian_(const DomainValue & x, JacobianRef out)
  {
    if (!m_cache_state.is_cached(x, common::optimization::ExpressionTerm::JACOBIAN)) {
      evaluate(x, ComputeMode{}.set_jacobian());
    }
    out = m_jacobian;
  }

  void hessian_(const DomainValue & x, HessianRef out)
  {
    if (!m_cache_state.is_cached(x, common::optimization::ExpressionTerm::SCORE)) {
      evaluate(x, ComputeMode{}.set_hessian());
    }
    out = m_hessian;
  }

  void evaluate(const DomainValue & x, const ComputeMode & mode)
  {
    if (!mode.score() && !mode.jacobian() && !mode.hessian()) {
      return;
    }
    m_cache_state.update(x, mode);
    // Call the implementation method.
    static_cast<Derived *>(this)->evaluate_(x, mode);
  }

protected:
  void set_score(Value score)
  {
    m_score = score;
  }

  void set_jacobian(const JacobianRef & jacobian)
  {
    m_jacobian = jacobian;
  }

  void set_hessian(const HessianRef & hessian)
  {
    m_hessian = hessian;
  }

private:
  Value m_score;
  Jacobian m_jacobian;
  Hessian m_hessian;
  CacheStateMachine<DomainValue, ComparatorT> m_cache_state;
};

/// A generalized representation of an optimization problem. Constrained
/// implementation classes must implement the following methods:
/// score_opt_(x);
/// jacobian_opt_(x, out);
/// hessian_opt_(x, out);
/// \tparam Derived Implementation class
/// \tparam DomainValueT The type for parameter to be optimized for.
/// \tparam NumJacobianColsT Number of columns in the jacobian matrix.
/// \tparam NumVarsT Number of variables.
/// \tparam ObjectiveT Objective implementation class.
/// \tparam EqualityConstraintsT A tuple of equality constraint expressions.
/// \tparam InequalityConstraintsT A tuple of inequality constraint expressions.
template<typename Derived, typename DomainValueT, int NumJacobianColsT, int NumVarsT,
  typename ObjectiveT, typename EqualityConstraintsT, typename InequalityConstraintsT>
class OPTIMIZATION_PUBLIC OptimizationProblem;


// Definition of OptimizationProblem.
// Template specialization is used for type deduction and forwarding constraint parameter packs
// to the tuples. This way "std::tuple" is enforced to be used on the constrained
// OptimizationProblem implementations.
template<typename Derived,
  typename ... EqualityConstraints,
  typename ... InequalityConstraints,
  typename ObjectiveT, typename DomainValueT, int NumJacobianColsT, int NumVarsT>
class OPTIMIZATION_PUBLIC OptimizationProblem<Derived, DomainValueT, NumJacobianColsT, NumVarsT,
    ObjectiveT,
    std::tuple<EqualityConstraints...>,
    std::tuple<InequalityConstraints...>>
  : public Expression<Derived, DomainValueT, NumJacobianColsT, NumVarsT>
{
public:
  using EqualityConstraintsT = std::tuple<EqualityConstraints...>;
  using InequalityConstraintsT = std::tuple<InequalityConstraints...>;
  using Value = autoware::common::types::float64_t;
  using Jacobian = Eigen::Matrix<Value, NumVarsT, NumJacobianColsT>;
  using Hessian = Eigen::Matrix<Value, NumVarsT, NumVarsT>;
  using JacobianRef = Eigen::Ref<Jacobian>;
  using HessianRef = Eigen::Ref<Hessian>;
  static constexpr bool8_t is_unconstrained{(std::tuple_size<EqualityConstraintsT>::value == 0U) &&
    (std::tuple_size<InequalityConstraintsT>::value == 0U)};

  OptimizationProblem(
    ObjectiveT && objective,
    EqualityConstraintsT && eq_constraints,
    InequalityConstraintsT && ineq_constraints
  )
  : m_objective(std::forward<ObjectiveT>(objective)),
    m_equality_constraints(std::forward<EqualityConstraintsT>(eq_constraints)),
    m_inequality_constraints(std::forward<InequalityConstraintsT>(ineq_constraints))
  {
  }

  template<typename ... Args, typename Dummy = void, typename = std::enable_if_t<
      is_unconstrained, Dummy>>
  OptimizationProblem(
    Args && ... args
  )
  : m_objective(std::forward<Args>(args)...)
  {
  }

  template<bool8_t enabled = is_unconstrained>
  typename std::enable_if_t<enabled, Value> score_(const DomainValueT & x)
  {
    return m_objective(x);
  }

  template<bool8_t enabled = is_unconstrained>
  typename std::enable_if_t<!enabled, Value> score_(const DomainValueT & x)
  {
    return opt_impl()->score_opt_(x);
  }

  template<bool8_t enabled = is_unconstrained>
  typename std::enable_if_t<enabled, void> jacobian_(const DomainValueT & x, JacobianRef out)
  {
    m_objective.jacobian(x, out);
  }

  template<bool8_t enabled = is_unconstrained>
  typename std::enable_if_t<!enabled, void> jacobian_(const DomainValueT & x, JacobianRef out)
  {
    opt_impl()->jacobian_opt_(x, out);
  }

  template<bool8_t enabled = is_unconstrained>
  typename std::enable_if_t<enabled, void> hessian_(const DomainValueT & x, HessianRef out)
  {
    m_objective.hessian(x, out);
  }

  template<bool8_t enabled = is_unconstrained>
  typename std::enable_if_t<!enabled, void> hessian_(const DomainValueT & x, HessianRef out)
  {
    opt_impl()->hessian_opt_(x, out);
  }

  template<bool8_t enabled = is_unconstrained>
  typename std::enable_if_t<enabled, void> evaluate(
    const DomainValueT & x,
    const ComputeMode & mode)
  {
    m_objective.evaluate(x, mode);
  }

  template<bool8_t enabled = is_unconstrained>
  typename std::enable_if_t<!enabled, void> evaluate(
    const DomainValueT & x,
    const ComputeMode & mode)
  {
    opt_impl()->evaluate_(x, mode);
  }

protected:
  ObjectiveT & objective()
  {
    return m_objective;
  }

  EqualityConstraintsT & equality_constraints()
  {
    return m_equality_constraints;
  }

  InequalityConstraintsT & inequality_constraints()
  {
    return m_inequality_constraints;
  }

private:
  Derived * opt_impl()
  {
    return static_cast<Derived *>(this);
  }

  ObjectiveT m_objective;
  EqualityConstraintsT m_equality_constraints;
  InequalityConstraintsT m_inequality_constraints;
};

/// Convenience class for unconstrained optimization problems. This is a thin wrapper
/// around the objective expression.
/// \tparam ObjectiveT Objective of the optimization problem.
/// \tparam DomainValueT Parameter type.
/// \tparam NumVarsT Number of variables.
template<typename ObjectiveT, typename DomainValueT, int NumVarsT>
class UnconstrainedOptimizationProblem : public
  OptimizationProblem<UnconstrainedOptimizationProblem<ObjectiveT, DomainValueT, NumVarsT>,
    DomainValueT, 1U, NumVarsT, ObjectiveT, std::tuple<>, std::tuple<>>
{
  using OptimizationProblem<UnconstrainedOptimizationProblem<ObjectiveT, DomainValueT, NumVarsT>,
    DomainValueT, 1U, NumVarsT, ObjectiveT, std::tuple<>, std::tuple<>>::OptimizationProblem;
};

}  // namespace optimization
}  // namespace common
}  // namespace autoware

#endif  // OPTIMIZATION__OPTIMIZATION_PROBLEM_HPP_
