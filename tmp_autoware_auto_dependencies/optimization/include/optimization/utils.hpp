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

#ifndef OPTIMIZATION__UTILS_HPP_
#define OPTIMIZATION__UTILS_HPP_

#include <common/types.hpp>
#include <optimization/visibility_control.hpp>
#include <Eigen/Core>
#include <functional>
#include <limits>

using autoware::common::types::bool8_t;

namespace autoware
{
namespace common
{
namespace optimization
{
/// Configuration class to sepcify which terms should get
/// computed during the evaluation of an expression. By default all terms default to false.
/// Terms to be computed should be enabled/set by either the constructor or the setter
/// methods.
class OPTIMIZATION_PUBLIC ComputeMode
{
public:
  /// Constructor
  ComputeMode() = default;
  /// Constructor with initial values.
  /// \param score True if score is to be computed.
  /// \param jacobian True if jacobian is to be computed.
  /// \param hessian True if hessian is to be computed.
  ComputeMode(bool8_t score, bool8_t jacobian, bool8_t hessian);
  /// Set score to true, return the new state for method chaining.
  /// \return Current modified instance.
  ComputeMode & set_score() noexcept;
  /// Set jacobian to true, return the new state for method chaining.
  /// \return Current modified instance.
  ComputeMode & set_jacobian() noexcept;
  /// Set hessian to true, return the new state for method chaining.
  /// \return Current modified instance.
  ComputeMode & set_hessian() noexcept;

  /// Get if score term is enabled.
  /// \return True if score term is enabled.
  bool8_t score() const noexcept;
  /// Get if jacobian term is enabled.
  /// \return True if jacobian term is enabled.
  bool8_t jacobian() const noexcept;
  /// Get if hessian term is enabled.
  /// \return True if hessian term is enabled.
  bool8_t hessian() const noexcept;

  bool8_t operator==(const ComputeMode & other) const noexcept;
  bool8_t operator!=(const ComputeMode & other) const noexcept;

private:
  bool8_t m_score{false};
  bool8_t m_jacobian{false};
  bool8_t m_hessian{false};
};

/// Enum class representing expression terms
enum class ExpressionTerm
{
  SCORE,  // Evaluation of the expression
  JACOBIAN,  // Evaluation of the first derivative
  HESSIAN   // Evaluation of the second derivative
};

/// Generic equality comparison functor for eigen matrices.
class OPTIMIZATION_PUBLIC EigenComparator
{
public:
  template<typename T, int H, int W>
  typename std::enable_if_t<std::is_floating_point<T>::value, bool8_t>
  operator()(const Eigen::Matrix<T, H, W> & lhs, const Eigen::Matrix<T, H, W> & rhs) const
  {
    return lhs.isApprox(rhs, std::numeric_limits<T>::epsilon());
  }

  template<typename T, int H, int W>
  typename std::enable_if_t<std::is_integral<T>::value, bool8_t>
  operator()(const Eigen::Matrix<T, H, W> & lhs, const Eigen::Matrix<T, H, W> & rhs) const
  {
    return lhs == rhs;
  }
};

/// State machine to keep track of the cache state of an expression
/// \tparam DomainValue Value type
/// \tparam ComparatorT Equality comparison functor for DomainValue
template<typename DomainValue, typename ComparatorT = decltype(std::equal_to<DomainValue>())>
class OPTIMIZATION_PUBLIC CacheStateMachine
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// Constructor
  /// \param comparator Equality comparison functor object.
  explicit CacheStateMachine(const ComparatorT & comparator = ComparatorT())
  : m_comparator(comparator) {}

  /// Constructor with initial values.
  /// \param value Initial value
  /// \param mode Initial mode
  /// \param comparator Equality comparison functor object.
  CacheStateMachine(
    const DomainValue & value, const ComputeMode & mode,
    const ComparatorT & comparator = ComparatorT())
  : m_comparator(comparator), m_last_mode(mode), m_last_value(value)
  {
  }

  /// Update the state with the given parameter and the computation mode/
  /// \param value Parameter value used in computation
  /// \param mode Computation mode
  void update(const DomainValue & value, const ComputeMode & mode) noexcept
  {
    m_last_mode = mode;
    m_last_value = value;
  }

  /// Check if the term is already evaluated and cached for a given parameter
  /// \param x Parameter value
  /// \param term Expression term to query the cache status. Can be one of the following:
  /// SCORE, JACOBIAN, HESSIAN
  /// \return True if the result for this term with the given parameter matches the previous
  /// state and is already cached.
  bool8_t is_cached(const DomainValue & x, const ExpressionTerm & term) const noexcept
  {
    bool8_t ret = false;
    if (m_comparator(x, m_last_value)) {
      switch (term) {
        case ExpressionTerm::SCORE:
          ret = m_last_mode.score();
          break;
        case ExpressionTerm::JACOBIAN:
          ret = m_last_mode.jacobian();
          break;
        case ExpressionTerm::HESSIAN:
          ret = m_last_mode.hessian();
          break;
        default:
          ret = false;
      }
    }
    return ret;
  }

private:
  const ComparatorT m_comparator;
  DomainValue m_last_value;
  ComputeMode m_last_mode;
};

}  // namespace optimization
}  // namespace common
}  // namespace autoware

#endif  // OPTIMIZATION__UTILS_HPP_
