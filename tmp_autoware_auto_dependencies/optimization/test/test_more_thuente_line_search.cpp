// Copyright 2019-2020 the Autoware Foundation
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

#include <optimization/line_search/more_thuente_line_search.hpp>
#include <optimization/optimization_problem.hpp>

#include <gtest/gtest.h>


using FloatT = autoware::common::types::float64_t;
using Matrix = Eigen::Matrix<FloatT, 1, 1>;
using Vector = Eigen::Matrix<FloatT, 1, 1>;

using autoware::common::optimization::MoreThuenteLineSearch;

constexpr FloatT kRelaxedEpsilon{0.01};

///
/// @brief      A quadratic function with an extremum at x_offset.
///
class QuadraticFunction
  : public autoware::common::optimization::Expression<QuadraticFunction, Vector, 1, 1>
{
public:
  explicit QuadraticFunction(FloatT multiplier = 1.0, Vector x_offset = Vector{0.0})
  : m_multiplier{multiplier}, m_x_offset{x_offset} {}
  FloatT score_(const Vector & x) {return m_multiplier * (x - m_x_offset) * (x - m_x_offset);}
  void jacobian_(const Vector & x, JacobianRef out) {out << m_multiplier * 2.0 * (x - m_x_offset);}

private:
  FloatT m_multiplier{};
  Vector m_x_offset{};
};

///
/// @brief      A function from the More-Thuente paper shown in Figure 3.
///
class FunctionFromFigure3
  : public autoware::common::optimization::Expression<FunctionFromFigure3, Vector, 1, 1>
{
public:
  FloatT score_(const Vector & x)
  {
    const auto value = -x(0, 0) / (x(0, 0) * x(0, 0) + 2.0);
    return value;
  }
  void jacobian_(const Vector & x, JacobianRef out)
  {
    const FloatT a = x(0, 0);
    const FloatT a_2 = a * a;
    out << (a_2 - 2.0) / ((a_2 + 2.0) * (a_2 + 2.0));
  }
};

/// @test       Check that we can minimize a simple parabola with a minimum at 0.
TEST(MoreThuenteLineSearchTest, SimpleQuadraticFunctionMinimization) {
  QuadraticFunction f;
  Matrix jacobian;

  auto max_step{10.0F};
  auto step = MoreThuenteLineSearch{max_step, 0.001F}.compute_next_step(
    Vector{-2.0}, Vector{1.0}, f);
  // We expect that we will be able to find a proper step in a single call. The max step does not
  // satisfy the strong Wolfe conditions and we will be searching for the proper step.
  EXPECT_NEAR(2.0, step(0, 0), kRelaxedEpsilon);

  // Check that the length of the direction vector does not play any role.
  step = MoreThuenteLineSearch{max_step, 0.001F}.compute_next_step(Vector{-2.0}, Vector{2.0}, f);
  EXPECT_NEAR(2.0, step(0, 0), kRelaxedEpsilon);

  step = MoreThuenteLineSearch{max_step, 0.001F}.compute_next_step(Vector{2.0}, Vector{-1.0}, f);
  // Same as above, but in reverse direction. Still minimization.
  EXPECT_NEAR(-2.0, step(0, 0), kRelaxedEpsilon);

  auto min_step = 0.1F;
  auto initial_step{Vector{0.01}};
  step = MoreThuenteLineSearch{max_step, min_step}.compute_next_step(Vector{-2.0}, initial_step, f);
  // The initial step is smaller than the minimum step, so we return the initial step.
  EXPECT_NEAR(initial_step(0, 0), step(0, 0), kRelaxedEpsilon);

  max_step = 0.1F;
  step = MoreThuenteLineSearch{max_step, 0.001F}.compute_next_step(Vector{-2.0}, Vector{0.01}, f);
  // We make one step in the needed direction as the maximum step already satisfies the conditions.
  EXPECT_NEAR(max_step, step(0, 0), kRelaxedEpsilon);
}

/// @test       Check that we can maximize a simple inverted parabola with a maximum at 0.
TEST(MoreThuenteLineSearchTest, SimpleQuadraticFunctionMaximization) {
  QuadraticFunction f{-1};  // Inverted quadratic function.
  Matrix jacobian;

  auto max_step{10.0F};
  Vector initial_step{10.0};
  MoreThuenteLineSearch line_search{
    max_step, 0.001F, MoreThuenteLineSearch::OptimizationDirection::kMaximization};
  auto step = line_search.compute_next_step(Vector{-2.0}, initial_step, f);
  // We expect that we will be able to find a proper step in a single call. The max step does not
  // satisfy the strong Wolfe conditions and we will be searching for the proper step. The
  // derivative at the query point is positive here so this should result in maximization instead of
  // minimization.
  EXPECT_NEAR(2.0, step(0, 0), kRelaxedEpsilon);

  step = line_search.compute_next_step(Vector{2.0}, Vector{-1.0}, f);
  // Same as above, but in reverse direction. Also maximization.
  EXPECT_NEAR(-2.0, step(0, 0), kRelaxedEpsilon);

  max_step = 0.1F;
  line_search = MoreThuenteLineSearch{
    max_step, 0.001F, MoreThuenteLineSearch::OptimizationDirection::kMaximization};
  step = line_search.compute_next_step(Vector{-2.0}, Vector{0.01}, f);
  // We make one step in the needed direction as the maximum step already satisfies the conditions.
  EXPECT_NEAR(max_step, step(0, 0), kRelaxedEpsilon);
}

/// @test       Check that we can minimize an offset parabola.
TEST(MoreThuenteLineSearchTest, OffsetQuadraticFunctionMinimization) {
  const Vector offset{2.0};
  QuadraticFunction f{1.0, offset};  // An offset quadratic function.
  Matrix jacobian;

  auto max_step{10.0F};
  auto start{Vector{-2.0}};
  auto initial_step{Vector{1.0}};
  auto step = MoreThuenteLineSearch{max_step, 0.001F}.compute_next_step(start, initial_step, f);
  // We expect that we will be able to find a proper step in a single call. The max step does not
  // satisfy the strong Wolfe conditions and we will be searching for the proper step.
  EXPECT_TRUE((offset - start).isApprox(step, kRelaxedEpsilon));

  start = Vector{3.0};
  initial_step = Vector{-1.0};
  step = MoreThuenteLineSearch{max_step, 0.001F}.compute_next_step(start, initial_step, f);
  // Same as above, but in reverse direction. Still minimization.
  EXPECT_TRUE((offset - start).isApprox(step, kRelaxedEpsilon));

  max_step = 0.1F;
  step = MoreThuenteLineSearch{max_step, 0.001F}.compute_next_step(Vector{-2.0}, Vector{0.01}, f);
  // We make one step in the needed direction as the maximum step already satisfies the conditions.
  EXPECT_NEAR(max_step, step(0, 0), kRelaxedEpsilon);
}

/// @test       Test that when we start closer than the smallest allowed step to the optimum we do
///             nothing.
TEST(MoreThuenteLineSearchTest, DoNothing) {
  QuadraticFunction f;
  Matrix jacobian;

  auto max_step{10.0F};
  auto min_step{0.001F};
  Vector start{-0.5F * min_step};
  Vector initial_step{1.0};
  auto step = MoreThuenteLineSearch{max_step, min_step}.compute_next_step(start, initial_step, f);
  // We expect to do nothing when we are already at the optimum.
  EXPECT_NEAR(0.0, step(0, 0), kRelaxedEpsilon);

  // Do the same for the opposite direction.
  start = Vector{0.5F * min_step};
  initial_step = Vector{-1.0};
  step = MoreThuenteLineSearch{max_step, min_step}.compute_next_step(start, initial_step, f);
  // We expect to do nothing when we are already at the optimum.
  EXPECT_NEAR(0.0, step(0, 0), kRelaxedEpsilon);
}

/// @test       Check that we can minimize a function shown in Figure 3 in the paper.
TEST(MoreThuenteLineSearchTest, FunctionFromPaper) {
  FunctionFromFigure3 f;
  Matrix jacobian;

  const auto max_step{10.0F};
  const auto min_step{0.001F};
  const Vector start{0.0};
  // Note: we *must* start at 0 here as otherwise the function will not correspond to one provided
  // in the paper. We can pick the actual starting point through the choice of max step though.
  const Vector initial_step{1.2};
  const auto mu{0.001F};
  const auto eta{0.001F};
  auto line_search{MoreThuenteLineSearch{
      max_step, min_step, MoreThuenteLineSearch::OptimizationDirection::kMinimization, mu, eta}};
  const auto step = line_search.compute_next_step(start, initial_step, f);
  EXPECT_NEAR(sqrt(2.0), step(0, 0), kRelaxedEpsilon);
}

/// @test       In the Figure 3 of the paper a failure case is presented (see 3rd row in Table 1)
///             where if an initial step already satisfies the strong Wolfe conditions even though
///             the point is far from the optimum. We reproduce this result also here.
TEST(MoreThuenteLineSearchTest, FunctionFromPaperFailure) {
  FunctionFromFigure3 f;
  Matrix jacobian;

  const auto max_step{100.0F};
  const auto min_step{0.001F};
  const Vector start{0.0};
  // Note: we *must* start at 0 here as otherwise the function will not correspond to one provided
  // in the paper. We can pick the actual starting point through the choice of initial_step though.
  const Vector initial_step{10.0};
  const auto mu{0.001F};
  const auto eta{0.1F};
  auto line_search{MoreThuenteLineSearch{
      max_step, min_step, MoreThuenteLineSearch::OptimizationDirection::kMinimization, mu, eta}};
  const auto step = line_search.compute_next_step(start, initial_step, f);
  EXPECT_NEAR(initial_step(0, 0), step(0, 0), kRelaxedEpsilon);
}

/// @test       Check that we cannot initialize the search wrongly.
TEST(MoreThuenteLineSearchTest, WrongInitialization) {
  // Min step must be smaller than the max step.
  EXPECT_THROW(MoreThuenteLineSearch(0.001F, 100.0F), std::domain_error);
  // Negative step sizes.
  EXPECT_THROW(MoreThuenteLineSearch(-1.0F, 100.0F), std::domain_error);
  EXPECT_THROW(MoreThuenteLineSearch(100.0F, -1.0F), std::domain_error);

  // Check that providing valid steps is ok but invalid mu and eta still trigger it.
  EXPECT_NO_THROW(MoreThuenteLineSearch(100.0F, 1.0F));
  auto mu = -1.0F;
  EXPECT_THROW(
    MoreThuenteLineSearch(
      100.0F, 1.0F, MoreThuenteLineSearch::OptimizationDirection::kMinimization, mu),
    std::domain_error);
  mu = 2.0F;
  EXPECT_THROW(
    MoreThuenteLineSearch(
      100.0F, 1.0F, MoreThuenteLineSearch::OptimizationDirection::kMinimization, mu),
    std::domain_error);
  auto eta = -1.0F;
  EXPECT_THROW(
    MoreThuenteLineSearch(
      100.0F, 1.0F, MoreThuenteLineSearch::OptimizationDirection::kMinimization, mu, eta),
    std::domain_error);
  eta = 2.0F;
  EXPECT_THROW(
    MoreThuenteLineSearch(
      100.0F, 1.0F, MoreThuenteLineSearch::OptimizationDirection::kMinimization, mu, eta),
    std::domain_error);

  // Check that number of iterations less than 1 is not allowed.
  EXPECT_NO_THROW(
    MoreThuenteLineSearch(
      100.0F, 1.0F, MoreThuenteLineSearch::OptimizationDirection::kMinimization, 0.1F, 0.1F));
  EXPECT_THROW(
    MoreThuenteLineSearch(
      100.0F, 1.0F, MoreThuenteLineSearch::OptimizationDirection::kMinimization, 0.1F, 0.1F, 0),
    std::domain_error);
}
