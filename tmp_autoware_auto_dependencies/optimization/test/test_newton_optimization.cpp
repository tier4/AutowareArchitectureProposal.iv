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

#include "test_newton_optimization.hpp"

#include <optimization/line_search/fixed_line_search.hpp>

#include <common/types.hpp>
#include <gtest/gtest.h>
#include <limits>

using autoware::common::types::float64_t;
using autoware::common::optimization::Polynomial1dParam;
using autoware::common::optimization::OptimizationOptions;
using autoware::common::optimization::FixedLineSearch;
using autoware::common::optimization::NewtonsMethodOptimizer;
using autoware::common::optimization::TerminationType;
using autoware::common::optimization::Polynomial1DOptimizationProblem;
using autoware::common::optimization::Vector1D;

using NewtonPolynomialTestParam1D = Polynomial1dParam<OptimizationOptions, FixedLineSearch>;
class NewtonOptimizationParamTest
  : public ::testing::TestWithParam<NewtonPolynomialTestParam1D> {};

TEST_P(NewtonOptimizationParamTest, NewtonOptimizationValidation) {
  auto problem = GetParam().problem;
  auto objective = problem.get_objective();
  const auto x0 = GetParam().x0;
  const auto options = GetParam().options;
  const auto line_search = GetParam().line_searcher;
  const auto expected_termination = GetParam().expected_termination;
  const auto solution = objective.solution;
  const auto is_convex = objective.convex;

  NewtonsMethodOptimizer<FixedLineSearch> optimizer{line_search, options};

  decltype(problem)::DomainValue x_out;
  const auto summary = optimizer.solve(problem, x0, x_out);
  EXPECT_EQ(summary.termination_type(), expected_termination);
  if (is_convex && (expected_termination == TerminationType::CONVERGENCE)) {
    EXPECT_FLOAT_EQ(x_out(0, 0), solution);
    EXPECT_LE(summary.estimated_distance_to_optimum(), line_search.get_step_max());
  }
}

INSTANTIATE_TEST_SUITE_P(
  TestTermination,
  NewtonOptimizationParamTest,
  ::testing::Values(
    NewtonPolynomialTestParam1D{
  Polynomial1DOptimizationProblem{1.0, 2, 1.0},  // (x+1)^2+1
  Vector1D{3.0},  // x0 = 3
  OptimizationOptions(30, 4e-2, 0.0, 0.0),  // Square of the step size threshold.
  FixedLineSearch(0.2),
  TerminationType::CONVERGENCE
},
    NewtonPolynomialTestParam1D{
  Polynomial1DOptimizationProblem{1.0, 2, 1.0},  // (x+1)^2+1
  Vector1D{3.0},  // x0 = 3
  OptimizationOptions(30, 0.0, 1e-5, 0.0),
  FixedLineSearch(0.2),
  TerminationType::NO_CONVERGENCE  // Can't converge because 0.2 > 1e-5
},
    NewtonPolynomialTestParam1D{
  Polynomial1DOptimizationProblem{1.0, 2, 1.0},  // (x+1)^2+1
  Vector1D{3.0},  // x0 = 3
  OptimizationOptions(30, 0.0, 0.0, 1e-4),
  FixedLineSearch(0.2),
  TerminationType::CONVERGENCE
}
    // cppcheck-suppress syntaxError
  ),
);

// since the convex polynomial objective will have a zero gradient solution,
// Using a gradient based tolerance will have less variance to the step size than
// the function tolerance which depends on the step size. Parameter tolerance is useless for
// fixed step search. Hence gradient tolerance is used for different cases.
INSTANTIATE_TEST_SUITE_P(
  TestDifferentProblems,
  NewtonOptimizationParamTest,
  ::testing::Values(
    // N = 2
    NewtonPolynomialTestParam1D{
  Polynomial1DOptimizationProblem{0.0, 2, -11.0},
  Vector1D{0.0},
  OptimizationOptions(10, 0.0, 0.0, 1e-4),
  FixedLineSearch(0.5),
  TerminationType::CONVERGENCE
},
    NewtonPolynomialTestParam1D{
  Polynomial1DOptimizationProblem{-1.0, 2, 1.0},
  Vector1D{0.0},
  OptimizationOptions(10, 0.0, 0.0, 1e-4),
  FixedLineSearch(0.5),
  TerminationType::CONVERGENCE
},
    NewtonPolynomialTestParam1D{
  Polynomial1DOptimizationProblem{2.0, 2, 100.0},
  Vector1D{2},
  OptimizationOptions(10, 0.0, 0.0, 1e-4),
  FixedLineSearch(0.5),
  TerminationType::CONVERGENCE
},
    NewtonPolynomialTestParam1D{
  Polynomial1DOptimizationProblem{1.0, 2, 1.0},
  Vector1D{100.0},
  OptimizationOptions(10, 0.0, 0.0, 1e-4),
  FixedLineSearch(0.5),
  TerminationType::NO_CONVERGENCE
},
    NewtonPolynomialTestParam1D{
  Polynomial1DOptimizationProblem{1.0, 2, 1.0},
  Vector1D{2.0},
  OptimizationOptions(10, 0.0, 0.0, 1e-4),
  FixedLineSearch(0.0001),  // won't converge because step size is too small
  TerminationType::NO_CONVERGENCE
},
    // Higher order N
    NewtonPolynomialTestParam1D{
  Polynomial1DOptimizationProblem{-2.0, 4, 10.0},
  Vector1D{0.0},
  OptimizationOptions(10, 0.0, 0.0, 1e-4),
  FixedLineSearch(0.5),
  TerminationType::CONVERGENCE
},
    NewtonPolynomialTestParam1D{
  Polynomial1DOptimizationProblem{-2.0, 16, 10.0},
  Vector1D{0.0},
  OptimizationOptions(10, 0.0, 0.0, 1e-4),
  FixedLineSearch(0.5),
  TerminationType::CONVERGENCE
}
  ),
);

constexpr auto inf = std::numeric_limits<float64_t>::infinity();
constexpr auto max = std::numeric_limits<float64_t>::max();
constexpr auto qnan = std::numeric_limits<float64_t>::quiet_NaN();

INSTANTIATE_TEST_SUITE_P(
  TestNumericFailure,
  NewtonOptimizationParamTest,
  ::testing::Values(
    NewtonPolynomialTestParam1D{
  Polynomial1DOptimizationProblem{-1.0, 2, 1.0},
  Vector1D{inf},
  OptimizationOptions(10, 0.0, 0.0, 1e-4),
  FixedLineSearch(0.5),
  TerminationType::FAILURE
},
    NewtonPolynomialTestParam1D{
  Polynomial1DOptimizationProblem{-1.0, 2, 1.0},
  Vector1D{max},
  OptimizationOptions(10, 0.0, 0.0, 1e-4),
  FixedLineSearch(0.5),
  TerminationType::FAILURE
},
    NewtonPolynomialTestParam1D{
  Polynomial1DOptimizationProblem{-1.0, 2, 1.0},
  Vector1D{qnan},
  OptimizationOptions(10, 0.0, 0.0, 1e-4),
  FixedLineSearch(0.5),
  TerminationType::FAILURE
},
    NewtonPolynomialTestParam1D{
  Polynomial1DOptimizationProblem{-1.0, 2, 1.0},
  Vector1D{0.0},
  OptimizationOptions(10, 0.0, 0.0, 1e-4),
  FixedLineSearch(inf),
  TerminationType::FAILURE
},
    NewtonPolynomialTestParam1D{
  Polynomial1DOptimizationProblem{-1.0, 2, 1.0},
  Vector1D{0.0},
  OptimizationOptions(10, 0.0, 0.0, 1e-4),
  FixedLineSearch(qnan),
  TerminationType::FAILURE
},
    NewtonPolynomialTestParam1D{
  Polynomial1DOptimizationProblem{-1.0, 2, 1.0},
  Vector1D{0.0},
  OptimizationOptions(10, 0.0, 0.0, 1e-4),
  FixedLineSearch(max),
  TerminationType::FAILURE
}
  ),
);

TEST(TestFixedLineSearch, FixedLineSearchValidation) {
  // set up varaibles
  constexpr auto step = 0.01F;
  Polynomial1DOptimizationProblem dummy_optimization_problem{-2.0, 16, 10.0};


  // test derived class
  FixedLineSearch fls;
  EXPECT_FLOAT_EQ(fls.get_step_max(), std::numeric_limits<float_t>::min());
  fls.set_step_max(step);
  // TODO(yunus.caliskan): enable in #308
  //  EXPECT_FLOAT_EQ(fls.compute_next_step(dummy_optimization_problem), step);
}
