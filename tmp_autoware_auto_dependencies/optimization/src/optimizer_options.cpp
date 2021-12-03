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

#include <common/types.hpp>
#include <optimization/optimizer_options.hpp>
#include <stdexcept>
#include <cmath>

using autoware::common::types::float64_t;

namespace autoware
{
namespace common
{
namespace optimization
{
OptimizationOptions::OptimizationOptions(
  uint64_t max_num_iterations,
  float64_t function_tolerance,
  float64_t parameter_tolerance,
  float64_t gradient_tolerance)
: m_max_num_iterations(max_num_iterations),
  m_function_tolerance(function_tolerance), m_parameter_tolerance(parameter_tolerance),
  m_gradient_tolerance(gradient_tolerance)
{
  if (!std::isfinite(m_function_tolerance) ||
    !std::isfinite(m_parameter_tolerance) ||
    !std::isfinite(m_gradient_tolerance))
  {
    throw std::domain_error("OptimizationOptions: Tolerance values must be finite.");
  }

  if ((m_function_tolerance <= 0.0) &&
    (m_parameter_tolerance <= 0.0) &&
    (m_gradient_tolerance <= 0.0))
  {
    throw std::domain_error(
            "OptimizationOptions: "
            "There should at least be one positive tolerance value "
            "for convergence.");
  }
}

uint64_t OptimizationOptions::max_num_iterations() const noexcept
{
  return m_max_num_iterations;
}
float64_t OptimizationOptions::function_tolerance() const noexcept
{
  return m_function_tolerance;
}
float64_t OptimizationOptions::parameter_tolerance() const noexcept
{
  return m_parameter_tolerance;
}
float64_t OptimizationOptions::gradient_tolerance() const noexcept
{
  return m_gradient_tolerance;
}

OptimizationSummary::OptimizationSummary(
  float64_t dist, TerminationType termination_type,
  uint64_t iter)
: m_estimated_distance_to_optimum(dist),
  m_number_of_iterations_made(iter),
  m_termination_type(termination_type)
{}

float64_t OptimizationSummary::estimated_distance_to_optimum() const noexcept
{
  return m_estimated_distance_to_optimum;
}
TerminationType OptimizationSummary::termination_type() const noexcept
{
  return m_termination_type;
}
uint64_t OptimizationSummary::number_of_iterations_made() const noexcept
{
  return m_number_of_iterations_made;
}
}  // namespace optimization
}  // namespace common
}  // namespace autoware
