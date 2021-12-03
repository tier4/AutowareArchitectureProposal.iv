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

#include <localization_common/optimized_registration_summary.hpp>
#include <limits>

namespace autoware
{
namespace localization
{
namespace localization_common
{

OptimizedRegistrationSummary::OptimizedRegistrationSummary(const OptimizationSummary & opt_summary)
: m_optimization_summary{opt_summary} {}

OptimizedRegistrationSummary::OptimizedRegistrationSummary()
: m_optimization_summary{OptimizationSummary{std::numeric_limits<float64_t>::max(),
      common::optimization::TerminationType::NO_CONVERGENCE,
      0}} {}

OptimizedRegistrationSummary::OptimizationSummary
OptimizedRegistrationSummary::optimization_summary() const
{
  return m_optimization_summary;
}

}  // namespace localization_common
}  // namespace localization
}  // namespace autoware
