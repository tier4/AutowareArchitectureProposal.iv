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

#ifndef LOCALIZATION_COMMON__OPTIMIZED_REGISTRATION_SUMMARY_HPP_
#define LOCALIZATION_COMMON__OPTIMIZED_REGISTRATION_SUMMARY_HPP_

#include <localization_common/visibility_control.hpp>
#include <optimization/optimizer_options.hpp>
#include <experimental/optional>

namespace autoware
{
namespace localization
{
namespace localization_common
{
/// Basic Registration Summary for localizers using an optimizer.
/// It only wraps the optimization summary of the optimizer.
class LOCALIZATION_COMMON_PUBLIC OptimizedRegistrationSummary
{
public:
  using OptimizationSummary = common::optimization::OptimizationSummary;
  explicit OptimizedRegistrationSummary(const OptimizationSummary & opt_summary);
  OptimizedRegistrationSummary();

  /// Get optimization summary.
  OptimizationSummary optimization_summary() const;

private:
  OptimizationSummary m_optimization_summary;
};
}  // namespace localization_common
}  // namespace localization
}  // namespace autoware

#endif  // LOCALIZATION_COMMON__OPTIMIZED_REGISTRATION_SUMMARY_HPP_
