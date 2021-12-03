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
#include <gtest/gtest.h>
#include <optimization/utils.hpp>
#include <Eigen/Core>
#include <vector>
#include <limits>
#include "test_cache_states.hpp"

using autoware::common::types::float64_t;

namespace autoware
{
namespace common
{
namespace optimization
{
TEST_F(CacheStateMachineTest, NumericTest) {
  auto eigen_comparator = [](const auto & lhs, const auto & rhs) {
      return lhs.isApprox(rhs, std::numeric_limits<float64_t>::epsilon());
    };

  for (const auto & elem : m_mode_map) {
    // Each element in the map has a term (score, hessian etc.) and all the modes
    // that enable computation of that term
    const auto term = elem.first;
    const auto modes = elem.second;
    for (auto & mode : modes) {
      do_test(mode, 1, 3, term);  // int
      do_test(mode, 1.5F, 2.7F, term);  // float
      do_test(mode, 1.5, 2.7, term);  // double
      do_test(mode, false, true, term);  // bool
      do_test(
        mode, Eigen::Vector3d{1.2, 2.5, 3.6},
        Eigen::Vector3d{7.9, 12.6, 100.9}, term, eigen_comparator);  // eigen vector
    }
  }
}


}  // namespace optimization
}  // namespace common
}  // namespace autoware
