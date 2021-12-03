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

#ifndef TEST_CACHE_STATES_HPP_
#define TEST_CACHE_STATES_HPP_

#include <gtest/gtest.h>
#include <optimization/utils.hpp>
#include <functional>
#include <vector>
#include <map>

namespace autoware
{
namespace common
{
namespace optimization
{
template<typename ValT, typename CompareFuncT = decltype(std::equal_to<ValT>())>
void do_test(
  const ComputeMode & mode, ValT val1, ValT val2, ExpressionTerm term,
  CompareFuncT comparator = CompareFuncT())
{
  auto get_an_unset_term = [](const ComputeMode & mode) {
      if (!mode.score()) {
        return ExpressionTerm::SCORE;
      } else if (!mode.jacobian()) {
        return ExpressionTerm::JACOBIAN;
      } else if (!mode.hessian()) {
        return ExpressionTerm::HESSIAN;
      } else {
        // Make sure there is an unset term before calling this function.
        EXPECT_TRUE(false);
        return ExpressionTerm::SCORE;  // Dummy value for completeness
      }
    };
  // Make sure val1 and val2 are different
  ASSERT_FALSE(comparator(val1, val2));

  CacheStateMachine<ValT, CompareFuncT> csm{comparator};
  csm.update(val1, mode);
  EXPECT_TRUE(csm.is_cached(val1, term));
  EXPECT_FALSE(csm.is_cached(val2, term));

  if (mode != ComputeMode{}.set_score().set_jacobian().set_hessian()) {
    const auto unset_term = get_an_unset_term(mode);
    EXPECT_FALSE(csm.is_cached(val1, unset_term));
    EXPECT_FALSE(csm.is_cached(val2, unset_term));
  }
}

class CacheStateMachineTest : public ::testing::Test
{
public:
  CacheStateMachineTest()
  {
    m_mode_map.emplace(
      ExpressionTerm::SCORE,
      std::vector<ComputeMode>{
            ComputeMode{}.set_score(),
            ComputeMode{}.set_score().set_jacobian(),
            ComputeMode{}.set_score().set_hessian(),
            ComputeMode{}.set_score().set_jacobian().set_hessian(),
          });
    m_mode_map.emplace(
      ExpressionTerm::JACOBIAN,
      std::vector<ComputeMode>{
            ComputeMode{}.set_jacobian(),
            ComputeMode{}.set_jacobian().set_score(),
            ComputeMode{}.set_jacobian().set_hessian(),
            ComputeMode{}.set_jacobian().set_score().set_hessian(),
          });

    m_mode_map.emplace(
      ExpressionTerm::HESSIAN,
      std::vector<ComputeMode>{
            ComputeMode{}.set_hessian(),
            ComputeMode{}.set_hessian().set_score(),
            ComputeMode{}.set_hessian().set_jacobian(),
            ComputeMode{}.set_hessian().set_jacobian().set_score(),
          });
  }

protected:
  void SetUp()
  {
    for (const auto & mode : m_mode_map[ExpressionTerm::SCORE]) {
      ASSERT_TRUE(mode.score());
    }
    for (const auto & mode : m_mode_map[ExpressionTerm::JACOBIAN]) {
      ASSERT_TRUE(mode.jacobian());
    }
    for (const auto & mode : m_mode_map[ExpressionTerm::HESSIAN]) {
      ASSERT_TRUE(mode.hessian());
    }
  }
  std::map<ExpressionTerm, std::vector<ComputeMode>> m_mode_map;
};

}  // namespace optimization
}  // namespace common
}  // namespace autoware


#endif  // TEST_CACHE_STATES_HPP_
