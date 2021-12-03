// Copyright 2017-2019 the Autoware Foundation
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

#include <gtest/gtest.h>
#include "common/types.hpp"
#include "lidar_utils/lidar_utils.hpp"

using autoware::common::types::float32_t;

constexpr float32_t FAST_ATAN2_MAX_ERROR = 0.00469f;

TEST(FastAtan2, CornerCases) {
  ASSERT_TRUE(
    fabsf(
      autoware::common::lidar_utils::fast_atan2(0.0f, 0.0f) -
      atan2f(0.0f, 0.0f)) < FAST_ATAN2_MAX_ERROR);
  ASSERT_TRUE(
    fabsf(
      autoware::common::lidar_utils::fast_atan2(1.0f, 0.0f) -
      atan2f(1.0f, 0.0f)) < FAST_ATAN2_MAX_ERROR);
  ASSERT_TRUE(
    fabsf(
      autoware::common::lidar_utils::fast_atan2(-1.0f, 0.0f) -
      atan2f(-1.0f, 0.0f)) < FAST_ATAN2_MAX_ERROR);
  ASSERT_TRUE(
    fabsf(
      autoware::common::lidar_utils::fast_atan2(0.0f, 1.0f) -
      atan2f(0.0f, 1.0f)) < FAST_ATAN2_MAX_ERROR);
  ASSERT_TRUE(
    fabsf(
      autoware::common::lidar_utils::fast_atan2(0.0f, -1.0f) -
      atan2f(0.0f, -1.0f)) < FAST_ATAN2_MAX_ERROR);
}

TEST(FastAtan2, MaxError) {
  float32_t max_error = 0;
  for (float32_t f = 0; f < autoware::common::types::TAU; f += 0.00001f) {
    float32_t x = std::cos(f);
    float32_t y = std::sin(f);
    max_error = std::max(
      max_error,
      fabsf(atan2f(y, x) - autoware::common::lidar_utils::fast_atan2(y, x)));
  }
  ASSERT_TRUE(max_error < FAST_ATAN2_MAX_ERROR);
}
