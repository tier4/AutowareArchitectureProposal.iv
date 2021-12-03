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

#include <ndt/ndt_scan.hpp>
#include <gtest/gtest.h>
#include <vector>
#include <algorithm>
#include "test_ndt_scan.hpp"

using autoware::localization::ndt::P2DNDTScan;

TEST_F(NDTScanTest, BadInput) {
  const auto capacity = 5U;
  ASSERT_LT(capacity, m_num_points);
  P2DNDTScan ndt_scan(capacity);
  EXPECT_THROW(ndt_scan.insert(m_pc), std::length_error);
}

TEST_F(NDTScanTest, Basics) {
  ASSERT_EQ(m_pc.width, m_num_points);
  ASSERT_EQ(m_points.size(), m_num_points);

  P2DNDTScan ndt_scan(m_num_points);
  EXPECT_TRUE(ndt_scan.empty());
  EXPECT_NO_THROW(ndt_scan.insert(m_pc));

  EXPECT_EQ(ndt_scan.size(), m_num_points);
  EXPECT_FALSE(ndt_scan.empty());

  // since all points that are used in the form (x,y,z) where x = y = z. We can use a 1D
  // container for easier comparison. This is done to express the fact that the ndt scan does
  // not need to keep the order of its input.
  std::vector<uint32_t> reduced_reference_points;
  for (const auto & pt : m_points) {
    ASSERT_FLOAT_EQ(pt(0U), pt(1U));
    ASSERT_FLOAT_EQ(pt(0U), pt(2U));
    reduced_reference_points.push_back(static_cast<uint32_t>(pt(0U)));
  }

  std::vector<uint32_t> reduced_scan_points;
  for (const auto & pt : ndt_scan) {
    ASSERT_FLOAT_EQ(pt(0U), pt(1U));
    ASSERT_FLOAT_EQ(pt(0U), pt(2U));
    reduced_scan_points.push_back(static_cast<uint32_t>(pt(0U)));
  }

  std::sort(reduced_reference_points.begin(), reduced_reference_points.end());
  std::sort(reduced_scan_points.begin(), reduced_scan_points.end());

  EXPECT_EQ(reduced_scan_points, reduced_reference_points);

  // Adding a new point cloud overwrites the old one.
  EXPECT_NO_THROW(ndt_scan.insert(m_pc));
  EXPECT_EQ(ndt_scan.size(), m_num_points);

  ndt_scan.clear();
  EXPECT_TRUE(ndt_scan.empty());
  EXPECT_EQ(ndt_scan.size(), 0U);
}
