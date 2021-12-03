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

#ifndef TEST_NDT_SCAN_HPP_
#define TEST_NDT_SCAN_HPP_
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <vector>
#include "test_ndt_map.hpp"
#include "common/types.hpp"

using autoware::common::types::float64_t;

class NDTScanTest : public ::testing::Test
{
public:
  using Point = Eigen::Vector3d;
  const uint32_t m_num_points{10U};

  NDTScanTest()
  {
    for (auto i = 0U; i < m_num_points; i++) {
      m_points.emplace_back(
        static_cast<float64_t>(i),
        static_cast<float64_t>(i),
        static_cast<float64_t>(i));
    }
    m_pc = make_pcl(m_points);
  }

protected:
  std::vector<Point> m_points;
  sensor_msgs::msg::PointCloud2 m_pc;
};

#endif  // TEST_NDT_SCAN_HPP_
