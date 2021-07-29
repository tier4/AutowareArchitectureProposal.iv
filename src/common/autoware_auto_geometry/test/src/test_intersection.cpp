// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <gtest/gtest.h>
#include <geometry/intersection.hpp>
#include <geometry/convex_hull.hpp>
#include <list>

struct TestPoint
{
  autoware::common::types::float32_t x;
  autoware::common::types::float32_t y;
};

struct IntersectionTestParams
{
  std::list<TestPoint> polygon1_pts;
  std::list<TestPoint> polygon2_pts;
  std::list<TestPoint> expected_intersection_pts;
};

void order_ccw(std::list<TestPoint> & points)
{
  const auto end_it = autoware::common::geometry::convex_hull(points);
  ASSERT_EQ(end_it, points.end());  // Points should have represent a shape
}

class IntersectionTest : public ::testing::TestWithParam<IntersectionTestParams>
{
};


TEST_P(IntersectionTest, basic) {
  const auto get_ordered_polygon = [](auto polygon) {
      order_ccw(polygon);
      return polygon;
    };

  const auto polygon1 = get_ordered_polygon(GetParam().polygon1_pts);
  const auto polygon2 = get_ordered_polygon(GetParam().polygon2_pts);
  const auto expected_intersection = get_ordered_polygon(GetParam().expected_intersection_pts);

  const auto result =
    autoware::common::geometry::convex_polygon_intersection2d(polygon1, polygon2);

  ASSERT_EQ(result.size(), expected_intersection.size());
  auto expected_shape_it = expected_intersection.begin();
  for (auto result_it = result.begin(); result_it != result.end(); ++result_it) {
    EXPECT_FLOAT_EQ(result_it->x, expected_shape_it->x);
    EXPECT_FLOAT_EQ(result_it->y, expected_shape_it->y);
    ++expected_shape_it;
  }
}

INSTANTIATE_TEST_CASE_P(
  basic, IntersectionTest,
  ::testing::Values(
    IntersectionTestParams{
  {},
  {},
  {}
},
    IntersectionTestParams{      // Partial intersection
  {{0.0F, 0.0F}, {10.0F, 0.0F}, {0.0F, 10.0F}, {10.0F, 10.0F}},
  {{5.0F, 5.0F}, {15.0F, 5.0F}, {5.0F, 15.0F}, {15.0F, 15.0F}},
  {{5.0F, 5.0F}, {10.0F, 5.0F}, {5.0F, 10.0F}, {10.0F, 10.0F}}
},
    // Full intersection with overlapping edges
    // TODO(yunus.caliskan): enable after #1231
//        IntersectionTestParams{
//            {{0.0F, 0.0F}, {10.0F, 0.0F}, {0.0F, 10.0F}, {10.0F, 10.0F}},
//            {{5.0F, 5.0F}, {10.0F, 5.0F}, {5.0F, 10.0F}, {10.0F, 10.0F}},
//            {{5.0F, 5.0F}, {10.0F, 5.0F}, {5.0F, 10.0F}, {10.0F, 10.0F}},
//        },
    IntersectionTestParams{      // Fully contained
  {{0.0F, 0.0F}, {10.0F, 0.0F}, {0.0F, 10.0F}, {10.0F, 10.0F}},
  {{5.0F, 5.0F}, {6.0F, 5.0F}, {5.0F, 7.0F}, {8.0F, 8.0F}},
  {{5.0F, 5.0F}, {6.0F, 5.0F}, {5.0F, 7.0F}, {8.0F, 8.0F}},
},
    IntersectionTestParams{      // Fully contained triangle
  {{0.0F, 0.0F}, {10.0F, 0.0F}, {0.0F, 10.0F}, {10.0F, 10.0F}},
  {{5.0F, 5.0F}, {6.0F, 5.0F}, {5.0F, 7.0F}},
  {{5.0F, 5.0F}, {6.0F, 5.0F}, {5.0F, 7.0F}},
},
    IntersectionTestParams{      // Triangle rectangle intersection.
  {{0.0F, 0.0F}, {10.0F, 0.0F}, {0.0F, 10.0F}, {10.0F, 10.0F}},
  {{5.0F, 1.0F}, {5.0F, 9.0F}, {15.0F, 5.0F}},
  {{5.0F, 1.0F}, {5.0F, 9.0F}, {10.0F, 3.0F}, {10.0F, 7.0F}}
},
    IntersectionTestParams{      // No intersection
  {{0.0F, 0.0F}, {10.0F, 0.0F}, {0.0F, 10.0F}, {10.0F, 10.0F}},
  {{15.0F, 15.0F}, {20.0F, 15.0F}, {15.0F, 20.0F}, {20.0F, 20.0F}},
  {}
}
    // cppcheck-suppress syntaxError
  ), );

TEST(PolygonPointTest, basic) {
  GTEST_SKIP();  // TODO(yunus.caliskan): enable after #1231
  std::list<TestPoint> polygon{{5.0F, 5.0F}, {10.0F, 5.0F}, {5.0F, 10.0F}, {10.0F, 10.0F}};
  order_ccw(polygon);
  EXPECT_FALSE(
    autoware::common::geometry::is_point_inside_polygon_2d(
      polygon.begin(), polygon.end(), TestPoint{0.0F, 10.0F}));
}
