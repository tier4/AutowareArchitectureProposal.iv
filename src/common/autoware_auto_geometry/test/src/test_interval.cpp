// Copyright 2020 Mapless AI, Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "geometry/interval.hpp"

using autoware::common::geometry::Interval_d;
using autoware::common::geometry::Interval_f;
using autoware::common::geometry::Interval;

namespace
{
const auto Inf = std::numeric_limits<double>::infinity();
const auto Min = std::numeric_limits<double>::lowest();
const auto Max = std::numeric_limits<double>::max();
const auto NaN = std::numeric_limits<double>::quiet_NaN();
const auto epsilon = 1e-5;
}  // namespace

//------------------------------------------------------------------------------

TEST(Geometry_Interval, abs_eq) {
  const auto i1 = Interval_d(-1.0, 1.0);
  const auto i2 = Interval_d(-1.0 + 0.5 * epsilon, 1.0 + 0.5 * epsilon);
  const auto shift = (2.0 * epsilon);
  const auto i3 = Interval_d(-1.0 + shift, 1.0 + shift);
  const auto i_empty = Interval_d();

  EXPECT_TRUE(Interval_d::abs_eq(i1, i1, epsilon));
  EXPECT_TRUE(Interval_d::abs_eq(i1, i2, epsilon));
  EXPECT_TRUE(Interval_d::abs_eq(i2, i1, epsilon));
  EXPECT_FALSE(Interval_d::abs_eq(i1, i3, epsilon));
  EXPECT_FALSE(Interval_d::abs_eq(i3, i1, epsilon));
  EXPECT_FALSE(Interval_d::abs_eq(i1, i_empty, epsilon));
  EXPECT_FALSE(Interval_d::abs_eq(i_empty, i1, epsilon));
  EXPECT_TRUE(Interval_d::abs_eq(i_empty, i_empty, epsilon));
}

//------------------------------------------------------------------------------

TEST(Geometry_Interval, is_subset_eq) {
  EXPECT_TRUE(Interval_d::is_subset_eq(Interval_d(-0.5, 0.5),
    Interval_d(-1.0, 1.0)));
  EXPECT_TRUE(Interval_d::is_subset_eq(Interval_d(3.2, 4.2),
    Interval_d(3.2, 4.2)));
  EXPECT_FALSE(Interval_d::is_subset_eq(Interval_d(-3.0, -1.0),
    Interval_d(1.0, 3.0)));
  EXPECT_FALSE(Interval_d::is_subset_eq(Interval_d(1.0, 3.0),
    Interval_d(2.0, 4.0)));
  EXPECT_FALSE(
    Interval_d::is_subset_eq(Interval_d(), Interval_d()));
}

//------------------------------------------------------------------------------

TEST(Geometry_Interval, clamp_to) {
  const auto i = Interval_d(-1.0, 1.0);
  {
    const auto val = 0.0;
    const auto p = Interval_d::clamp_to(i, val);
    EXPECT_EQ(p, val);
  }

  {
    const auto val = -3.4;
    const auto p = Interval_d::clamp_to(i, val);
    EXPECT_EQ(p, Interval_d::min(i));
  }

  {
    const auto val = 2.7;
    const auto p = Interval_d::clamp_to(i, val);
    EXPECT_EQ(p, Interval_d::max(i));
  }

  const auto val = 1.0;
  const auto empty_interval = Interval_d();
  const auto projected = Interval_d::clamp_to(empty_interval, val);
  EXPECT_TRUE(std::isnan(projected));
}

//------------------------------------------------------------------------------

TEST(Geometry_Interval, comparisons) {
  {
    const auto i1 = Interval_d(0.25, 1);
    const auto i2 = Interval_d(0, 1);
    EXPECT_FALSE((i1 == i2));
    EXPECT_TRUE((i1 != i2));
  }

  {
    const auto i1 = Interval_d(-0.25, 0.5);
    const auto i2 = Interval_d(0, 1);
    EXPECT_FALSE((i1 == i2));
    EXPECT_TRUE((i1 != i2));
  }

  {
    const auto i1 = Interval_d(0, 0.5);
    const auto i2 = Interval_d(0, 1);
    EXPECT_FALSE((i1 == i2));
    EXPECT_TRUE((i1 != i2));
  }

  {
    const auto i1 = Interval_d(0, 1);
    const auto i2 = Interval_d(0, 1);
    EXPECT_TRUE((i1 == i2));
    EXPECT_FALSE((i1 != i2));
  }

  {
    const auto i1 = Interval_d(0, 1);
    const auto i2 = Interval_d();
    EXPECT_FALSE((i1 == i2));
    EXPECT_TRUE((i1 != i2));
  }

  {
    const auto i1 = Interval_d();
    const auto i2 = Interval_d();
    EXPECT_TRUE((i1 == i2));
    EXPECT_FALSE((i1 != i2));
  }
}

//------------------------------------------------------------------------------

TEST(Geometry_Interval, contains) {
  {
    const auto i = Interval_d();
    EXPECT_FALSE(Interval_d::contains(i, 0.0));
  }

  {
    const auto i = Interval_d(-1.0, 1.0);
    EXPECT_TRUE(Interval_d::contains(i, 0.0));
    EXPECT_FALSE(Interval_d::contains(i, 2.0));
  }
}

//------------------------------------------------------------------------------

TEST(Geometry_Interval, empty) {
  {
    const auto i1 = Interval_d();
    const auto i2 = Interval_d();
    const auto i3 = Interval_d::intersect(i1, i2);
    EXPECT_TRUE(Interval_d::empty(i3));
  }

  {
    const auto i1 = Interval_d();
    const auto i2 = Interval_d(0.0, 1.0);
    const auto i3 = Interval_d::intersect(i1, i2);
    EXPECT_TRUE(Interval_d::empty(i3));
  }
}

//------------------------------------------------------------------------------

TEST(Geometry_Interval, zero_measure) {
  {
    const auto i = Interval_d(0, 1);
    EXPECT_FALSE(Interval_d::zero_measure(i));
  }

  {
    const auto i = Interval_d();
    EXPECT_FALSE(Interval_d::zero_measure(i));
  }

  {
    const auto i = Interval_d(2, 2);
    EXPECT_TRUE(Interval_d::zero_measure(i));
  }
}

//------------------------------------------------------------------------------

TEST(Geometry_Interval, intersection_measure) {
  {
    const auto i1 = Interval_d(-1.0, 1.0);
    const auto i2 = Interval_d(-0.5, 1.5);
    const auto i = Interval_d::intersect(i1, i2);
    EXPECT_EQ(Interval_d::min(i), -0.5);
    EXPECT_EQ(Interval_d::max(i), 1.0);
    EXPECT_NEAR(Interval_d::measure(i), 1.5, epsilon);
  }

  {
    const auto i1 = Interval_d(-2.0, -1.0);
    const auto i2 = Interval_d(1.0, 2.0);
    const auto i = Interval_d::intersect(i1, i2);
    EXPECT_TRUE(Interval_d::empty(i));
    EXPECT_TRUE(std::isnan(Interval_d::min(i)));
    EXPECT_TRUE(std::isnan(Interval_d::max(i)));
    EXPECT_TRUE(std::isnan(Interval_d::measure(i)));
  }
}

//------------------------------------------------------------------------------

TEST(Geometry_Interval, construction_measure) {
  {
    const auto i = Interval_d();
    EXPECT_TRUE(std::isnan(Interval_d::min(i)));
    EXPECT_TRUE(std::isnan(Interval_d::max(i)));
    EXPECT_TRUE(std::isnan(Interval_d::measure(i)));
  }

  {
    const auto i = Interval_d(-1.0, 1.0);
    EXPECT_EQ(Interval_d::min(i), -1.0);
    EXPECT_EQ(Interval_d::max(i), 1.0);
    EXPECT_NEAR(Interval_d::measure(i), 2.0, epsilon);
  }

  {
    const auto i = Interval_d(0.0, 0.0);
    EXPECT_EQ(Interval_d::min(i), 0.0);
    EXPECT_EQ(Interval_d::max(i), 0.0);
    EXPECT_FALSE(Interval_d::empty(i));
    EXPECT_EQ(Interval_d::measure(i), 0.0);
  }

  {
    EXPECT_THROW({Interval_d(1.0, -1.0);}, std::runtime_error);
  }
}

//------------------------------------------------------------------------------

TEST(Geometry_Interval, interpolate) {
  const Interval_f unit_interval(0.0f, 1.0f);
  const Interval_f offset_interval(2.0f, 3.5f);
  const Interval_f empty_interval;

  {
    const auto scaling = 0.0f;
    EXPECT_EQ(Interval_f::interpolate(unit_interval, scaling), Interval_f::min(unit_interval));
    EXPECT_EQ(Interval_f::interpolate(offset_interval, scaling), Interval_f::min(offset_interval));
    EXPECT_TRUE(std::isnan(Interval_f::interpolate(empty_interval, scaling)));
  }

  {
    const auto scaling = 1.0f;
    EXPECT_EQ(Interval_f::interpolate(unit_interval, scaling), Interval_f::max(unit_interval));
    EXPECT_EQ(Interval_f::interpolate(offset_interval, scaling), Interval_f::max(offset_interval));
    EXPECT_TRUE(std::isnan(Interval_f::interpolate(empty_interval, scaling)));
  }

  {
    const auto scaling = -1.0f;
    EXPECT_EQ(Interval_f::interpolate(unit_interval, scaling), Interval_f::min(unit_interval));
    EXPECT_EQ(Interval_f::interpolate(offset_interval, scaling), Interval_f::min(offset_interval));
    EXPECT_TRUE(std::isnan(Interval_f::interpolate(empty_interval, scaling)));
  }

  {
    const auto scaling = 2.0f;
    EXPECT_EQ(Interval_f::interpolate(unit_interval, scaling), Interval_f::max(unit_interval));
    EXPECT_EQ(Interval_f::interpolate(offset_interval, scaling), Interval_f::max(offset_interval));
    EXPECT_TRUE(std::isnan(Interval_f::interpolate(empty_interval, scaling)));
  }

  {
    const auto scaling = 0.75f;
    EXPECT_EQ(Interval_f::interpolate(unit_interval, scaling), 0.75f);
    EXPECT_EQ(Interval_f::interpolate(offset_interval, scaling), 3.125f);
    EXPECT_TRUE(std::isnan(Interval_f::interpolate(empty_interval, scaling)));
  }
}

//------------------------------------------------------------------------------
