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

#ifndef TEST_VOXEL_GRID_HPP_
#define TEST_VOXEL_GRID_HPP_

#include <common/types.hpp>
#include <memory>
#include <limits>
#include "voxel_grid/voxel_grid.hpp"

using autoware::perception::filters::voxel_grid::PointXYZ;
using autoware::perception::filters::voxel_grid::Config;
using autoware::perception::filters::voxel_grid::Voxel;
using autoware::perception::filters::voxel_grid::ApproximateVoxel;
using autoware::perception::filters::voxel_grid::CentroidVoxel;
using autoware::perception::filters::voxel_grid::VoxelGrid;
using autoware::perception::filters::voxel_grid::PointXYZIF;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

/// Hold some basic configuration parameters
class VoxelTest : public ::testing::Test
{
public:
  VoxelTest()
  {
    // simple 8 index voxel grid:
    min_point.x = -1.0F;
    min_point.y = -1.0F;
    min_point.z = -1.0F;
    max_point.x = 1.0F;
    max_point.y = 1.0F;
    max_point.z = 1.0F;
    voxel_size.x = 1.0F;
    voxel_size.y = 1.0F;
    voxel_size.z = 1.0F;
    capacity = 7U;
    cfg_ptr = std::make_unique<Config>(min_point, max_point, voxel_size, capacity);
    // Note: dep due to API changes, but I'd like to keep the info aorund
    // ASSERT_EQ(cfg.y_stride, 2U);
    // ASSERT_EQ(cfg.z_stride, 4U);
  }

protected:
  PointXYZ min_point;
  PointXYZ max_point;
  PointXYZ voxel_size;
  uint64_t capacity;
  std::unique_ptr<Config> cfg_ptr;
};

/// Typed for various point types
template<typename PointT>
class TypedVoxelTest : public VoxelTest
{
public:
  using VoxelTest::VoxelTest;

protected:
  PointT make(const float32_t x, const float32_t y, const float32_t z) const
  {
    PointT pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    return pt;
  }
};

// Instantiate tests for given types, add more types here as they are used
using PointTypes = ::testing::Types<PointXYZ, PointXYZIF>;
// cppcheck-suppress syntaxError
TYPED_TEST_SUITE(TypedVoxelTest, PointTypes, );
/// NOTE: This is the older version due to 1.8.0 of GTest. v1.8.1 uses TYPED_TEST_SUITE

/// Note on memory tests: They assume instantiating the given point type does not allocate heap
/// memory. This is pretty reasonable, and if you're doing this, you need to take a good long look
/// at what you're doing.


/// \brief Minimal testing of getter API
TEST_F(VoxelTest, Basic)
{
  //// Getter API ////
  // Min point
  EXPECT_FLOAT_EQ(cfg_ptr->get_min_point().x, min_point.x);
  EXPECT_FLOAT_EQ(cfg_ptr->get_min_point().y, min_point.y);
  EXPECT_FLOAT_EQ(cfg_ptr->get_min_point().z, min_point.z);
  // Max point
  EXPECT_FLOAT_EQ(
    cfg_ptr->get_max_point().x,
    max_point.x - std::numeric_limits<float32_t>::epsilon());
  EXPECT_FLOAT_EQ(
    cfg_ptr->get_max_point().y,
    max_point.y - std::numeric_limits<float32_t>::epsilon());
  EXPECT_FLOAT_EQ(
    cfg_ptr->get_max_point().z,
    max_point.z - std::numeric_limits<float32_t>::epsilon());
  // Voxel Size
  EXPECT_FLOAT_EQ(cfg_ptr->get_voxel_size().x, voxel_size.x);
  EXPECT_FLOAT_EQ(cfg_ptr->get_voxel_size().y, voxel_size.y);
  EXPECT_FLOAT_EQ(cfg_ptr->get_voxel_size().z, voxel_size.z);
  // Capacity
  EXPECT_EQ(cfg_ptr->get_capacity(), capacity);
}

/// Bad configuration cases
TEST_F(VoxelTest, BadCases)
{
  // Min leaf size
  PointXYZ tmp;
  tmp = voxel_size;
  tmp.x = Config::MIN_VOXEL_SIZE_M - 0.00001F;
  EXPECT_THROW(Config(min_point, max_point, tmp, capacity), std::domain_error);
  tmp = voxel_size;
  tmp.y = Config::MIN_VOXEL_SIZE_M - 0.00001F;
  EXPECT_THROW(Config(min_point, max_point, tmp, capacity), std::domain_error);
  tmp = voxel_size;
  tmp.z = Config::MIN_VOXEL_SIZE_M - 0.00001F;
  EXPECT_THROW(Config(min_point, max_point, tmp, capacity), std::domain_error);
  // Min > Max
  tmp = min_point;
  tmp.x = max_point.x + 0.00001F;
  EXPECT_THROW(Config(tmp, max_point, voxel_size, capacity), std::domain_error);
  tmp = min_point;
  tmp.y = max_point.y + 0.00001F;
  EXPECT_THROW(Config(tmp, max_point, voxel_size, capacity), std::domain_error);
  tmp = min_point;
  tmp.z = max_point.z + 0.00001F;
  EXPECT_THROW(Config(tmp, max_point, voxel_size, capacity), std::domain_error);
  // Overflow: y stride
  tmp = min_point;
  PointXYZ tmp2 = max_point;
  constexpr float32_t min = -1.0E13F;
  constexpr float32_t max = 1.0E13F;
  tmp.x = min;
  tmp2.x = max;
  tmp.y = min;
  tmp2.y = max;
  EXPECT_THROW(Config(tmp, tmp2, voxel_size, capacity), std::domain_error);
  // Overflow: z stride
  tmp = min_point;
  tmp2 = max_point;
  tmp.z = min;
  tmp2.z = max;
  tmp.y = min;
  tmp2.y = max;
  EXPECT_THROW(Config(tmp, tmp2, voxel_size, capacity), std::domain_error);
  constexpr float32_t min2 = -std::numeric_limits<float32_t>::max();
  constexpr float32_t max2 = std::numeric_limits<float32_t>::max();
  // Overflow: x width
  tmp = min_point;
  tmp2 = max_point;
  tmp.x = min2;
  tmp2.x = max2;
  EXPECT_THROW(Config(tmp, tmp2, voxel_size, capacity), std::domain_error);
  // Overflow: y width
  tmp = min_point;
  tmp2 = max_point;
  tmp.y = min2;
  tmp2.y = max2;
  EXPECT_THROW(Config(tmp, tmp2, voxel_size, capacity), std::domain_error);
  // Overflow: z width
  tmp = min_point;
  tmp2 = max_point;
  tmp.z = min2;
  tmp2.z = max2;
  EXPECT_THROW(Config(tmp, tmp2, voxel_size, capacity), std::domain_error);
  // Overflow 2
  constexpr float32_t min3 = std::numeric_limits<float32_t>::min();
  // Overflow: x width
  tmp = min_point;
  tmp2 = max_point;
  tmp.x = min3;
  tmp2.x = max2;
  EXPECT_THROW(Config(tmp, tmp2, voxel_size, capacity), std::domain_error);
  // Overflow: y width
  tmp = min_point;
  tmp2 = max_point;
  tmp.y = min3;
  tmp2.y = max2;
  EXPECT_THROW(Config(tmp, tmp2, voxel_size, capacity), std::domain_error);
  // Overflow: z width
  tmp = min_point;
  tmp2 = max_point;
  tmp.z = min3;
  tmp2.z = max2;
  EXPECT_THROW(Config(tmp, tmp2, voxel_size, capacity), std::domain_error);
}
////////////////////////////////////////////////////////////////////////////////
TYPED_TEST(TypedVoxelTest, IndexSimple)
{
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, -0.5F, -0.5F)), 0UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, -0.5F, -0.5F)), 1UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, 0.5F, -0.5F)), 2UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, 0.5F, -0.5F)), 3UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, -0.5F, 0.5F)), 4UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, -0.5F, 0.5F)), 5UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, 0.5F, 0.5F)), 6UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, 0.5F, 0.5F)), 7UL);
}
/// make sure index computation works
TYPED_TEST(TypedVoxelTest, IndexEdge)
{
  // do boundary queries, expect to round down:
  /*   ^ y
    _______
   | 2 | 3 |
   |___|___| _> x
   | 0 | 1 |
   |___|___|

   Edge ownership should be as follows, for 0: bottom and left edge is 0's, edge 01, and 02 are not
   those belong to 1 and 2 respectively
  */
  //// simple internal edges (only on one boundary)
  // xy boundaries, z < 0
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.0F, -0.5F, -0.5F)), 1UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, 0.0F, -0.5F)), 3UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.0F, 0.5F, -0.5F)), 3UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, 0.0F, -0.5F)), 2UL);
  // xy boundaries, z > 0
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.0F, -0.5F, 0.5F)), 5UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, 0.0F, 0.5F)), 7UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.0F, 0.5F, 0.5F)), 7UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, 0.0F, 0.5F)), 6UL);
  // *z boundaries, z == 0
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, -0.5F, 0.0F)), 4UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, -0.5F, 0.0F)), 5UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, 0.5F, 0.0F)), 7UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, 0.5F, 0.0F)), 6UL);

  //// simple external edges
  // xy boundaries, z < 0
  EXPECT_EQ(this->cfg_ptr->index(this->make(-1.0F, -0.5F, -0.5F)), 0UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, -1.0F, -0.5F)), 0UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(1.0F, -0.5F, -0.5F)), 1UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, -1.0F, -0.5F)), 1UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(1.0F, 0.5F, -0.5F)), 3UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, 1.0F, -0.5F)), 3UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, 1.0F, -0.5F)), 2UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-1.0F, 0.5F, -0.5F)), 2UL);
  // xy boundaries, z > 0
  EXPECT_EQ(this->cfg_ptr->index(this->make(-1.0F, -0.5F, 0.5F)), 4UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, -1.0F, 0.5F)), 4UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(1.0F, -0.5F, 0.5F)), 5UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, -1.0F, 0.5F)), 5UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(1.0F, 0.5F, 0.5F)), 7UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, 1.0F, 0.5F)), 7UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, 1.0F, 0.5F)), 6UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-1.0F, 0.5F, 0.5F)), 6UL);
  // z boundaries, z == -1
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, -0.5F, -1.0F)), 0UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, -0.5F, -1.0F)), 1UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, 0.5F, -1.0F)), 3UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, 0.5F, -1.0F)), 2UL);
  // z boundaries, z == +1
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, -0.5F, 1.0F)), 4UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, -0.5F, 1.0F)), 5UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, 0.5F, 1.0F)), 7UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, 0.5F, 1.0F)), 6UL);

  //// internal edges (two boundaries)
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.0F, 0.0F, 0.5F)), 7UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.0F, 0.0F, -0.5F)), 3UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, 0.0F, 0.0F)), 7UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, 0.0F, 0.0F)), 6UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.0F, 0.5F, 0.0F)), 7UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.0F, -0.5F, 0.0F)), 5UL);

  //// external edges (two boundaries)
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.0F, 0.0F, 1.0F)), 7UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.0F, 0.0F, -1.0F)), 3UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(1.0F, 0.0F, 0.0F)), 7UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.0F, 1.0F, 0.0F)), 7UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.0F, -1.0F, 0.0F)), 5UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-1.0F, 0.0F, 0.0F)), 6UL);

  // complex edge (all three boundaries)
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.0F, 0.0F, 0.0F)), 7UL);
}

/// make sure oob points are snapped to the edges
TYPED_TEST(TypedVoxelTest, IndexOob)
{
  // voxel (0, 0, 0)
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, -0.5F, -1.5F)), 0UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, -1.5F, -1.5F)), 0UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-1.5F, -0.5F, -1.5F)), 0UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, -1.5F, -0.5F)), 0UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-1.5F, -0.5F, -0.5F)), 0UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-1.5F, -1.5F, -1.5F)), 0UL);
  // voxel (1, 0, 0)
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, -0.5F, -1.5F)), 1UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, -1.5F, -1.5F)), 1UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(1.5F, -0.5F, -1.5F)), 1UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, -1.5F, -0.5F)), 1UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(1.5F, -0.5F, -0.5F)), 1UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(1.5F, -1.5F, -1.5F)), 1UL);
  // voxel (0, 1, 0)
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, 0.5F, -1.5F)), 2UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, 1.5F, -1.5F)), 2UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-1.5F, 0.5F, -1.5F)), 2UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, 1.5F, -0.5F)), 2UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-1.5F, 0.5F, -0.5F)), 2UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-1.5F, 1.5F, -1.5F)), 2UL);
  // voxel (1, 1, 0)
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, 0.5F, -1.5F)), 3UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, 1.5F, -1.5F)), 3UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(1.5F, 0.5F, -1.5F)), 3UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, 1.5F, -0.5F)), 3UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(1.5F, 0.5F, -0.5F)), 3UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(1.5F, 1.5F, -1.5F)), 3UL);
  // voxel (0, 0, 1)
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, -0.5F, 1.5F)), 4UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, -1.5F, 1.5F)), 4UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-1.5F, -0.5F, 1.5F)), 4UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, -1.5F, 0.5F)), 4UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-1.5F, -0.5F, 0.5F)), 4UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-1.5F, -1.5F, 1.5F)), 4UL);
  // voxel (1, 0, 1)
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, -0.5F, 1.5F)), 5UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, -1.5F, 1.5F)), 5UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(1.5F, -0.5F, 1.5F)), 5UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, -1.5F, 0.5F)), 5UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(1.5F, -0.5F, 0.5F)), 5UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(1.5F, -1.5F, 1.5F)), 5UL);
  // voxel (0, 1, 1)
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, 0.5F, 1.5F)), 6UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, 1.5F, 1.5F)), 6UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-1.5F, 0.5F, 1.5F)), 6UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-0.5F, 1.5F, 0.5F)), 6UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-1.5F, 0.5F, 0.5F)), 6UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(-1.5F, 1.5F, 1.5F)), 6UL);
  // voxel (1, 1, 1)
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, 0.5F, 1.5F)), 7UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, 1.5F, 1.5F)), 7UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(1.5F, 0.5F, 1.5F)), 7UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(0.5F, 1.5F, 0.5F)), 7UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(1.5F, 0.5F, 0.5F)), 7UL);
  EXPECT_EQ(this->cfg_ptr->index(this->make(1.5F, 1.5F, 1.5F)), 7UL);
}

/// \brief Test generic voxel mechanisms
TYPED_TEST(TypedVoxelTest, Voxel)
{
  Voxel<TypeParam> v1;
  // Dispatches to calls to occupied() and count()
  EXPECT_FALSE(static_cast<bool8_t>(v1));
  // We have no guarantees on how PointT might get default initialized
  const TypeParam p = this->make(1.0F, -1.0F, 1.5F);
  Voxel<TypeParam> v2{p};
  EXPECT_TRUE(static_cast<bool8_t>(v2));
  EXPECT_EQ(v2.count(), 1U);
  // Dispatches to get()
  const TypeParam q{static_cast<TypeParam>(v2)};
  EXPECT_EQ(q.x, 1.0F);
  EXPECT_EQ(q.y, -1.0F);
  EXPECT_EQ(q.z, 1.5F);
  v2.clear();
  EXPECT_FALSE(static_cast<bool8_t>(v2));
  EXPECT_EQ(v2.count(), 0U);
  EXPECT_THROW(v2.get(), std::out_of_range);
}
////////////////////////////////////////////////////////////////////////////////
/// simple centroid computation for various types
TYPED_TEST(TypedVoxelTest, CentroidVoxel)
{
  // simple xyz point
  CentroidVoxel<TypeParam> v1;
  // Exercise configure noop
  EXPECT_FALSE(static_cast<bool8_t>(v1));
  v1.configure(*this->cfg_ptr, 0U);
  EXPECT_FALSE(static_cast<bool8_t>(v1));
  TypeParam pt1;
  const float32_t TOL = 1.0E-6F;
  // Previously checked that everything was 0, but we can't get from an empty voxel
  v1.add_observation(this->make(1.0F, 1.0F, 1.0F));
  pt1 = v1.get();
  ASSERT_LT(fabs(pt1.x - 1.0F), TOL);
  ASSERT_LT(fabs(pt1.y - 1.0F), TOL);
  ASSERT_LT(fabs(pt1.z - 1.0F), TOL);
  v1.add_observation(this->make(0.0F, 0.0F, 0.0F));
  pt1 = v1.get();
  ASSERT_LT(fabs(pt1.x - 0.5F), TOL);
  ASSERT_LT(fabs(pt1.y - 0.5F), TOL);
  ASSERT_LT(fabs(pt1.z - 0.5F), TOL);
  v1.add_observation(this->make(-1.0F, -1.0F, -1.0F));
  pt1 = v1.get();
  ASSERT_LT(fabs(pt1.x), TOL);
  ASSERT_LT(fabs(pt1.y), TOL);
  ASSERT_LT(fabs(pt1.z), TOL);

  v1.clear();
  v1.add_observation(this->make(1.0F, 1.0F, 1.0F));
  pt1 = v1.get();
  ASSERT_LT(fabs(pt1.x - 1.0F), TOL);
  ASSERT_LT(fabs(pt1.y - 1.0F), TOL);
  ASSERT_LT(fabs(pt1.z - 1.0F), TOL);
  v1.add_observation(this->make(0.0F, 0.0F, 0.0F));
  pt1 = v1.get();
  ASSERT_LT(fabs(pt1.x - 0.5F), TOL);
  ASSERT_LT(fabs(pt1.y - 0.5F), TOL);
  ASSERT_LT(fabs(pt1.z - 0.5F), TOL);
  v1.add_observation(this->make(-1.0F, -1.0F, -1.0F));
  pt1 = v1.get();
  ASSERT_LT(fabs(pt1.x), TOL);
  ASSERT_LT(fabs(pt1.y), TOL);
  ASSERT_LT(fabs(pt1.z), TOL);
}

/// approximate centroid
TYPED_TEST(TypedVoxelTest, ApproximateCentroid)
{
  constexpr float32_t TOL = 1.0E-6F;
  TypeParam pt;
  // Validate computation of centroids
  pt = this->cfg_ptr->template centroid<TypeParam>(0U);
  ASSERT_LT(fabsf(pt.x + 0.5F), TOL);
  ASSERT_LT(fabsf(pt.y + 0.5F), TOL);
  ASSERT_LT(fabsf(pt.z + 0.5F), TOL);

  pt = this->cfg_ptr->template centroid<TypeParam>(1U);
  ASSERT_LT(fabsf(pt.x - 0.5F), TOL);
  ASSERT_LT(fabsf(pt.y + 0.5F), TOL);
  ASSERT_LT(fabsf(pt.z + 0.5F), TOL);

  pt = this->cfg_ptr->template centroid<TypeParam>(2U);
  ASSERT_LT(fabsf(pt.x + 0.5F), TOL);
  ASSERT_LT(fabsf(pt.y - 0.5F), TOL);
  ASSERT_LT(fabsf(pt.z + 0.5F), TOL);

  pt = this->cfg_ptr->template centroid<TypeParam>(3U);
  ASSERT_LT(fabsf(pt.x - 0.5F), TOL);
  ASSERT_LT(fabsf(pt.y - 0.5F), TOL);
  ASSERT_LT(fabsf(pt.z + 0.5F), TOL);

  pt = this->cfg_ptr->template centroid<TypeParam>(4U);
  ASSERT_LT(fabsf(pt.x + 0.5F), TOL);
  ASSERT_LT(fabsf(pt.y + 0.5F), TOL);
  ASSERT_LT(fabsf(pt.z - 0.5F), TOL);

  pt = this->cfg_ptr->template centroid<TypeParam>(5U);
  ASSERT_LT(fabsf(pt.x - 0.5F), TOL);
  ASSERT_LT(fabsf(pt.y + 0.5F), TOL);
  ASSERT_LT(fabsf(pt.z - 0.5F), TOL);

  pt = this->cfg_ptr->template centroid<TypeParam>(6U);
  ASSERT_LT(fabsf(pt.x + 0.5F), TOL);
  ASSERT_LT(fabsf(pt.y - 0.5F), TOL);
  ASSERT_LT(fabsf(pt.z - 0.5F), TOL);

  pt = this->cfg_ptr->template centroid<TypeParam>(7U);
  ASSERT_LT(fabsf(pt.x - 0.5F), TOL);
  ASSERT_LT(fabsf(pt.y - 0.5F), TOL);
  ASSERT_LT(fabsf(pt.z - 0.5F), TOL);

  // Check actual mechanisms for ApproximateVoxel
  ApproximateVoxel<TypeParam> v;
  EXPECT_EQ(v.count(), 0U);
  // -0.5
  v.configure(*this->cfg_ptr, 0U);
  EXPECT_EQ(v.count(), 0U);
  // + 0.5
  v.add_observation(pt);
  pt = v.get();
  EXPECT_LT(fabsf(pt.x + 0.5F), TOL);
  EXPECT_LT(fabsf(pt.y + 0.5F), TOL);
  EXPECT_LT(fabsf(pt.z + 0.5F), TOL);
  EXPECT_EQ(v.count(), 1U);
}

/// test weirdness: indices should not roll over
TYPED_TEST(TypedVoxelTest, SmallVoxel)
{
  constexpr float32_t sz = 0.03F;
  this->min_point.x = -130.0F;
  this->min_point.y = -130.0F;
  this->min_point.z = -3.0F;
  this->max_point.x = 130.0F;
  this->max_point.y = 130.0F;
  this->max_point.z = 10.0F;
  this->voxel_size.x = sz;
  this->voxel_size.y = sz;
  this->voxel_size.z = sz;
  Config cfg(this->min_point, this->max_point, this->voxel_size, this->capacity);

  TypeParam pt;
  pt.x = 0.0F;
  pt.y = 0.0F;
  pt.z = 0.0F;

  const uint64_t idx = cfg.index(pt);
  TypeParam pt2 = cfg.template centroid<TypeParam>(idx);

  ASSERT_LT(fabsf(pt2.x), 2.0F * sz);
  ASSERT_LT(fabsf(pt2.y), 2.0F * sz);
  ASSERT_LT(fabsf(pt2.z), 2.0F * sz);
}

////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
class TypedVoxelGridTest : public TypedVoxelTest<PointT>
{
public:
  TypedVoxelGridTest()
  : TypedVoxelTest<PointT>()
  {
    // Fill out reference points
    obs_points1 = {
      this->make(-1.0F, -1.0F, -1.0F),  // voxel 0
      this->make(-0.5F, -0.5F, -0.5F),
      this->make(1.0F, -1.0F, -1.0F),  // voxel 1
      this->make(0.5F, -0.5F, -0.5F),
      this->make(-1.0F, 1.0F, -1.0F),  // voxel 2
      this->make(-0.5F, 0.5F, -0.5F),
      this->make(1.0F, 1.0F, -1.0F),  // voxel 3
      this->make(0.5F, 0.5F, -0.5F),
      this->make(-1.0F, -1.0F, 1.0F),  // voxel 4
      this->make(-0.5F, -0.5F, 0.5F),
      this->make(1.0F, -1.0F, 1.0F),  // voxel 5
      this->make(0.5F, -0.5F, 0.5F),
      this->make(-1.0F, 1.0F, 1.0F),  // voxel 6
      this->make(-0.5F, 0.5F, 0.5F),
      this->make(1.0F, 1.0F, 1.0F),  // voxel 7
      this->make(0.5F, 0.5F, 0.5F)
    };
    obs_points2[0U] = this->make(-1.0F, -1.0F, 1.0F);  // voxel 4
    obs_points2[1U] = this->make(1.0F, -1.0F, 1.0F);  // voxel 5
    obs_points2[2U] = this->make(-1.0F, 1.0F, 1.0F);  // voxel 6
    obs_points2[3U] = this->make(1.0F, 1.0F, 1.0F);  // voxel 7
  }

protected:
  bool8_t check(const PointT & pt, const PointT & ref)
  {
    constexpr float32_t TOL = 1.0E-6F;
    return (fabsf(pt.x - ref.x) <= TOL) &&
           (fabsf(pt.y - ref.y) <= TOL) &&
           (fabsf(pt.z - ref.z) <= TOL);
  }
  template<std::size_t N>
  bool8_t check(const PointT & pt, const std::array<PointT, N> & ref, const std::size_t max = N)
  {
    bool8_t ret = false;
    for (std::size_t idx = 0U; idx < max; ++idx) {
      const auto tmp = ref[idx];
      if (check(pt, tmp)) {
        ret = true;
        break;
      }
    }
    return ret;
  }
  std::array<PointT, 16U> obs_points1;
  std::array<PointT, 4U> obs_points2;
  std::array<PointT, 8U> ref_points1;
  std::array<PointT, 4U> ref_points2;
};

// TODO(c.ho) move to another file
// TYPED_TEST_SUITE(TypedVoxelGridTest, PointTypes);
TYPED_TEST_SUITE(TypedVoxelGridTest, PointXYZ, );

/// basic i/o for voxel grid
TYPED_TEST(TypedVoxelGridTest, centroid_voxel_grid)
{
  VoxelGrid<CentroidVoxel<TypeParam>> grid{*this->cfg_ptr};
  this->ref_points1[0U] = this->make(-0.75F, -0.75F, -0.75F);
  this->ref_points1[1U] = this->make(0.75F, -0.75F, -0.75F);
  this->ref_points1[2U] = this->make(-0.75F, 0.75F, -0.75F);
  this->ref_points1[3U] = this->make(0.75F, 0.75F, -0.75F);
  this->ref_points1[4U] = this->make(-0.75F, -0.75F, 0.75F);
  this->ref_points1[5U] = this->make(0.75F, -0.75F, 0.75F);
  this->ref_points1[6U] = this->make(-0.75F, 0.75F, 0.75F);
  this->ref_points1[7U] = this->make(0.75F, 0.75F, 0.75F);
  // TODO(c.ho) properly order these...
  this->ref_points2[0U] = this->make(1.0F, 1.0F, 1.0F);
  this->ref_points2[1U] = this->make(-1.0F, 1.0F, 1.0F);
  this->ref_points2[2U] = this->make(1.0F, -1.0F, 1.0F);
  this->ref_points2[3U] = this->make(-1.0F, -1.0F, 1.0F);
  // TODO(c.ho) Memory testing after static allocators
  EXPECT_TRUE(grid.empty());
  EXPECT_EQ(grid.size(), 0U);
  EXPECT_EQ(grid.capacity(), this->capacity);
  // Add more points: scan 0
  grid.insert(this->obs_points2.begin(), this->obs_points2.end());
  EXPECT_EQ(grid.size(), 4U);
  EXPECT_EQ(grid.capacity(), this->capacity);
  // Check via new_voxels: all
  const auto & list = grid.new_voxels();
  for (const auto it : list) {
    EXPECT_TRUE(this->check(it->second.get(), this->ref_points2));
  }
  // make sure the voxels are right
  for (const auto it : grid) {
    EXPECT_TRUE(this->check(it.second.get(), this->ref_points2));
  }
  // reset
  grid.clear();
  EXPECT_TRUE(grid.empty());
  EXPECT_EQ(grid.size(), 0U);
  EXPECT_EQ(grid.capacity(), this->capacity);

  // add a bunch of points: Scan 1
  const std::size_t mid = 9U;
  for (std::size_t idx = 0U; idx < mid; ++idx) {
    grid.insert(this->obs_points1[idx]);
  }
  EXPECT_EQ(grid.size(), 5U);
  EXPECT_EQ(grid.capacity(), this->capacity);

  // Check via new_voxels
  const auto & list1 = grid.new_voxels();
  // Voxels are in reverse order
  auto tmp = list1.begin();
  EXPECT_NE(tmp, list1.end());
  EXPECT_TRUE(this->check((*tmp)->second.get(), this->obs_points1[8U]));  // Single observation
  ++tmp;
  EXPECT_NE(tmp, list1.end());
  EXPECT_TRUE(this->check((*tmp)->second.get(), this->ref_points1[3U]));
  ++tmp;
  EXPECT_NE(tmp, list1.end());
  EXPECT_TRUE(this->check((*tmp)->second.get(), this->ref_points1[2U]));
  ++tmp;
  EXPECT_NE(tmp, list1.end());
  EXPECT_TRUE(this->check((*tmp)->second.get(), this->ref_points1[1U]));
  ++tmp;
  EXPECT_NE(tmp, list1.end());
  EXPECT_TRUE(this->check((*tmp)->second.get(), this->ref_points1[0U]));
  ++tmp;
  EXPECT_EQ(tmp, list1.end());
  // Insert remainder
  for (std::size_t idx = mid; idx < this->obs_points1.size() - 2U; ++idx) {
    grid.insert(this->obs_points1[idx]);
  }
  // Check new
  EXPECT_EQ(grid.size(), this->ref_points1.size() - 1U);
  EXPECT_EQ(grid.capacity(), this->capacity);
  const auto & list2 = grid.new_voxels();
  EXPECT_EQ(list1, list2);
  // Voxels are in reverse order
  tmp = list1.begin();
  EXPECT_NE(tmp, list1.end());
  EXPECT_TRUE(this->check((*tmp)->second.get(), this->ref_points1[6U]));  // Single observation
  ++tmp;
  EXPECT_NE(tmp, list1.end());
  EXPECT_TRUE(this->check((*tmp)->second.get(), this->ref_points1[5U]));
  ++tmp;
  EXPECT_EQ(tmp, list1.end());

  // make sure the voxels are right
  for (const auto it : grid) {
    EXPECT_TRUE(this->check(it.second.get(), this->ref_points1, this->ref_points1.size() - 1U));
  }
  // Bad case: insert too many
  EXPECT_THROW(grid.insert(*(this->obs_points1.end() - 1)), std::length_error);
  EXPECT_THROW(grid.insert(*(this->obs_points1.end() - 2)), std::length_error);
}

/// basic i/o for approximate voxel grid
TYPED_TEST(TypedVoxelGridTest, approximate_voxel_grid)
{
  VoxelGrid<ApproximateVoxel<TypeParam>> grid{*this->cfg_ptr};
  this->ref_points1[0U] = this->make(-0.5F, -0.5F, -0.5F);
  this->ref_points1[1U] = this->make(0.5F, -0.5F, -0.5F);
  this->ref_points1[2U] = this->make(-0.5F, 0.5F, -0.5F);
  this->ref_points1[3U] = this->make(0.5F, 0.5F, -0.5F);
  this->ref_points1[4U] = this->make(-0.5F, -0.5F, 0.5F);
  this->ref_points1[5U] = this->make(0.5F, -0.5F, 0.5F);
  this->ref_points1[6U] = this->make(-0.5F, 0.5F, 0.5F);
  this->ref_points1[7U] = this->make(0.5F, 0.5F, 0.5F);
  // TODO(c.ho) properly order these...
  this->ref_points2[0U] = this->make(0.5F, 0.5F, 0.5F);
  this->ref_points2[1U] = this->make(-0.5F, 0.5F, 0.5F);
  this->ref_points2[2U] = this->make(0.5F, -0.5F, 0.5F);
  this->ref_points2[3U] = this->make(-0.5F, -0.5F, 0.5F);

  EXPECT_TRUE(grid.empty());
  EXPECT_EQ(grid.size(), 0U);
  EXPECT_EQ(grid.capacity(), this->capacity);
  // Add more points: scan 0
  grid.insert(this->obs_points2.begin(), this->obs_points2.end());
  EXPECT_EQ(grid.size(), 4U);
  EXPECT_EQ(grid.capacity(), this->capacity);
  // Check via new_voxels TODO
  // make sure the voxels are right
  for (const auto it : grid) {
    EXPECT_TRUE(this->check(it.second.get(), this->ref_points2));
  }
  // reset
  grid.clear();
  EXPECT_TRUE(grid.empty());
  EXPECT_EQ(grid.size(), 0U);
  EXPECT_EQ(grid.capacity(), this->capacity);

  // add a bunch of points: Scan 1
  const std::size_t mid = 9U;
  for (std::size_t idx = 0U; idx < mid; ++idx) {
    grid.insert(this->obs_points1[idx]);
  }
  EXPECT_EQ(grid.size(), 5U);
  EXPECT_EQ(grid.capacity(), this->capacity);

  // Check via new_voxels
  const auto & list1 = grid.new_voxels();
  // Voxels are in reverse order
  auto tmp = list1.begin();
  EXPECT_NE(tmp, list1.end());
  EXPECT_TRUE(this->check((*tmp)->second.get(), this->ref_points1[4U]));
  ++tmp;
  EXPECT_NE(tmp, list1.end());
  EXPECT_TRUE(this->check((*tmp)->second.get(), this->ref_points1[3U]));
  ++tmp;
  EXPECT_NE(tmp, list1.end());
  EXPECT_TRUE(this->check((*tmp)->second.get(), this->ref_points1[2U]));
  ++tmp;
  EXPECT_NE(tmp, list1.end());
  EXPECT_TRUE(this->check((*tmp)->second.get(), this->ref_points1[1U]));
  ++tmp;
  EXPECT_NE(tmp, list1.end());
  EXPECT_TRUE(this->check((*tmp)->second.get(), this->ref_points1[0U]));
  ++tmp;
  EXPECT_EQ(tmp, list1.end());
  // Insert remainder
  for (std::size_t idx = mid; idx < this->obs_points1.size() - 2U; ++idx) {
    grid.insert(this->obs_points1[idx]);
  }
  // Check new
  EXPECT_EQ(grid.size(), this->ref_points1.size() - 1U);
  EXPECT_EQ(grid.capacity(), this->capacity);
  const auto & list2 = grid.new_voxels();
  EXPECT_EQ(list1, list2);
  // Voxels are in reverse order
  tmp = list1.begin();
  EXPECT_NE(tmp, list1.end());
  EXPECT_TRUE(this->check((*tmp)->second.get(), this->ref_points1[6U]));  // Single observation
  ++tmp;
  EXPECT_NE(tmp, list1.end());
  EXPECT_TRUE(this->check((*tmp)->second.get(), this->ref_points1[5U]));
  ++tmp;
  EXPECT_EQ(tmp, list1.end());
  // make sure the voxels are right
  for (const auto it : grid) {
    EXPECT_TRUE(this->check(it.second.get(), this->ref_points1, this->ref_points1.size() - 1U));
  }
  // Bad case: insert too many
  EXPECT_THROW(grid.insert(*(this->obs_points1.end() - 1)), std::length_error);
  EXPECT_THROW(grid.insert(*(this->obs_points1.end() - 2)), std::length_error);
}
#endif  // TEST_VOXEL_GRID_HPP_
