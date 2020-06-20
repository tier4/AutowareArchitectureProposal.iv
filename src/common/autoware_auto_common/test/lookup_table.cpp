// Copyright 2017-2020 Apex.AI, Inc.
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

#include <helper_functions/lookup_table.hpp>
#include <common/types.hpp>

#include <memory>
#include <vector>

using autoware::common::helper_functions::lookup_1d;
using autoware::common::helper_functions::LookupTable1D;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

template<typename T>
class bad_case : public ::testing::Test
{
};

using BadTypes = ::testing::Types<float32_t, float64_t, int32_t, uint32_t>;
TYPED_TEST_CASE(bad_case, BadTypes, );

// Empty domain/range
TYPED_TEST(bad_case, empty)
{
  using T = TypeParam;
  EXPECT_THROW(lookup_1d<T>({}, {}, T{}), std::domain_error);
}

// Unequal domain/range
TYPED_TEST(bad_case, unequal_domain)
{
  using T = TypeParam;
  EXPECT_THROW(lookup_1d<T>({T{1}, T{2}}, {T{1}}, T{}), std::domain_error);
}

// Not sorted domain
TYPED_TEST(bad_case, domain_not_sorted)
{
  using T = TypeParam;
  EXPECT_THROW(lookup_1d<T>({T{2}, T{1}}, {T{1}, T{2}}, T{}), std::domain_error);
}

////////////////////////////////////////////////////////////////////////////////

template<typename Type>
class sanity_check : public ::testing::Test
{
public:
  using T = Type;

  void SetUp() override
  {
    domain_ = std::vector<Type>{T{1}, T{3}, T{5}};
    range_ = std::vector<Type>{T{2}, T{4}, T{0}};
    ASSERT_NO_THROW(table_ = std::make_unique<LookupTable1D<Type>>(domain_, range_));
  }

  bool not_in_domain(const T bad_value) const noexcept
  {
    return std::find(domain_.begin(), domain_.end(), bad_value) == domain_.end();
  }

  void check(const T expected, const T actual) const noexcept
  {
    if (std::is_floating_point<T>::value) {
      EXPECT_FLOAT_EQ(actual, expected);
    } else {
      EXPECT_EQ(actual, expected);
    }
  }

protected:
  std::vector<Type> domain_{};
  std::vector<Type> range_{};
  std::unique_ptr<LookupTable1D<Type>> table_{};
};

using NormalTypes = ::testing::Types<float32_t, float64_t, int32_t, uint32_t>;
TYPED_TEST_CASE(sanity_check, NormalTypes, );

TYPED_TEST(sanity_check, exact)
{
  const auto x = this->domain_[1U];
  const auto result = this->table_->lookup(x);
  ASSERT_FALSE(this->not_in_domain(x));
  this->check(result, this->range_[1U]);
}

TYPED_TEST(sanity_check, interpolation)
{
  const auto x = TypeParam{2};
  // Asssert x is not identically in domain_
  ASSERT_TRUE(this->not_in_domain(x));
  const auto result = this->table_->lookup(x);
  this->check(result, TypeParam{3});
}

TYPED_TEST(sanity_check, above_range)
{
  const auto x = TypeParam{999999};
  ASSERT_GT(x, this->domain_.back());  // domain is known to be sorted
  const auto result = this->table_->lookup(x);
  this->check(result, this->range_.back());
}

TYPED_TEST(sanity_check, below_range)
{
  const auto x = TypeParam{0};
  ASSERT_LT(x, this->domain_.front());  // domain is known to be sorted
  const auto result = this->table_->lookup(x);
  this->check(result, this->range_.front());
}

// TODO(c.ho) check with more interesting functions
