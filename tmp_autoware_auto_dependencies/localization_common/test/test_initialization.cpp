// Copyright 2020 the Autoware Foundation
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
#include <localization_common/initialization.hpp>
#include <time_utils/time_utils.hpp>
#include "test_initialization.hpp"

using autoware::localization::localization_common::BestEffortInitializer;

class BestEffortInitializationTest : public
  ::testing::TestWithParam<BestEffortInitializerTestParams>
{
};

TEST(BestEffortInitializationTest, BadLookup) {
  BestEffortInitializer initializer;
  const auto now = std::chrono::system_clock::now();
  constexpr auto target_frame{"base_link"};
  constexpr auto source_frame{"map"};
  constexpr auto nonexisting_frame{"asdasdasd"};
  constexpr std::chrono::seconds dt{1U};
  ASSERT_NE(nonexisting_frame, target_frame);
  ASSERT_NE(nonexisting_frame, source_frame);
  tf2::BufferCore tf_graph;
  EXPECT_THROW(
    initializer.guess(tf_graph, now, target_frame, source_frame),
    tf2::LookupException);
  auto transform = make_transform(15.0F, 15.0F, 15.0F, 15.0F, 15.0F, 15.0F);
  transform.header.stamp = time_utils::to_message(now);
  tf_graph.setTransform(transform, "testauthority");
  EXPECT_NO_THROW(
    check_transform_eq(
      transform, initializer.guess(tf_graph, now, target_frame, source_frame)));

  // Extrapolation can be performed now that there's data in the tf graph.
  EXPECT_NO_THROW(
    check_transform_eq(
      transform, initializer.guess(tf_graph, now + dt, target_frame, source_frame)));

  // Backwards extrapolation is not supported.
  EXPECT_THROW(
    check_transform_eq(
      transform, initializer.guess(tf_graph, now - dt, target_frame, source_frame)),
    std::domain_error);

  EXPECT_THROW(
    initializer.guess(
      tf_graph, now,
      nonexisting_frame, source_frame), tf2::LookupException);
}

TEST_P(BestEffortInitializationTest, Basic) {
  tf2::BufferCore tf_graph;
  const auto param = GetParam();
  const auto dt = param.dt;
  const auto dt_small = param.dt_small;

  const auto ratio = static_cast<float32_t>(dt_small.count()) / static_cast<float32_t>(dt.count());
  ASSERT_TRUE(ratio < 1.0);
  constexpr auto target_frame{"base_link"};
  constexpr auto source_frame{"map"};

  BestEffortInitializer initializer;

  const auto t0 = std::chrono::system_clock::now();
  const auto t_interpolate = t0 + dt_small;
  const auto t1 = t0 + dt;
  auto tf0 = param.tf0;
  tf0.header.stamp = time_utils::to_message(t0);
  auto tf1 = param.tf1;
  tf1.header.stamp = time_utils::to_message(t1);

  tf_graph.setTransform(tf0, "testauthority");
  tf_graph.setTransform(tf1, "testauthority");

  geometry_msgs::msg::TransformStamped tf_interpolated;
  tf_interpolated.header = tf0.header;
  tf_interpolated.header.stamp = time_utils::to_message(t_interpolate);
  tf_interpolated.transform = get_interpolation(tf0.transform, tf1.transform, ratio);

  const auto tf0_guess = initializer.guess(tf_graph, t0, target_frame, source_frame);
  const auto tf_interpolate_guess = initializer.guess(
    tf_graph, t_interpolate, target_frame,
    source_frame);
  const auto tf1_guess = initializer.guess(tf_graph, t1, target_frame, source_frame);
  const auto tf_extrapolate_guess =
    initializer.guess(tf_graph, t1 + dt, target_frame, source_frame);

  check_transform_eq(tf0, tf0_guess);
  check_transform_eq(tf_interpolated, tf_interpolate_guess);
  check_transform_eq(tf1, tf1_guess);
  check_transform_eq(tf1, tf_extrapolate_guess);
}

INSTANTIATE_TEST_SUITE_P(
  SanityTest, BestEffortInitializationTest,
  ::testing::Values(
    BestEffortInitializerTestParams{
  make_transform(50.0F, 25.0F, -50.0F, -5.0F, 5.0F, 0.0F),
  make_transform(100.0F, -100.0F, 100.0F, 10.0F, 10.0F, 10.0F),
  std::chrono::milliseconds{100},
  std::chrono::milliseconds{25}
},
    BestEffortInitializerTestParams{
  make_transform(50.0F, 25.0F, -50.0F, -5.0F, 5.0F, 0.0F),
  make_transform(100.0F, -100.0F, 100.0F, 10.0F, 10.0F, 10.0F),
  std::chrono::milliseconds{100},
  std::chrono::milliseconds{1}
},
    BestEffortInitializerTestParams{
  make_transform(50.0F, 25.0F, -50.0F, -5.0F, 5.0F, 0.0F),
  make_transform(100.0F, -100.0F, 100.0F, 10.0F, 10.0F, 10.0F),
  std::chrono::milliseconds{100},
  std::chrono::milliseconds{99}
},
    BestEffortInitializerTestParams{
  make_transform(0.0F, -180.0F, 5.0F, -54.51F, 120.8754F, 0.0F),
  make_transform(0.0F, -180.0F, 95.0F, 65.12F, 10.65F, 0.0018955F),
  std::chrono::milliseconds{100},
  std::chrono::milliseconds{50}
}
    // cppcheck-suppress syntaxError
  ), );

/////////// Helper function implementations:

geometry_msgs::msg::TransformStamped make_transform(
  float ang_x, float ang_y, float ang_z,
  float x, float y, float z)
{
  // using tf2 structs to quickly convert angles to quaternion
  tf2::Quaternion quat;
  quat.setRPY(ang_x, ang_y, ang_z);
  const auto rot = geometry_msgs::msg::Quaternion{}.
  set__x(quat.x()).set__y(quat.y()).set__z(quat.z()).set__w(quat.w());
  const auto trans = geometry_msgs::msg::Vector3{}.set__x(x).set__y(y).set__z(z);
  const auto transform = geometry_msgs::msg::Transform{}.set__rotation(rot).set__translation(trans);
  geometry_msgs::msg::TransformStamped ret;
  ret.transform = transform;
  ret.header.frame_id = "base_link";
  ret.child_frame_id = "map";
  return ret;
}

void check_transform_eq(
  const geometry_msgs::msg::TransformStamped & t1,
  const geometry_msgs::msg::TransformStamped & t2)
{
  EXPECT_EQ(t1.header, t2.header);
  EXPECT_FLOAT_EQ(t1.transform.translation.x, t2.transform.translation.x);
  EXPECT_FLOAT_EQ(t1.transform.translation.y, t2.transform.translation.y);
  EXPECT_FLOAT_EQ(t1.transform.translation.z, t2.transform.translation.z);
  EXPECT_FLOAT_EQ(t1.transform.rotation.x, t2.transform.rotation.x);
  EXPECT_FLOAT_EQ(t1.transform.rotation.y, t2.transform.rotation.y);
  EXPECT_FLOAT_EQ(t1.transform.rotation.z, t2.transform.rotation.z);
  EXPECT_FLOAT_EQ(t1.transform.rotation.w, t2.transform.rotation.w);
}

geometry_msgs::msg::Transform get_interpolation(
  const geometry_msgs::msg::Transform & tf1, const geometry_msgs::msg::Transform & tf2,
  float ratio)
{
  geometry_msgs::msg::Transform ret;

  auto interpolate_line = [ratio](auto pt1, auto pt2) {
      return pt1 + (pt2 - pt1) * ratio;
    };
  // compute translation:
  ret.translation.x = interpolate_line(tf1.translation.x, tf2.translation.x);
  ret.translation.y = interpolate_line(tf1.translation.y, tf2.translation.y);
  ret.translation.z = interpolate_line(tf1.translation.z, tf2.translation.z);

  // compute rotation:
  tf2::Quaternion q1{tf1.rotation.x,
    tf1.rotation.y,
    tf1.rotation.z,
    tf1.rotation.w};
  tf2::Quaternion q2{tf2.rotation.x,
    tf2.rotation.y,
    tf2.rotation.z,
    tf2.rotation.w};
  const auto rot_quat = tf2::slerp(q1, q2, ratio);
  ret.rotation.set__x(rot_quat.x()).set__y(rot_quat.y()).set__z(rot_quat.z()).set__w(rot_quat.w());
  return ret;
}
