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

#include <helper_functions/message_adapters.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <time_utils/time_utils.hpp>
#include <memory>
#include <vector>

using autoware::common::helper_functions::message_field_adapters::get_stamp;
using autoware::common::helper_functions::message_field_adapters::get_frame_id;

TEST(MessageFieldAdapterTest, const_header_tests) {
  using Message = geometry_msgs::msg::TransformStamped;

  const auto stamp = ::time_utils::to_message(std::chrono::system_clock::now());
  const auto frame_id = "MessageFieldAdapterTest_frame";

  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = frame_id;
  const Message msg{Message{}.set__header(header)};

  EXPECT_EQ(stamp, get_stamp(msg));
  EXPECT_EQ(frame_id, get_frame_id(msg));
}

TEST(MessageFieldAdapterTest, nonconst_header_tests) {
  using Message = geometry_msgs::msg::TransformStamped;

  const auto stamp = ::time_utils::to_message(std::chrono::system_clock::now());
  const auto frame_id = "MessageFieldAdapterTest_frame";
  const auto stamp2 = ::time_utils::to_message(std::chrono::system_clock::now() +
      std::chrono::milliseconds{500});
  const auto frame_id2 = "MessageFieldAdapterTest_frame2";

  ASSERT_NE(stamp, stamp2);
  ASSERT_NE(frame_id, frame_id2);

  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = frame_id;
  Message msg{Message{}.set__header(header)};

  EXPECT_EQ(stamp, get_stamp(msg));
  EXPECT_EQ(frame_id, get_frame_id(msg));

  get_stamp(msg) = stamp2;
  get_frame_id(msg) = frame_id2;

  EXPECT_EQ(stamp2, get_stamp(msg));
  EXPECT_EQ(frame_id2, get_frame_id(msg));
}
