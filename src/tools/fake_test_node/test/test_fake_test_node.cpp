// Copyright 2021 Apex.AI, Inc.
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

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#include <common/types.hpp>
#include <fake_test_node/fake_test_node.hpp>
#include <std_msgs/msg/bool.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using autoware::common::types::bool8_t;

using FakeNodeFixture = autoware::tools::testing::FakeTestNode;
using FakeNodeFixtureParametrized = autoware::tools::testing::FakeTestNodeParametrized<bool8_t>;
using std_msgs::msg::Bool;

namespace
{

class RepublishingNodeUnderTest : public rclcpp::Node
{
public:
  RepublishingNodeUnderTest()
  : rclcpp::Node{"state_estimation_node"}
  {
    m_pub = this->create_publisher<std_msgs::msg::Bool>("/output_topic", 10);
    m_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/input_topic", 10,
      [&](const std_msgs::msg::Bool::SharedPtr msg) {publish(*msg);});
  }

  void publish(const std_msgs::msg::Bool & msg)
  {
    m_pub->publish(msg);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub{};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub{};
};

template<typename FixtureT>
void run_test(bool8_t value_in_message, FixtureT * fixture)
{
  std_msgs::msg::Bool msg{};
  msg.data = value_in_message;
  const auto node = std::make_shared<RepublishingNodeUnderTest>();

  Bool::SharedPtr last_received_msg{};
  auto fake_odom_publisher = fixture->template create_publisher<Bool>("/input_topic");
  auto result_odom_subscription = fixture->template create_subscription<Bool>(
    "/input_topic", *node,
    [&last_received_msg](const Bool::SharedPtr msg) {last_received_msg = msg;});

  const auto dt{std::chrono::milliseconds{100LL}};
  const auto max_wait_time{std::chrono::seconds{10LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (!last_received_msg) {
    fake_odom_publisher->publish(msg);
    rclcpp::spin_some(node);
    rclcpp::spin_some(fixture->get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      FAIL() << "Did not receive a message soon enough.";
    }
  }
  EXPECT_EQ(last_received_msg->data, value_in_message);
  SUCCEED();
}


}  // namespace


/// @test Test that we can use a non-parametrized test.
TEST_F(FakeNodeFixture, Test) {
  run_test(true, this);
}

INSTANTIATE_TEST_CASE_P(
  FakeNodeFixtureTests,
  FakeNodeFixtureParametrized,
  // cppcheck-suppress syntaxError  // cppcheck doesn't like the trailing comma.
  ::testing::Values(true, false), /*This comment needed to avoid a warning*/);

/// @test Test that we can use a parametrized test.
TEST_P(FakeNodeFixtureParametrized, Test) {
  run_test(GetParam(), this);
}
