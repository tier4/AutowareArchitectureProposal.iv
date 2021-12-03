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


#include <localization_nodes/localization_node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>
#include <memory>
#include <utility>

#include "test_relative_localizer_node.hpp"

#include "common/types.hpp"

using autoware::common::types::bool8_t;

class RelativeLocalizationNodeTest : public ::testing::Test
{
public:
  RelativeLocalizationNodeTest()
  {
    m_observation_msg.header.frame_id = m_obs_frame;
    m_map_msg.header.frame_id = m_map_frame;
    // output of `register_measurement` has the frame ID:
    // output_frame_id = observation.frame_id + initial_guess.frame_id
    // output_frame_id = observation.frame_id + (observation.frame_id + map.frame_id)
    m_expected_aggregated_frame = m_obs_frame + m_obs_frame + m_map_frame;
  }
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    // TODO(yunus.caliskan): add assertions!
  }
  void TearDown()
  {
    rclcpp::shutdown();
  }

protected:
  const std::string m_observation_topic{"test_obs"};
  const std::string m_map_topic{"test_map"};
  const std::string m_out_topic{"test_pose_out"};
  const std::string m_init_pose_topic{"test_pose_init"};
  const uint32_t m_history_depth{10U};
  const std::string m_map_frame{"map"};
  const std::string m_obs_frame{"obs"};
  std::string m_expected_aggregated_frame;
  TestObservation m_observation_msg;
  MsgWithHeader m_map_msg;
};


TEST_F(RelativeLocalizationNodeTest, Basic) {
  ////////////////////////////// Define lambdas

  /// Spin until tracker pointer reaches
  auto spin_until_tracker_match =
    [](auto & node_ptr, auto & ptr, auto msg_id, auto max_poll_iters) {
      for (auto iter = 0U; (iter < max_poll_iters) && (get_msg_id(*ptr) != msg_id); ++iter) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        rclcpp::spin_some(node_ptr);
      }
    };

  /////////////////////// initialize
  constexpr auto initial_ID = INITIAL_ID;
  auto cur_map_id = initial_ID;
  auto cur_obs_id = initial_ID;
  const auto max_poll_iters = 50U;
  auto callback_called = false;

  // Create pointers to inject into the node to track its state.
  auto observation_tracker_ptr = std::make_shared<TestObservation>();
  auto map_tracker_ptr = std::make_shared<MsgWithHeader>();
  // Tag the pointers with the initial id
  set_msg_id(*observation_tracker_ptr, initial_ID);
  set_msg_id(*map_tracker_ptr, initial_ID);

  // Initialize localizer node
  auto localizer_ptr = std::make_unique<MockRelativeLocalizer>(
    observation_tracker_ptr);
  auto map_ptr = std::make_unique<TestMap>(map_tracker_ptr);

  auto localizer_node = std::make_shared<TestRelativeLocalizerNode>(
    "TestNode", "",
    TopicQoS{m_observation_topic, rclcpp::SystemDefaultsQoS{}},
    TopicQoS{m_map_topic, rclcpp::SystemDefaultsQoS{}},
    TopicQoS{m_out_topic, rclcpp::SystemDefaultsQoS{}},
    TopicQoS{m_init_pose_topic, rclcpp::SystemDefaultsQoS{}},
    MockInitializer{});

  localizer_node->set_localizer_(std::move(localizer_ptr));
  localizer_node->set_map_(std::move(map_ptr));


  // Create mock observation and map publishers.
  const auto observation_pub = localizer_node->create_publisher<TestObservation>(
    m_observation_topic,
    m_history_depth);
  const auto map_pub =
    localizer_node->create_publisher<MsgWithHeader>(m_map_topic, m_history_depth);

  // Create a subscription to get the output from the localizer node. The callback compares the
  // ID of the output to the last published
  const auto pose_out_sub = localizer_node->create_subscription<PoseWithCovarianceStamped>(
    m_out_topic,
    rclcpp::QoS{rclcpp::KeepLast{m_history_depth}},
    [this, &callback_called, &cur_obs_id](PoseWithCovarianceStamped::ConstSharedPtr pose) {
      // Check that the result has the expected frame ID.
      // See the initiallization of m_expected_aggregated_frame for an explanation.
      EXPECT_EQ(pose->header.frame_id, m_expected_aggregated_frame);
      // Compare the ID to the id of the last published observation.
      EXPECT_EQ(get_msg_id(*pose), cur_obs_id);
      callback_called = true;
    });

  // Wait until publishers have a subscription available.
  wait_for_matched(map_pub);
  wait_for_matched(observation_pub);
  wait_for_matched(localizer_node->get_publisher());

  ////////////////////////// Test routine:
  // * publish a map on 0th and 5th iterations
  // * publish an observation on each iteration.
  // At each iteration, injected tracker pointers are used to confirm that the localizer node
  // receives the published messages and forwarded them to the correct places in the localizer.
  // At the end of each iteration, the output of the localizer node is checked for confirmation.
  for (auto i = 0; i < 10; ++i) {
    ASSERT_NE(i, TEST_ERROR_ID);
    // Only publish maps every 5th message, starting from 0.
    if (i % 5U == 0U) {
      cur_map_id = i;
      ASSERT_NE(cur_map_id, initial_ID);
      set_msg_id(m_map_msg, cur_map_id);
      map_pub->publish(m_map_msg);
    }
    // Wait until correct map is received by the localizer.
    spin_until_tracker_match(localizer_node, map_tracker_ptr, cur_map_id, max_poll_iters);

    // Confirm map is correct.
    EXPECT_EQ(get_msg_id(*map_tracker_ptr), cur_map_id);

    // Publish a new observation with a new ID
    cur_obs_id = i;
    ASSERT_NE(cur_obs_id, initial_ID);
    set_msg_id(m_observation_msg, cur_obs_id);
    observation_pub->publish(m_observation_msg);

    // Wait until correct observation is received by the localizer
    spin_until_tracker_match(localizer_node, observation_tracker_ptr, cur_obs_id, max_poll_iters);

    EXPECT_EQ(get_msg_id(*observation_tracker_ptr), cur_obs_id);

    for (auto iter = 0U; (iter < max_poll_iters) && !callback_called; ++iter) {
      rclcpp::spin_some(localizer_node);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    EXPECT_TRUE(callback_called);
    callback_called = false;

    EXPECT_FALSE(localizer_node->map_exception());
    EXPECT_FALSE(localizer_node->register_exception());
  }
}

TEST_F(RelativeLocalizationNodeTest, ExceptionHandling) {
  ////////////////////////////// Define lambdas
  /// Spin until condition for node is met
  auto spin_until_condition =
    [](auto & node_ptr, auto checker, auto max_poll_iters) {
      for (auto iter = 0U; (iter < max_poll_iters) && !checker(node_ptr); ++iter) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        rclcpp::spin_some(node_ptr);
      }
    };

  /////////////////////// initialize
  constexpr auto initial_id = INITIAL_ID;
  constexpr auto valid_map_id = 0;
  const auto max_poll_iters = 50U;

  // Create pointers to inject into the node to track its state.
  auto map_tracker_ptr = std::make_shared<MsgWithHeader>();
  set_msg_id(*map_tracker_ptr, initial_id);

  ASSERT_NE(initial_id, valid_map_id);
  ASSERT_NE(initial_id, TEST_ERROR_ID);
  ASSERT_NE(valid_map_id, TEST_ERROR_ID);

  // Initialize localizer node
  auto localizer_ptr = std::make_unique<MockRelativeLocalizer>(nullptr);
  auto map_ptr = std::make_unique<TestMap>(map_tracker_ptr);
  auto localizer_node = std::make_shared<TestRelativeLocalizerNode>(
    "TestNode", "",
    TopicQoS{m_observation_topic, rclcpp::SystemDefaultsQoS{}},
    TopicQoS{m_map_topic, rclcpp::SystemDefaultsQoS{}},
    TopicQoS{m_out_topic, rclcpp::SystemDefaultsQoS{}},
    TopicQoS{m_init_pose_topic, rclcpp::SystemDefaultsQoS{}},
    MockInitializer{});

  localizer_node->set_localizer_(std::move(localizer_ptr));
  localizer_node->set_map_(std::move(map_ptr));

  // Create mock observation and map publishers.
  const auto observation_pub = localizer_node->create_publisher<TestObservation>(
    m_observation_topic, m_history_depth);
  const auto map_pub =
    localizer_node->create_publisher<MsgWithHeader>(m_map_topic, m_history_depth);

  // Wait until publishers have a subscription available.
  wait_for_matched(map_pub);
  wait_for_matched(observation_pub);

  set_msg_id(m_map_msg, TEST_ERROR_ID);
  map_pub->publish(m_map_msg);
  // Wait until map exception occurs.
  spin_until_condition(
    localizer_node, [](auto loc_nd_ptr) {return loc_nd_ptr->map_exception();},
    max_poll_iters);
  EXPECT_TRUE(localizer_node->map_exception());

  set_msg_id(m_observation_msg, TEST_ERROR_ID);
  observation_pub->publish(m_observation_msg);
  // run until observation is attempted to be registered when no valid map exists.
  spin_until_condition(
    localizer_node, [](auto loc_nd_ptr) {
      return loc_nd_ptr->register_on_invalid_map();
    }, max_poll_iters);
  // no exception will be thrown despite the registration is bad because no map is set yet.
  EXPECT_FALSE(localizer_node->register_exception());
  // Confirm that a registration was received with no valid map.
  EXPECT_TRUE(localizer_node->register_on_invalid_map());

  // Now we will set the node to a state where it has a valid map and try again.
  set_msg_id(m_map_msg, valid_map_id);
  map_pub->publish(m_map_msg);
  spin_until_condition(
    localizer_node, [map_tracker_ptr, valid_map_id](auto &) {
      return get_msg_id(*map_tracker_ptr) == valid_map_id;
    }, max_poll_iters);

  observation_pub->publish(m_observation_msg);
  spin_until_condition(
    localizer_node, [](auto loc_nd_ptr) {
      return loc_nd_ptr->register_exception();
    }, max_poll_iters);
  EXPECT_TRUE(localizer_node->register_exception());
}

//////////////////////////////////////////////////////////////////////// Implementations

TestMap::TestMap(const std::shared_ptr<MapMsg> & map_ptr)
: m_msg_ptr(map_ptr) {}

void TestMap::set(const MsgWithHeader & map_msg)
{
  if (get_msg_id(map_msg) == TEST_ERROR_ID) {
    throw TestMapException{};
  }
  *m_msg_ptr = map_msg;
}

const std::string & TestMap::frame_id()
{
  return m_msg_ptr->header.frame_id;
}

bool TestMap::valid()
{
  const auto id = get_msg_id(*m_msg_ptr);
  return (id != INITIAL_ID) && (id != TEST_ERROR_ID);
}

const std::shared_ptr<MsgWithHeader> & TestMap::get_msg_tracker()
{
  return m_msg_ptr;
}


void TestRelativeLocalizerNode::set_localizer_(std::unique_ptr<MockRelativeLocalizer> && localizer)
{
  set_localizer(std::forward<std::unique_ptr<MockRelativeLocalizer>>(localizer));
}

void TestRelativeLocalizerNode::set_map_(std::unique_ptr<TestMap> && map)
{
  set_map(std::forward<std::unique_ptr<TestMap>>(map));
}

MockRelativeLocalizer::MockRelativeLocalizer(
  std::shared_ptr<MsgWithHeader> obs_ptr)
: m_observation_tracking_ptr{obs_ptr} {}

PoseWithCovarianceStamped MockRelativeLocalizer::register_measurement(
  const TestObservation & msg, const Transform & transform_initial,
  const TestMap &,
  Summary *)
{
  if (get_msg_id(msg) == TEST_ERROR_ID) {
    throw TestRegistrationException{};
  }
  PoseWithCovarianceStamped pose_out;
  // The resulting frame id should contain observation's frame + initial guess' frame ID
  // So the result should be: obs_frame + obs_frame + map_frame
  pose_out.header.frame_id = msg.header.frame_id + transform_initial.header.frame_id;
  set_msg_id(pose_out, get_msg_id(msg));

  // Update the tracking pointer for notifying the test.
  if (m_observation_tracking_ptr) {
    *m_observation_tracking_ptr = msg;
  }
  return pose_out;
}

void TestRelativeLocalizerNode::on_bad_registration(std::exception_ptr eptr)
{
  try {
    std::rethrow_exception(eptr);
  } catch (TestRegistrationException &) {
    m_register_exception = true;
  }
}

void TestRelativeLocalizerNode::on_bad_map(std::exception_ptr eptr)
{
  try {
    std::rethrow_exception(eptr);
  } catch (TestMapException &) {
    m_map_exception = true;
  }
}

bool8_t TestRelativeLocalizerNode::register_exception()
{
  return m_register_exception;
}
bool8_t TestRelativeLocalizerNode::map_exception()
{
  return m_map_exception;
}

bool TestRelativeLocalizerNode::register_on_invalid_map()
{
  return m_register_on_invalid_map;
}

void TestRelativeLocalizerNode::on_observation_with_invalid_map(
  TestObservation::ConstSharedPtr)
{
  m_register_on_invalid_map = true;
}

// Return a transform that contains information regarding two frame ids.
// The resulting frame id should contain: obs_frame + map_frame.
Transform MockInitializer::guess(
  const tf2::BufferCore &, tf2::TimePoint stamp,
  const std::string & map_frame, const std::string & obs_frame)
{
  Transform transform;
  transform.header.stamp = ::time_utils::to_message(stamp);
  transform.header.frame_id = obs_frame + map_frame;
  return transform;
}

void MockInitializer::set_fallback_pose(const geometry_msgs::msg::TransformStamped &) {}
