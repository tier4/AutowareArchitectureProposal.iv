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

#ifndef TEST_RELATIVE_LOCALIZER_NODE_HPP_
#define TEST_RELATIVE_LOCALIZER_NODE_HPP_

#include <localization_common/optimized_registration_summary.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <memory>
#include <string>

#include "common/types.hpp"

using autoware::common::types::bool8_t;
using autoware::localization::localization_nodes::RelativeLocalizerNode;
using autoware::localization::localization_nodes::TopicQoS;

using MsgWithHeader = geometry_msgs::msg::TransformStamped;
using TestObservation = MsgWithHeader;
class TestMap
{
public:
  using MapMsg = MsgWithHeader;
  explicit TestMap(const std::shared_ptr<MapMsg> & map_ptr);
  // Interface methods
  void set(const MsgWithHeader & map_msg);

  const std::string & frame_id();

  bool valid();
  // Introspection method for the test:
  const std::shared_ptr<MapMsg> & get_msg_tracker();

  std::shared_ptr<MapMsg> m_msg_ptr;
};

using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using Transform = geometry_msgs::msg::TransformStamped;

constexpr int TEST_ERROR_ID = -9999;
constexpr int INITIAL_ID = -1;

class MockRelativeLocalizer
{
public:
  using Summary = autoware::localization::localization_common::OptimizedRegistrationSummary;
  template<typename ...>
  using MapConstraint = int;

  MockRelativeLocalizer(
    std::shared_ptr<MsgWithHeader> obs_ptr);
  // constructor when the tracking is not needed.
  MockRelativeLocalizer() = default;

  PoseWithCovarianceStamped register_measurement(
    const TestObservation & msg,
    const Transform & transform_initial,
    const TestMap & map,
    Summary * summary = nullptr);

private:
  std::shared_ptr<TestObservation> m_observation_tracking_ptr;
};

class MockInitializer
{
public:
  Transform guess(
    const tf2::BufferCore &, tf2::TimePoint stamp,
    const std::string & id1, const std::string & id2);

  void set_fallback_pose(const geometry_msgs::msg::TransformStamped &);
};

class TestRelativeLocalizerNode : public RelativeLocalizerNode<
    MsgWithHeader,
    MsgWithHeader,
    TestMap,
    MockRelativeLocalizer,
    MockInitializer>
{
public:
  using Base = RelativeLocalizerNode<
    MsgWithHeader,
    MsgWithHeader,
    TestMap,
    MockRelativeLocalizer,
    MockInitializer>;

  using Base::Base;

  // Expose protected methods for convenience
  void set_localizer_(std::unique_ptr<MockRelativeLocalizer> && localizer);
  void set_map_(std::unique_ptr<TestMap> && map);

  bool8_t register_exception();
  bool8_t map_exception();
  bool8_t register_on_invalid_map();

protected:
  void on_bad_registration(std::exception_ptr eptr) override;

  /// Handle the exceptions during map setting.
  void on_bad_map(std::exception_ptr eptr) override;

  void on_observation_with_invalid_map(TestObservation::ConstSharedPtr msg) override;

private:
  bool8_t m_map_exception{false};
  bool8_t m_register_exception{false};
  bool8_t m_register_on_invalid_map{false};
};

/// Wait until publisher reaches desired num. of subscriptions.
template<typename T>
void wait_for_matched(
  const T & pub_ptr,
  const uint32_t num_expected_subs = 1U,
  std::chrono::milliseconds match_timeout = std::chrono::seconds{10U})
{
  const auto match_start = std::chrono::steady_clock::now();
  // Ensure map publisher has a map that is listening.
  while (pub_ptr->get_subscription_count() < num_expected_subs) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() - match_start > match_timeout) {
      throw std::runtime_error("");
    }
  }
}
///// Test exceptions

class TestRegistrationException : public std::exception {};
class TestMapException : public std::exception {};

/// Abstraction functions to tag and track dummy messages.
inline int64_t get_msg_id(const MsgWithHeader & msg)
{
  return msg.transform.translation.x;
}

inline void set_msg_id(MsgWithHeader & msg, int64_t id)
{
  msg.transform.translation.x = id;
}

inline int64_t get_msg_id(const PoseWithCovarianceStamped & msg)
{
  return msg.pose.pose.position.x;
}

inline void set_msg_id(PoseWithCovarianceStamped & msg, int64_t id)
{
  msg.pose.pose.position.x = id;
}

#endif  // TEST_RELATIVE_LOCALIZER_NODE_HPP_
