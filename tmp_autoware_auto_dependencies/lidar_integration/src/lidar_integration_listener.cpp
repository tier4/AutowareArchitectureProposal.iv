// Copyright 2017-2018 the Autoware Foundation
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

#include <common/types.hpp>
#include <cmath>
#include <string>
#include "lidar_integration/lidar_integration_listener.hpp"

namespace lidar_integration
{

using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

void LidarIntegrationListener::init_statistics(Statistics & stats)
{
  stats.success = false;
  stats.last_failed = false;
  // TODO(andreas.pasternak) Hacks to discern invalid data. Should be an optional?
  stats.first_pub_time = decltype(stats.first_pub_time)::max();
  stats.last_pub_time = decltype(stats.last_pub_time)::max();
  stats.count = 0U;
  stats.total_size = 0U;
}

void LidarIntegrationListener::callback(const uint32_t size)
{
  if (m_stats.last_failed) {
    RCLCPP_ERROR(get_logger(), "nonterminal fail");
    m_stats.success = false;
    m_stats.last_failed = false;
  }
  const auto right_now = std::chrono::steady_clock::now();
  // check periodicity
  if (decltype(m_stats.first_pub_time)::max() == m_stats.first_pub_time) {
    // got at least one message!
    m_stats.success = true;
    m_stats.first_pub_time = right_now;
  }
  // check size
  m_stats.total_size += size;
  m_stats.last_pub_time = right_now;
  ++m_stats.count;
  RCLCPP_INFO(get_logger(), "callback. count is now: %u", m_stats.count);
}

LidarIntegrationListener::LidarIntegrationListener(
  const std::string & name,
  const float32_t expected_period_ms,
  const uint32_t expected_size,
  const float32_t relative_tolerance_period,
  const float32_t relative_tolerance_size)
: Node(name),
  m_expected_period_us(expected_period_ms * 1000.0F),
  m_relative_tolerance(relative_tolerance_period),
  m_relative_size_tolerance(relative_tolerance_size),
  m_expected_size(expected_size)
{
  init_statistics(m_stats);

  RCLCPP_INFO(
    get_logger(), "\texpected_period_ms: %d",
    static_cast<int32_t>(expected_period_ms * 1000.0F));
  RCLCPP_INFO(get_logger(), "\texpected_size: %u", expected_size);

  RCLCPP_INFO(
    get_logger(), "\trelative_tolerance_period: %f",
    static_cast<float64_t>(relative_tolerance_period));
  RCLCPP_INFO(
    get_logger(), "\trelative_tolerance_size: %f",
    static_cast<float64_t>(relative_tolerance_size));

  RCLCPP_INFO(get_logger(), "LidarIntegrationListener initialized. ");
}

LidarIntegrationListener::~LidarIntegrationListener()
{
}

bool8_t LidarIntegrationListener::has_valid_period(const Statistics & stats) const
{
  const float32_t sample_duration_us =
    std::chrono::duration_cast<std::chrono::duration<float32_t, std::micro>>(
    stats.last_pub_time - stats.first_pub_time).count();
  bool8_t ret = false;
  if (stats.count > 1U) {
    const float32_t sample_period_us =
      sample_duration_us / (static_cast<float32_t>(stats.count) - 1.0F);
    ret = std::abs(sample_period_us - m_expected_period_us) <
      m_relative_tolerance * m_expected_period_us;

    RCLCPP_INFO(get_logger(), "Period: %0.2f", static_cast<float64_t>(sample_period_us));
    RCLCPP_INFO(get_logger(), "Count: %u", stats.count);
    RCLCPP_INFO(
      get_logger(), "Period difference abs: %0.2f",
      static_cast<float64_t>(std::abs(sample_period_us - m_expected_period_us)));
    RCLCPP_INFO(
      get_logger(), "Period difference tolerance: %0.2f",
      static_cast<float64_t>(m_relative_tolerance * m_expected_period_us));

    printf("Sample Duration: %0.0f\n", static_cast<float64_t>(sample_duration_us));
    printf("Period: %0.1f\n", static_cast<float64_t>(sample_period_us));
    printf("Count: %0.0f\n", static_cast<float64_t>(stats.count));
    printf(
      "Period difference abs:       %0.1f\n",
      static_cast<float64_t>(std::abs(sample_period_us - m_expected_period_us)));
    printf(
      "Period difference tolerance: %0.1f\n",
      static_cast<float64_t>(m_relative_tolerance * m_expected_period_us));
  }

  return ret;
}

bool8_t LidarIntegrationListener::has_valid_size(
  const Statistics & stats, const uint32_t expected_size) const
{
  bool8_t ret = false;
  if (stats.count > 1U) {
    const float32_t mean_size =
      static_cast<float32_t>(stats.total_size) / (static_cast<float32_t>(stats.count) - 1.0F);

    RCLCPP_INFO(get_logger(), "Mean size: %0.2f", static_cast<float64_t>(mean_size));
    RCLCPP_INFO(
      get_logger(), "Error: %0.2f",
      static_cast<float64_t>(std::abs(mean_size - static_cast<float32_t>(expected_size))));
    RCLCPP_INFO(
      get_logger(), "Size difference tolerance: %0.2f",
      static_cast<float64_t>(m_relative_size_tolerance * static_cast<float32_t>(expected_size)));
    RCLCPP_INFO(
      get_logger(), "Actual size difference: %0.2f", static_cast<float64_t>(std::abs(
        mean_size - static_cast<float32_t>(expected_size))));

    printf("Mean size: %0.2f\n", static_cast<float64_t>(mean_size));
    printf(
      "Size difference tolerance: %0.2f\n",
      static_cast<float64_t>(m_relative_size_tolerance * static_cast<float32_t>(expected_size)));
    printf(
      "Actual size difference:    %0.2f\n",
      static_cast<float64_t>(std::abs(mean_size - static_cast<float32_t>(expected_size))));

    ret = std::abs(mean_size - static_cast<float32_t>(expected_size)) <
      m_relative_size_tolerance * static_cast<float32_t>(expected_size);
  }

  return ret;
}

bool8_t LidarIntegrationListener::is_success(
  const rclcpp::SubscriptionBase * const sub_ptr,
  const char8_t * const src) const
{
  if (sub_ptr != nullptr) {
    console_statistics(m_stats, src);
  }

  return (sub_ptr == nullptr) ||
         (has_valid_period(m_stats) &
         m_stats.success &
         has_valid_size(m_stats, m_expected_size));
}

void LidarIntegrationListener::console_statistics(
  const Statistics & stat, const char8_t * src) const
{
  RCLCPP_INFO(get_logger(), ("Statistics of " + std::string(src)).c_str());
  RCLCPP_INFO(get_logger(), "\tsuccess: %s", stat.success ? "true" : "false");
  RCLCPP_INFO(get_logger(), "\tlast_failed: %s", stat.last_failed ? "true" : "false");
  RCLCPP_INFO(get_logger(), "\ttotal_size: %u", stat.total_size);
  RCLCPP_INFO(get_logger(), "\tcount: %u", stat.count);
  std::cout << "Statistics of " << src << "\n";
  std::cout << "\ttotal_size: " << stat.total_size << "\n";
  std::cout << "\tcount: " << stat.count << "\n";
}

////////////////////////////////////////////////////////////////////////////////
LidarIntegrationPclListener::LidarIntegrationPclListener(
  const std::string & topic,
  const float32_t expected_period_ms,
  const uint32_t expected_size,
  const float32_t relative_tolerance_period,
  const float32_t relative_tolerance_size,
  const std::string & name)
: LidarIntegrationListener{
    name,
    expected_period_ms,
    expected_size,
    relative_tolerance_period,
    relative_tolerance_size},
  m_sub_ptr{create_subscription<PointCloud2>(
      topic, rclcpp::QoS(rclcpp::KeepLast(20)),
      [this](const PointCloud2::SharedPtr msg_ptr) {
        this->callback(msg_ptr->width);
        RCLCPP_INFO(get_logger(), "\tdata length: %lu", msg_ptr->data.size());
      })}
{
  RCLCPP_INFO(get_logger(), ("\tpcl_topic1: " + topic).c_str());
}
bool8_t LidarIntegrationPclListener::is_success() const
{
  return LidarIntegrationListener::is_success(m_sub_ptr.get(), "pcl1");
}
////////////////////////////////////////////////////////////////////////////////
LidarIntegrationBoxListener::LidarIntegrationBoxListener(
  const std::string & topic,
  const float32_t expected_period_ms,
  const uint32_t expected_size,
  const float32_t relative_tolerance_period,
  const float32_t relative_tolerance_size,
  const std::string & name)
: LidarIntegrationListener{
    name,
    expected_period_ms,
    expected_size,
    relative_tolerance_period,
    relative_tolerance_size},
  m_sub_ptr{create_subscription<BoundingBoxArray>(
      topic, rclcpp::QoS(rclcpp::KeepLast(20)),
      [this](const BoundingBoxArray::SharedPtr msg_ptr) {
        this->callback(static_cast<uint32_t>(msg_ptr->boxes.size()));
      })}
{
  RCLCPP_INFO(get_logger(), ("\tbox_topic: " + topic).c_str());
}
bool8_t LidarIntegrationBoxListener::is_success() const
{
  return LidarIntegrationListener::is_success(m_sub_ptr.get(), "box");
}

}  // namespace lidar_integration
