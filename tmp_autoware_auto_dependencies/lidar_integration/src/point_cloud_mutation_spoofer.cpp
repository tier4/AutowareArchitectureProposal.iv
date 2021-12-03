// Copyright 2018 the Autoware Foundation
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

#include <string>
#include <thread>

#include "lidar_integration/point_cloud_mutation_spoofer.hpp"
#include "lidar_integration/lidar_integration_common.hpp"

using autoware::common::types::char8_t;
using autoware::common::types::float32_t;

namespace lidar_integration
{
PointCloudMutationSpooferNode::PointCloudMutationSpooferNode(
  const char8_t * topic,
  const uint32_t step_mean,
  const uint32_t step_std,
  const uint32_t width_mean,
  const uint32_t width_std,
  const float32_t freq)
: rclcpp::Node("point_cloud_mutation_spoofer"),
  m_dis_width(width_mean, width_std),
  m_dis_row_step(step_mean, step_std),
  m_dis_data(0, 255),
  m_dice(0, 16),
  m_sleep_time(static_cast<uint32_t>(1000UL / freq)),
  m_running(false)
{
  m_mt.seed(m_rd());
  m_pub = create_publisher<sensor_msgs::msg::PointCloud2>(topic, 10);

  std::string frame_id("frameid");
  m_msg.header.frame_id = frame_id;

  m_msg.height = 1U;

  m_msg.fields.resize(4U);
  m_msg.fields[0U].name = "x";
  m_msg.fields[1U].name = "y";
  m_msg.fields[2U].name = "z";
  m_msg.fields[3U].name = "intensity";
  for (uint32_t idx = 0U; idx < 4U; ++idx) {
    m_msg.fields[idx].offset = static_cast<uint32_t>(idx * sizeof(float32_t));
    m_msg.fields[idx].datatype = sensor_msgs::msg::PointField::FLOAT32;
    m_msg.fields[idx].count = 1U;
    m_msg.point_step += static_cast<uint32_t>(sizeof(float32_t));
  }

  m_msg.is_bigendian = false;
  m_msg.is_dense = false;
}

void PointCloudMutationSpooferNode::init()
{
  wait_for_matched(1U, std::chrono::seconds(10));
  LIDAR_INTEGRATION_INFO("Spoofer initialized.");
}

void PointCloudMutationSpooferNode::wait_for_matched(
  const uint32_t num_expected_subs,
  std::chrono::milliseconds match_timeout)
{
  const auto match_start = std::chrono::steady_clock::now();
  while (m_pub->get_subscription_count() < num_expected_subs) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() - match_start > match_timeout) {
      throw std::runtime_error(
              "PointCloudMutationSpooferNode: couldn't match "
              "any subscriptions within the initialization time budget.");
    }
  }
}

void PointCloudMutationSpooferNode::start()
{
  m_running.store(true);
  m_thread = std::thread {[this] {task_function();}};
}

void PointCloudMutationSpooferNode::stop()
{
  m_running.store(false);
  if (m_thread.joinable()) {
    m_thread.join();
  }
}

void PointCloudMutationSpooferNode::task_function()
{
  while (m_running.load(std::memory_order_relaxed)) {
    m_msg.width = static_cast<uint32_t>(m_dis_width(m_mt));

    uint32_t data_length = static_cast<uint32_t>(m_dis_row_step(m_mt));
    m_msg.row_step = data_length;

    uint32_t capacity = m_msg.row_step * m_msg.height;

    m_msg.data.clear();
    m_msg.data.reserve(capacity);

    for (std::size_t i = 0; i < capacity; ++i) {
      m_msg.data.push_back(static_cast<uint8_t>(m_dis_data(m_mt)));
    }

    switch (m_dice(m_mt)) {
      case 0:
        // Shuffle the data randomly.
        std::random_shuffle(m_msg.data.begin(), m_msg.data.end());
        break;
      case 1:
        // Data length is not as described. Equation not holds.
        m_msg.row_step = static_cast<uint32_t>(m_dis_row_step(m_mt));
        break;
      case 2:
        // change in frequency, or simulate communication delay.
        // This will introduce 0~12ms lag.
        std::this_thread::sleep_for(std::chrono::milliseconds(m_dis_data(m_mt) / 20U));
        break;
      case 3:
        // not full scan
        m_msg.data.resize(m_msg.row_step * 3U / 4U);
        break;
      case 4:
        // data from rosbag message in the future?
        break;
      default:
        break;
    }

    this->m_pub->publish(this->m_msg);

    LIDAR_INTEGRATION_INFO("New message is published. m_msg.data.size=%lu", m_msg.data.size());

    std::this_thread::sleep_for(std::chrono::milliseconds(m_sleep_time));
  }
}
}  // namespace lidar_integration
