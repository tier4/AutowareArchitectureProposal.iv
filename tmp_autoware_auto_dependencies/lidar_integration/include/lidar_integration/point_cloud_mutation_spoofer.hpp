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
#ifndef LIDAR_INTEGRATION__POINT_CLOUD_MUTATION_SPOOFER_HPP_
#define LIDAR_INTEGRATION__POINT_CLOUD_MUTATION_SPOOFER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <common/types.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <lidar_integration/visibility_control.hpp>

#include <memory>
#include <random>

namespace lidar_integration
{
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;

/// \brief Spoofs random point clouds which may or may not be properly formatted.
/// Intended for use with mutation testing
class LIDAR_INTEGRATION_PUBLIC PointCloudMutationSpooferNode : public rclcpp::Node
{
public:
  PointCloudMutationSpooferNode(
    const char8_t * topic,
    const uint32_t step_mean,
    const uint32_t step_std,
    const uint32_t width_mean,
    const uint32_t width_std,
    const float32_t freq);

  void init();

  void start();

  void stop();

protected:
  void wait_for_matched(
    const uint32_t num_expected_subs,
    std::chrono::milliseconds match_timeout);

  void task_function();

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pub;
  sensor_msgs::msg::PointCloud2 m_msg;

  std::mt19937 m_mt;
  std::random_device m_rd;
  std::normal_distribution<> m_dis_width;
  std::normal_distribution<> m_dis_row_step;
  std::uniform_int_distribution<uint16_t> m_dis_data;
  std::uniform_int_distribution<uint16_t> m_dice;

  uint32_t m_sleep_time;
  std::thread m_thread;
  std::atomic_bool m_running;
};  // Spoofer
}  // namespace lidar_integration
#endif  // LIDAR_INTEGRATION__POINT_CLOUD_MUTATION_SPOOFER_HPP_
