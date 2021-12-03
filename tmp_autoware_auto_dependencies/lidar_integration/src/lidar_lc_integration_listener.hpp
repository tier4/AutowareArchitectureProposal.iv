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


#ifndef LIDAR_LC_INTEGRATION_LISTENER_HPP_
#define LIDAR_LC_INTEGRATION_LISTENER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <common/types.hpp>
#include <lidar_integration/lidar_integration_common.hpp>
#include <string>
#include <memory>

namespace lidar_integration
{

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;

class Lcm : public rclcpp::Node
{
public:
  explicit Lcm(const std::string & node_name)
  : Node(node_name)
  {}

  ~Lcm()
  {
    if (m_thread.joinable()) {
      m_running.store(false);
      m_thread.join();
    }
  }

  void start()
  {
    m_running.store(true);
    m_thread = std::thread{[this] {task_function();}};
  }

  void init()
  {
    client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
      node_get_state_topic);
    client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
      node_change_state_topic);
    const char8_t * const driver_action_topic = "lidar_driver_action";
    const char8_t * const fuser_action_topic = "lidar_fuser_action";
    driver_action_pub_ptr =
      this->create_publisher<std_msgs::msg::UInt8MultiArray>(driver_action_topic, 10);
    fuser_action_pub_ptr =
      this->create_publisher<std_msgs::msg::UInt8MultiArray>(fuser_action_topic, 10);
  }
  uint8_t get_state()
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    // We send the service request for asking the current
    auto future_result = client_get_state_->async_send_request(request);

    if (future_result.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
      LIDAR_INTEGRATION_ERROR("Server time out while getting current state.");
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We have an succesful answer.
    if (future_result.get()) {
      return future_result.get()->current_state.id;
    } else {
      LIDAR_INTEGRATION_ERROR("Failed to get current state.");

      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }

  void change_state(std::uint8_t transition)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    // We send the request with the transition we want to invoke.
    auto future_result = client_change_state_->async_send_request(request);

    if (future_result.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      auto * lifecycle_node = "lidar_detector_managed";
      LIDAR_INTEGRATION_ERROR(
        "Server time out while getting current state for node: %s",
        lifecycle_node);
    }

    // if its not sucess
    if (!future_result.get()->success) {
      LIDAR_INTEGRATION_ERROR("Failed to trigger transition %u", static_cast<uint32_t>(transition));
    }
  }

  void publish_actions()
  {
    // start the drivers
    std_msgs::msg::UInt8MultiArray msg;
    msg.data.resize(2U);
    msg.data[0] = 1U;  // restart
    msg.data[1] = 0U;  // nothing
    driver_action_pub_ptr->publish(msg);
    msg.data[0] = 0U;  // nothing
    msg.data[1] = 1U;  // restart
    driver_action_pub_ptr->publish(msg);

    // revive fusers
    msg.data[0] = 3U;  // revive
    msg.data[1] = 0U;  // nothing
    fuser_action_pub_ptr->publish(msg);
    msg.data[0] = 0U;  // nothing
    msg.data[1] = 3U;  // revive
    fuser_action_pub_ptr->publish(msg);
  }

private:
  /// \brief Fills BoundingBoxArray message with boxes, and publishes
  void task_function()
  {
    using namespace std::chrono_literals;  // NOLINT

    auto count = 0;
    auto once = true;
    while (m_running.load(std::memory_order_relaxed)) {
      if (count == 5) {
        // configure (will be done by ros2 launch)
        change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      } else if (count == 10) {
        // activate (will be done by ros2 launch)
        change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
      } else if (once) {
        if (get_state() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          publish_actions();
          // reset the flag
          once = false;
        }
      }
      count++;

      std::this_thread::sleep_for(10ms);
    }
  }

  static constexpr char8_t const * node_get_state_topic = "/lidar_detector_managed/get_state";
  static constexpr char8_t const * node_change_state_topic = "/lidar_detector_managed/change_state";

  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>> driver_action_pub_ptr;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>> fuser_action_pub_ptr;

  std::atomic_bool m_running{false};
  std::thread m_thread;
};
}  // namespace lidar_integration
#endif  // LIDAR_LC_INTEGRATION_LISTENER_HPP_
