// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <vector>

#include "trajectory_follower_nodes/latlon_muxer_node.hpp"

#include "autoware_auto_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_msgs/msg/longitudinal_command.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "gtest/gtest.h"

using LatLonMuxer = autoware::motion::control::trajectory_follower_nodes::LatLonMuxer;
using LateralCommand = autoware_auto_msgs::msg::AckermannLateralCommand;
using LongitudinalCommand = autoware_auto_msgs::msg::LongitudinalCommand;
using ControlCommand = autoware_auto_msgs::msg::AckermannControlCommand;

class TestROS : public ::testing::Test
{
protected:
  std::shared_ptr<LatLonMuxer> m_node;

  rclcpp::Publisher<LateralCommand>::SharedPtr m_lat_pub;
  rclcpp::Publisher<LongitudinalCommand>::SharedPtr m_lon_pub;
  rclcpp::Subscription<ControlCommand>::SharedPtr m_cmd_sub;

  ControlCommand m_cmd_msg;
  bool m_received_combined_command = false;

  void SetUp()
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.append_parameter_override("timeout_thr_sec", 0.5);
    m_node = std::make_shared<LatLonMuxer>(node_options);

    m_lat_pub = m_node->create_publisher<LateralCommand>(
      "input/lateral/control_cmd",
      rclcpp::QoS(10));
    m_lon_pub = m_node->create_publisher<LongitudinalCommand>(
      "input/longitudinal/control_cmd",
      rclcpp::QoS(10));
    m_cmd_sub = m_node->create_subscription<ControlCommand>(
      "output/control_cmd",
      rclcpp::QoS(10), std::bind(&TestROS::HandleOutputCommand, this, std::placeholders::_1));
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

  void HandleOutputCommand(const ControlCommand::SharedPtr combined_msg)
  {
    m_cmd_msg = *combined_msg;
    m_received_combined_command = true;
  }
};

const rclcpp::Duration one_second(1, 0);

// TODO(Maxime CLEMENT): tests are flacky [issue #1189]
TEST_F(TestROS, DISABLED_test_correct_output)
{
  // Prepare messages
  LateralCommand lat_msg;
  LongitudinalCommand lon_msg;
  lat_msg.steering_tire_angle = 1.5;
  lat_msg.steering_tire_rotation_rate = 0.2f;
  lon_msg.speed = 5.0;
  lon_msg.acceleration = -1.0;
  lon_msg.jerk = 0.25;
  // Publish messages
  lat_msg.stamp = m_node->now();
  lon_msg.stamp = m_node->now();
  m_lat_pub->publish(lat_msg);
  m_lon_pub->publish(lon_msg);
  rclcpp::spin_some(m_node);
  rclcpp::spin_some(m_node);
  // Ensure the combined control command was published and contains correct data
  ASSERT_TRUE(m_received_combined_command);
  ASSERT_EQ(m_cmd_msg.lateral.steering_tire_angle, lat_msg.steering_tire_angle);
  ASSERT_EQ(m_cmd_msg.lateral.steering_tire_rotation_rate, lat_msg.steering_tire_rotation_rate);
  ASSERT_EQ(m_cmd_msg.longitudinal.speed, lon_msg.speed);
  ASSERT_EQ(m_cmd_msg.longitudinal.acceleration, lon_msg.acceleration);
  ASSERT_EQ(m_cmd_msg.longitudinal.jerk, lon_msg.jerk);
  ASSERT_GT(rclcpp::Time(m_cmd_msg.stamp), rclcpp::Time(lat_msg.stamp));
  ASSERT_GT(rclcpp::Time(m_cmd_msg.stamp), rclcpp::Time(lon_msg.stamp));
}

TEST_F(TestROS, DISABLED_test_lateral_timeout)
{
  // Prepare empty messages
  LateralCommand lat_msg;
  LongitudinalCommand lon_msg;
  // Generate a timeout of the lateral message
  lat_msg.stamp = m_node->now() - one_second;
  lon_msg.stamp = m_node->now();
  m_lat_pub->publish(lat_msg);
  m_lon_pub->publish(lon_msg);
  rclcpp::spin_some(m_node);
  rclcpp::spin_some(m_node);
  // Ensure the inputs were not combined
  ASSERT_FALSE(m_received_combined_command);
}

TEST_F(TestROS, DISABLED_test_longitudinal_timeout)
{
  // Prepare empty messages
  LateralCommand lat_msg;
  LongitudinalCommand lon_msg;
  // Generate a timeout of the longitudinal message
  lat_msg.stamp = m_node->now();
  lon_msg.stamp = m_node->now() - one_second;
  m_lat_pub->publish(lat_msg);
  m_lon_pub->publish(lon_msg);
  rclcpp::spin_some(m_node);
  rclcpp::spin_some(m_node);
  // Ensure the inputs were not combined
  ASSERT_FALSE(m_received_combined_command);
}
TEST_F(TestROS, DISABLED_test_latlon_timeout)
{
  // Prepare empty messages
  LateralCommand lat_msg;
  LongitudinalCommand lon_msg;
  // Generate a timeout of both messages
  m_node->set_parameter(rclcpp::Parameter("timeout_thr_sec", 0.5));
  lat_msg.stamp = m_node->now() - one_second;
  lon_msg.stamp = m_node->now() - one_second;
  m_lat_pub->publish(lat_msg);
  m_lon_pub->publish(lon_msg);
  rclcpp::spin_some(m_node);
  rclcpp::spin_some(m_node);
  // Ensure the inputs were not combined
  ASSERT_FALSE(m_received_combined_command);
}
