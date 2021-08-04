// Copyright 2021 Tier IV, Inc.
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


#include <trajectory_follower_nodes/longitudinal_controller_node.hpp>

#include <memory>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "autoware_auto_msgs/msg/trajectory.hpp"
#include "autoware_auto_msgs/msg/longitudinal_command.hpp"
#include "autoware_auto_msgs/msg/vehicle_kinematic_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "fake_test_node/fake_test_node.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "trajectory_follower_test_utils.hpp"

using LongitudinalController =
  autoware::motion::control::trajectory_follower_nodes::LongitudinalController;
using LongitudinalCommand = autoware_auto_msgs::msg::LongitudinalCommand;
using Trajectory = autoware_auto_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_msgs::msg::TrajectoryPoint;
using TwistStamped = geometry_msgs::msg::TwistStamped;

using FakeNodeFixture = autoware::tools::testing::FakeTestNode;

TEST_F(FakeNodeFixture, simple_test) {
  // Data to test
  LongitudinalCommand::SharedPtr cmd_msg;
  bool received_longitudinal_command = false;

  // Node
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("control_rate", 30.0);
  node_options.append_parameter_override("delay_compensation_time", 0.17);
  node_options.append_parameter_override("enable_smooth_stop", true);
  node_options.append_parameter_override("enable_overshoot_emergency", true);
  node_options.append_parameter_override("enable_slope_compensation", false);
  node_options.append_parameter_override("drive_state_stop_dist", 0.5);
  node_options.append_parameter_override("drive_state_offset_stop_dist", 1.0);
  node_options.append_parameter_override("stopping_state_stop_dist", 0.49);
  node_options.append_parameter_override("stopped_state_entry_vel", 0.1);
  node_options.append_parameter_override("stopped_state_entry_acc", 0.1);
  node_options.append_parameter_override("emergency_state_overshoot_stop_dist", 1.5);
  node_options.append_parameter_override("emergency_state_traj_trans_dev", 3.0);
  node_options.append_parameter_override("emergency_state_traj_rot_dev", 0.7);
  node_options.append_parameter_override("kp", 1.0);
  node_options.append_parameter_override("ki", 0.1);
  node_options.append_parameter_override("kd", 0.0);
  node_options.append_parameter_override("max_out", 1.0);
  node_options.append_parameter_override("min_out", -1.0);
  node_options.append_parameter_override("max_p_effort", 1.0);
  node_options.append_parameter_override("min_p_effort", -1.0);
  node_options.append_parameter_override("max_i_effort", 0.3);
  node_options.append_parameter_override("min_i_effort", -0.3);
  node_options.append_parameter_override("max_d_effort", 0.0);
  node_options.append_parameter_override("min_d_effort", 0.0);
  node_options.append_parameter_override("lpf_vel_error_gain", 0.9);
  node_options.append_parameter_override("current_vel_threshold_pid_integration", 0.5);
  node_options.append_parameter_override("smooth_stop_max_strong_acc", -0.5);
  node_options.append_parameter_override("smooth_stop_min_strong_acc", -1.0);
  node_options.append_parameter_override("smooth_stop_weak_acc", -0.3);
  node_options.append_parameter_override("smooth_stop_weak_stop_acc", -0.8);
  node_options.append_parameter_override("smooth_stop_strong_stop_acc", -3.4);
  node_options.append_parameter_override("smooth_stop_max_fast_vel", 0.5);
  node_options.append_parameter_override("smooth_stop_min_running_vel", 0.01);
  node_options.append_parameter_override("smooth_stop_min_running_acc", 0.01);
  node_options.append_parameter_override("smooth_stop_weak_stop_time", 0.8);
  node_options.append_parameter_override("smooth_stop_weak_stop_dist", -0.3);
  node_options.append_parameter_override("smooth_stop_strong_stop_dist", -0.5);
  node_options.append_parameter_override("stopped_vel", 0.0);
  node_options.append_parameter_override("stopped_acc", -3.4);
  node_options.append_parameter_override("stopped_jerk", -5.0);
  node_options.append_parameter_override("emergency_vel", 0.0);
  node_options.append_parameter_override("emergency_acc", -5.0);
  node_options.append_parameter_override("emergency_jerk", -3.0);
  node_options.append_parameter_override("max_acc", 3.0);
  node_options.append_parameter_override("min_acc", -5.0);
  node_options.append_parameter_override("max_jerk", 2.0);
  node_options.append_parameter_override("min_jerk", -5.0);
  node_options.append_parameter_override("use_trajectory_for_pitch_calculation", false);
  node_options.append_parameter_override("lpf_pitch_gain", 0.95);
  node_options.append_parameter_override("max_pitch_rad", 0.1);
  node_options.append_parameter_override("min_pitch_rad", -0.1);
  std::shared_ptr<LongitudinalController> node = std::make_shared<LongitudinalController>(
    node_options);

  // Publisher/Subscribers
  rclcpp::Publisher<TwistStamped>::SharedPtr twist_pub = node->create_publisher<TwistStamped>(
    "input/current_velocity",
    rclcpp::QoS(10));
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub = node->create_publisher<Trajectory>(
    "input/current_trajectory",
    rclcpp::QoS(10));
  rclcpp::Subscription<LongitudinalCommand>::SharedPtr cmd_sub =
    this->create_subscription<LongitudinalCommand>(
    "output/control_cmd", *node,
    [&cmd_msg, &received_longitudinal_command](const LongitudinalCommand::SharedPtr msg) {
      cmd_msg = msg; received_longitudinal_command = true;
    });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  // Enable all logging in the node
  auto ret = rcutils_logging_set_logger_level(
    node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK) {std::cout << "Failed to set logging severerity to DEBUG\n";}

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);
  /// Already running at target vel + Non stopping trajectory -> no change in velocity
  // Publish velocity
  TwistStamped twist;
  twist.header.stamp = node->now();
  twist.twist.linear.x = 1.0;
  twist_pub->publish(twist);
  // the node needs to receive two velocity msg
  rclcpp::spin_some(node);
  rclcpp::spin_some(this->get_fake_node());
  twist.header.stamp = node->now();
  twist_pub->publish(twist);
  // Publish non stopping trajectory
  Trajectory traj;
  TrajectoryPoint point;
  point.x = 0.0;
  point.y = 0.0;
  point.longitudinal_velocity_mps = 1.0;
  traj.points.push_back(point);
  point.x = 50.0;
  point.y = 0.0;
  point.longitudinal_velocity_mps = 1.0;
  traj.points.push_back(point);
  point.x = 100.0;
  point.y = 0.0;
  point.longitudinal_velocity_mps = 1.0;
  traj.points.push_back(point);
  traj_pub->publish(traj);
  test_utils::waitForMessage(node, this, received_longitudinal_command);

  ASSERT_TRUE(received_longitudinal_command);
  EXPECT_DOUBLE_EQ(cmd_msg->speed, 1.0);
  EXPECT_DOUBLE_EQ(cmd_msg->acceleration, 0.0);
}
