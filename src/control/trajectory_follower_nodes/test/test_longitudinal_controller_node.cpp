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
using VehicleState = autoware_auto_msgs::msg::VehicleKinematicState;

using FakeNodeFixture = autoware::tools::testing::FakeTestNode;

TEST_F(FakeNodeFixture, simple_test) {
  // Data to test
  LongitudinalCommand::SharedPtr cmd_msg;
  bool received_longitudinal_command = false;

  // Node
  rclcpp::NodeOptions node_options;
  node_options.arguments(
    {"--ros-args", "--params-file", ament_index_cpp::get_package_share_directory(
        "trajectory_follower_nodes") + "/param/longitudinal_controller_defaults.yaml"});
  std::shared_ptr<LongitudinalController> node = std::make_shared<LongitudinalController>(
    node_options);

  // Publisher/Subscribers
  rclcpp::Publisher<VehicleState>::SharedPtr state_pub = node->create_publisher<VehicleState>(
    "input/current_state",
    rclcpp::QoS(10));
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub = node->create_publisher<Trajectory>(
    "input/current_trajectory",
    rclcpp::QoS(10));
  rclcpp::Subscription<LongitudinalCommand>::SharedPtr cmd_sub =
    this->create_subscription<LongitudinalCommand>(
    "output/longitudinal_control_cmd", *node,
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
  VehicleState state;
  state.header.stamp = node->now();
  state.state.longitudinal_velocity_mps = 1.0;
  state_pub->publish(state);
  // the node needs to receive two velocity msg
  rclcpp::spin_some(node);
  rclcpp::spin_some(this->get_fake_node());
  state.header.stamp = node->now();
  state_pub->publish(state);
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
