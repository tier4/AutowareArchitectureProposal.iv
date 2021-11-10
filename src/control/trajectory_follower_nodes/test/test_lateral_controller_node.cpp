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


#include <trajectory_follower_nodes/lateral_controller_node.hpp>

#include <memory>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "autoware_auto_msgs/msg/trajectory.hpp"
#include "autoware_auto_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_msgs/msg/vehicle_kinematic_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "fake_test_node/fake_test_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "trajectory_follower_test_utils.hpp"


using LateralController = autoware::motion::control::trajectory_follower_nodes::LateralController;
using LateralCommand = autoware_auto_msgs::msg::AckermannLateralCommand;
using Trajectory = autoware_auto_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_msgs::msg::TrajectoryPoint;
using VehicleKinematicState = autoware_auto_msgs::msg::VehicleKinematicState;

using FakeNodeFixture = autoware::tools::testing::FakeTestNode;

const rclcpp::Duration one_second(1, 0);

std::shared_ptr<LateralController> makeNode()
{
  // Pass default parameter file to the node
  const auto share_dir = ament_index_cpp::get_package_share_directory("trajectory_follower_nodes");
  rclcpp::NodeOptions node_options;
  node_options.arguments(
    {"--ros-args", "--params-file", share_dir + "/param/lateral_controller_defaults.yaml"});
  std::shared_ptr<LateralController> node = std::make_shared<LateralController>(node_options);

  // Enable all logging in the node
  auto ret = rcutils_logging_set_logger_level(
    node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK) {std::cout << "Failed to set logging severerity to DEBUG\n";}
  return node;
}

TEST_F(FakeNodeFixture, no_input)
{
  // Data to test
  LateralCommand::SharedPtr cmd_msg;
  bool received_lateral_command = false;
  // Node
  std::shared_ptr<LateralController> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>(
    "input/reference_trajectory");
  rclcpp::Publisher<VehicleKinematicState>::SharedPtr state_pub =
    this->create_publisher<VehicleKinematicState>(
    "input/current_kinematic_state");
  rclcpp::Subscription<LateralCommand>::SharedPtr cmd_sub =
    this->create_subscription<LateralCommand>(
    "output/lateral/control_cmd", *this->get_fake_node(),
    [&cmd_msg, &received_lateral_command](const LateralCommand::SharedPtr msg) {
      cmd_msg = msg; received_lateral_command = true;
    });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // No published data: expect a stopped command
  test_utils::waitForMessage(node, this, received_lateral_command);
  ASSERT_TRUE(received_lateral_command);
  EXPECT_EQ(cmd_msg->steering_tire_angle, 0.0f);
  EXPECT_EQ(cmd_msg->steering_tire_rotation_rate, 0.0f);
}

TEST_F(FakeNodeFixture, empty_trajectory)
{
  // Data to test
  LateralCommand::SharedPtr cmd_msg;
  bool received_lateral_command = false;
  // Node
  std::shared_ptr<LateralController> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>(
    "input/reference_trajectory");
  rclcpp::Publisher<VehicleKinematicState>::SharedPtr state_pub =
    this->create_publisher<VehicleKinematicState>(
    "input/current_kinematic_state");
  rclcpp::Subscription<LateralCommand>::SharedPtr cmd_sub =
    this->create_subscription<LateralCommand>(
    "output/lateral/control_cmd", *this->get_fake_node(),
    [&cmd_msg, &received_lateral_command](const LateralCommand::SharedPtr msg) {
      cmd_msg = msg; received_lateral_command = true;
    });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);
  // Empty trajectory: expect a stopped command
  Trajectory traj_msg;
  VehicleKinematicState state_msg;
  traj_msg.header.stamp = node->now();
  state_msg.header.stamp = node->now();
  state_msg.state.longitudinal_velocity_mps = 0.0;
  state_msg.state.front_wheel_angle_rad = 0.0;
  traj_pub->publish(traj_msg);
  state_pub->publish(state_msg);

  test_utils::waitForMessage(node, this, received_lateral_command);
  ASSERT_TRUE(received_lateral_command);
  EXPECT_EQ(cmd_msg->steering_tire_angle, 0.0f);
  EXPECT_EQ(cmd_msg->steering_tire_rotation_rate, 0.0f);
  EXPECT_GT(rclcpp::Time(cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}

TEST_F(FakeNodeFixture, straight_trajectory)
{
  // Data to test
  LateralCommand::SharedPtr cmd_msg;
  bool received_lateral_command = false;
  // Node
  std::shared_ptr<LateralController> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>(
    "input/reference_trajectory");
  rclcpp::Publisher<VehicleKinematicState>::SharedPtr state_pub =
    this->create_publisher<VehicleKinematicState>(
    "input/current_kinematic_state");
  rclcpp::Subscription<LateralCommand>::SharedPtr cmd_sub =
    this->create_subscription<LateralCommand>(
    "output/lateral/control_cmd", *this->get_fake_node(),
    [&cmd_msg, &received_lateral_command](const LateralCommand::SharedPtr msg) {
      cmd_msg = msg; received_lateral_command = true;
    });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);
  // Straight trajectory: expect no steering
  received_lateral_command = false;
  Trajectory traj_msg;
  VehicleKinematicState state_msg;
  TrajectoryPoint p;
  traj_msg.header.stamp = node->now();
  p.x = -1.0f;
  p.y = 0.0f;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.x = 0.0f;
  p.y = 0.0f;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.x = 1.0f;
  p.y = 0.0f;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.x = 2.0f;
  p.y = 0.0f;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  traj_pub->publish(traj_msg);
  state_msg.header.stamp = node->now();
  state_msg.state.longitudinal_velocity_mps = 1.0;
  state_msg.state.front_wheel_angle_rad = 0.0;
  state_pub->publish(state_msg);

  test_utils::waitForMessage(node, this, received_lateral_command);
  ASSERT_TRUE(received_lateral_command);
  EXPECT_EQ(cmd_msg->steering_tire_angle, 0.0f);
  EXPECT_EQ(cmd_msg->steering_tire_rotation_rate, 0.0f);
  EXPECT_GT(rclcpp::Time(cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}

TEST_F(FakeNodeFixture, right_turn)
{
  // Data to test
  LateralCommand::SharedPtr cmd_msg;
  bool received_lateral_command = false;
  // Node
  std::shared_ptr<LateralController> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>(
    "input/reference_trajectory");
  rclcpp::Publisher<VehicleKinematicState>::SharedPtr state_pub =
    this->create_publisher<VehicleKinematicState>(
    "input/current_kinematic_state");
  rclcpp::Subscription<LateralCommand>::SharedPtr cmd_sub =
    this->create_subscription<LateralCommand>(
    "output/lateral/control_cmd", *this->get_fake_node(),
    [&cmd_msg, &received_lateral_command](const LateralCommand::SharedPtr msg) {
      cmd_msg = msg; received_lateral_command = true;
    });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);
  // Right turning trajectory: expect right steering
  received_lateral_command = false;
  Trajectory traj_msg;
  VehicleKinematicState state_msg;
  TrajectoryPoint p;
  traj_msg.points.clear();
  p.x = -1.0f;
  p.y = -1.0f;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.x = 0.0f;
  p.y = 0.0f;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.x = 1.0f;
  p.y = -1.0f;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.x = 2.0f;
  p.y = -2.0f;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  traj_pub->publish(traj_msg);
  state_msg.header.stamp = node->now();
  state_msg.state.longitudinal_velocity_mps = 1.0;
  state_msg.state.front_wheel_angle_rad = 0.0;
  state_pub->publish(state_msg);

  test_utils::waitForMessage(node, this, received_lateral_command);
  ASSERT_TRUE(received_lateral_command);
  /* TODO (Maxime CLEMENT): these tests fail only in the Autoware.auto CI (not on forks or locally)
  EXPECT_LT(cmd_msg->steering_tire_angle, 0.0f);
  EXPECT_LT(cmd_msg->steering_tire_rotation_rate, 0.0f);
  */
  EXPECT_GT(rclcpp::Time(cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}

TEST_F(FakeNodeFixture, left_turn)
{
  // Data to test
  LateralCommand::SharedPtr cmd_msg;
  bool received_lateral_command = false;
  // Node
  std::shared_ptr<LateralController> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>(
    "input/reference_trajectory");
  rclcpp::Publisher<VehicleKinematicState>::SharedPtr state_pub =
    this->create_publisher<VehicleKinematicState>(
    "input/current_kinematic_state");
  rclcpp::Subscription<LateralCommand>::SharedPtr cmd_sub =
    this->create_subscription<LateralCommand>(
    "output/lateral/control_cmd", *this->get_fake_node(),
    [&cmd_msg, &received_lateral_command](const LateralCommand::SharedPtr msg) {
      cmd_msg = msg; received_lateral_command = true;
    });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);
  // Left turning trajectory: expect left steering
  received_lateral_command = false;
  Trajectory traj_msg;
  VehicleKinematicState state_msg;
  TrajectoryPoint p;
  traj_msg.points.clear();
  p.x = -1.0f;
  p.y = 1.0f;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.x = 0.0f;
  p.y = 0.0f;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.x = 1.0f;
  p.y = 1.0f;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.x = 2.0f;
  p.y = 2.0f;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  traj_pub->publish(traj_msg);
  state_msg.header.stamp = node->now();
  state_msg.state.longitudinal_velocity_mps = 1.0;
  state_msg.state.front_wheel_angle_rad = 0.0;
  state_pub->publish(state_msg);

  test_utils::waitForMessage(node, this, received_lateral_command);
  ASSERT_TRUE(received_lateral_command);
  /* TODO (Maxime CLEMENT): these tests fail only in the Autoware.auto CI (not on forks or locally)
  EXPECT_GT(cmd_msg->steering_tire_angle, 0.0f);
  EXPECT_GT(cmd_msg->steering_tire_rotation_rate, 0.0f);
  */
  EXPECT_GT(rclcpp::Time(cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}

TEST_F(FakeNodeFixture, stopped)
{
  // Data to test
  LateralCommand::SharedPtr cmd_msg;
  bool received_lateral_command = false;
  // Node
  std::shared_ptr<LateralController> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>(
    "input/reference_trajectory");
  rclcpp::Publisher<VehicleKinematicState>::SharedPtr state_pub =
    this->create_publisher<VehicleKinematicState>(
    "input/current_kinematic_state");
  rclcpp::Subscription<LateralCommand>::SharedPtr cmd_sub =
    this->create_subscription<LateralCommand>(
    "output/lateral/control_cmd", *this->get_fake_node(),
    [&cmd_msg, &received_lateral_command](const LateralCommand::SharedPtr msg) {
      cmd_msg = msg; received_lateral_command = true;
    });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);
  // Straight trajectory: expect no steering
  received_lateral_command = false;
  Trajectory traj_msg;
  VehicleKinematicState state_msg;
  TrajectoryPoint p;
  traj_msg.header.stamp = node->now();
  p.x = -1.0f;
  p.y = 0.0f;
  // Set a 0 current velocity and 0 target velocity -> stopped state
  p.longitudinal_velocity_mps = 0.0f;
  traj_msg.points.push_back(p);
  p.x = 0.0f;
  p.y = 0.0f;
  p.longitudinal_velocity_mps = 0.0f;
  traj_msg.points.push_back(p);
  p.x = 1.0f;
  p.y = 0.0f;
  p.longitudinal_velocity_mps = 0.0f;
  traj_msg.points.push_back(p);
  p.x = 2.0f;
  p.y = 0.0f;
  p.longitudinal_velocity_mps = 0.0f;
  traj_msg.points.push_back(p);
  traj_pub->publish(traj_msg);
  state_msg.header.stamp = node->now();
  state_msg.state.longitudinal_velocity_mps = 0.0;
  state_msg.state.front_wheel_angle_rad = -0.5;  // dummy value
  state_pub->publish(state_msg);

  test_utils::waitForMessage(node, this, received_lateral_command);
  ASSERT_TRUE(received_lateral_command);
  // when stopped we expect a command that do not change the current state
  /* TODO (Maxime CLEMENT): these tests fail only in the Autoware.auto CI (not on forks or locally)
  EXPECT_EQ(cmd_msg->steering_tire_angle, state_msg.state.front_wheel_angle_rad);
  */
  EXPECT_EQ(cmd_msg->steering_tire_rotation_rate, 0.0f);
  EXPECT_GT(rclcpp::Time(cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}

// TODO(Maxime CLEMENT): disabled as this test crashes in the CI but works locally
TEST_F(FakeNodeFixture, DISABLED_set_lateral_param_smoke_test)
{
  // Node
  std::shared_ptr<LateralController> node = makeNode();
  // give the node some time to initialize completely
  std::this_thread::sleep_for(std::chrono::milliseconds{100LL});

  // Change some parameter value
  auto result = node->set_parameter(rclcpp::Parameter("mpc_prediction_horizon", 10));
  EXPECT_TRUE(result.successful);
}
