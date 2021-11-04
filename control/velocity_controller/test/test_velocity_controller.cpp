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

#include "velocity_controller/velocity_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <autoware_control_msgs/msg/control_command_stamped.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>

class TestROS : public ::testing::Test
{
protected:
  std::shared_ptr<VelocityController> m_node;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_vel_pub;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr m_traj_pub;
  rclcpp::Subscription<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr m_cmd_sub;

  autoware_control_msgs::msg::ControlCommandStamped m_cmd_msg;
  bool m_received_command;

  void SetUp()
  {
    rclcpp::init(0, nullptr);
    m_received_command = false;

    // Publish dummy transform for current pose at (0,0,0) in map frame
    rclcpp::Node tmp_node("tmp", rclcpp::NodeOptions());
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(tmp_node);
    geometry_msgs::msg::TransformStamped transform;
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.header.stamp = tmp_node.now();
    br->sendTransform(transform);

    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.append_parameter_override("wheel_radius", 1.0);
    node_options.append_parameter_override("wheel_width", 1.0);
    node_options.append_parameter_override("wheel_base", 1.0);
    node_options.append_parameter_override("wheel_tread", 1.0);
    node_options.append_parameter_override("front_overhang", 1.0);
    node_options.append_parameter_override("rear_overhang", 1.0);
    node_options.append_parameter_override("left_overhang", 1.0);
    node_options.append_parameter_override("right_overhang", 1.0);
    node_options.append_parameter_override("vehicle_height", 1.0);
    m_node = std::make_shared<VelocityController>(node_options);

    m_vel_pub = m_node->create_publisher<geometry_msgs::msg::TwistStamped>(
      "~/current_velocity", rclcpp::QoS(10));
    m_traj_pub = m_node->create_publisher<autoware_planning_msgs::msg::Trajectory>(
      "~/current_trajectory", rclcpp::QoS(10));
    m_cmd_sub = m_node->create_subscription<autoware_control_msgs::msg::ControlCommandStamped>(
      "~/control_cmd", rclcpp::QoS(10),
      std::bind(&TestROS::HandleOutputCommand, this, std::placeholders::_1));

    // Enable all logging in the node
    auto ret =
      rcutils_logging_set_logger_level(m_node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
      std::cout << "Failed to set logging severity to DEBUG\n";
    }
  }

  void TearDown() { rclcpp::shutdown(); }

  void HandleOutputCommand(const autoware_control_msgs::msg::ControlCommandStamped::SharedPtr cmd)
  {
    m_cmd_msg = *cmd;
    m_received_command = true;
  }
};

TEST_F(TestROS, simple_test)
{
  geometry_msgs::msg::TwistStamped twist;
  autoware_planning_msgs::msg::Trajectory traj;
  autoware_planning_msgs::msg::TrajectoryPoint point;
  /// Already running + Non stopping trajectory
  // Publish velocity
  twist.header.stamp = m_node->now();
  twist.twist.linear.x = 1.0;
  m_vel_pub->publish(twist);
  rclcpp::spin_some(m_node);
  twist.header.stamp = m_node->now();
  m_vel_pub->publish(twist);
  // Publish non stopping trajectory
  point.pose.position.x = 0.0;
  point.pose.position.y = 0.0;
  point.twist.linear.x = 1.0;
  traj.points.push_back(point);
  point.pose.position.x = 50.0;
  point.pose.position.y = 0.0;
  point.twist.linear.x = 1.0;
  traj.points.push_back(point);
  point.pose.position.x = 100.0;
  point.pose.position.y = 0.0;
  point.twist.linear.x = 1.0;
  traj.points.push_back(point);
  m_traj_pub->publish(traj);
  const rclcpp::Duration one_sec(1, 0);
  const rclcpp::Time start = m_node->now();
  while (!m_received_command && m_node->now() - start < one_sec) {
    rclcpp::spin_some(m_node);
  }
  ASSERT_TRUE(m_received_command);
  EXPECT_DOUBLE_EQ(m_cmd_msg.control.steering_angle, 0.0);
  EXPECT_DOUBLE_EQ(m_cmd_msg.control.steering_angle_velocity, 0.0);
  EXPECT_DOUBLE_EQ(m_cmd_msg.control.velocity, 1.0);
  EXPECT_DOUBLE_EQ(m_cmd_msg.control.acceleration, 0.0);
  m_received_command = false;
}
