// Copyright 2018-2019 Autoware Foundation
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

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include "tf2_ros/transform_broadcaster.h"

#include "ekf_localizer/ekf_localizer.hpp"

using std::placeholders::_1;

class EKFLocalizerTestSuite : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown()
  {
    (void)rclcpp::shutdown();
  }
};  // sanity_check

class TestEKFLocalizerNode : public EKFLocalizer
{
public:
  TestEKFLocalizerNode(const std::string & node_name, const rclcpp::NodeOptions & node_options)
  : EKFLocalizer(node_name, node_options)
  {
    sub_twist = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/ekf_twist", 1, std::bind(
        &TestEKFLocalizerNode::testCallbackTwist, this,
        _1));
    sub_pose =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ekf_pose", 1,
      std::bind(&TestEKFLocalizerNode::testCallbackPose, this, _1));

    auto test_timer_callback = std::bind(&TestEKFLocalizerNode::testTimerCallback, this);
    auto period = std::chrono::milliseconds(100);
    test_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(test_timer_callback)>>(
      this->get_clock(), period, std::move(test_timer_callback),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(test_timer_, nullptr);
  }
  ~TestEKFLocalizerNode() {}

  std::string frame_id_a_ = "world";
  std::string frame_id_b_ = "base_link";

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose;

  rclcpp::TimerBase::SharedPtr test_timer_;

  geometry_msgs::msg::PoseStamped::SharedPtr test_current_pose_ptr_;
  geometry_msgs::msg::TwistStamped::SharedPtr test_current_twist_ptr_;

  void testTimerCallback()
  {
    /* !!! this should be defined before sendTransform() !!! */
    static std::shared_ptr<tf2_ros::TransformBroadcaster> br =
      std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
    geometry_msgs::msg::TransformStamped sended;

    rclcpp::Time current_time = this->now();

    sended.header.stamp = current_time;
    sended.header.frame_id = frame_id_a_;
    sended.child_frame_id = frame_id_b_;
    sended.transform.translation.x = -7.11;
    sended.transform.translation.y = 0.0;
    sended.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0.5);
    sended.transform.rotation.x = q.x();
    sended.transform.rotation.y = q.y();
    sended.transform.rotation.z = q.z();
    sended.transform.rotation.w = q.w();

    br->sendTransform(sended);
  }

  void testCallbackPose(geometry_msgs::msg::PoseStamped::SharedPtr pose)
  {
    test_current_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(*pose);
  }

  void testCallbackTwist(geometry_msgs::msg::TwistStamped::SharedPtr twist)
  {
    test_current_twist_ptr_ = std::make_shared<geometry_msgs::msg::TwistStamped>(*twist);
  }

  void resetCurrentPoseAndTwist()
  {
    test_current_pose_ptr_ = nullptr;
    test_current_twist_ptr_ = nullptr;
  }
};

TEST_F(EKFLocalizerTestSuite, measurementUpdatePose)
{
  rclcpp::NodeOptions node_options;
  auto ekf = std::make_shared<TestEKFLocalizerNode>("EKFLocalizerTestSuite", node_options);

  auto pub_pose = ekf->create_publisher<geometry_msgs::msg::PoseStamped>("/in_pose", 1);

  geometry_msgs::msg::PoseStamped in_pose;
  in_pose.header.frame_id = "world";
  in_pose.pose.position.x = 1.0;
  in_pose.pose.position.y = 2.0;
  in_pose.pose.position.z = 3.0;
  in_pose.pose.orientation.x = 0.0;
  in_pose.pose.orientation.y = 0.0;
  in_pose.pose.orientation.z = 0.0;
  in_pose.pose.orientation.w = 1.0;

  /* test for valid value */
  const double pos_x = 12.3;
  in_pose.pose.position.x = pos_x;  // for vaild value

  for (int i = 0; i < 20; ++i) {
    in_pose.header.stamp = ekf->now();
    pub_pose->publish(in_pose);
    rclcpp::spin_some(ekf);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  ASSERT_NE(ekf->test_current_pose_ptr_, nullptr);
  ASSERT_NE(ekf->test_current_twist_ptr_, nullptr);

  double ekf_x = ekf->test_current_pose_ptr_->pose.position.x;
  bool is_succeeded = !(std::isnan(ekf_x) || std::isinf(ekf_x));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";

  ASSERT_TRUE(std::fabs(ekf_x - pos_x) < 0.1) <<
    "ekf pos x: " << ekf_x << " should be close to " << pos_x;

  /* test for invalid value */
  in_pose.pose.position.x = NAN;  // check for invalid values
  for (int i = 0; i < 10; ++i) {
    in_pose.header.stamp = ekf->now();
    pub_pose->publish(in_pose);
    rclcpp::spin_some(ekf);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  is_succeeded = !(std::isnan(ekf_x) || std::isinf(ekf_x));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";

  ekf->resetCurrentPoseAndTwist();
}

TEST_F(EKFLocalizerTestSuite, measurementUpdateTwist)
{
  rclcpp::NodeOptions node_options;
  auto ekf = std::make_shared<TestEKFLocalizerNode>("EKFLocalizerTestSuite", node_options);

  auto pub_twist = ekf->create_publisher<geometry_msgs::msg::TwistStamped>("/in_twist", 1);
  geometry_msgs::msg::TwistStamped in_twist;
  in_twist.header.frame_id = "base_link";

  /* test for valid value */
  const double vx = 12.3;
  in_twist.twist.linear.x = vx;  // for vaild value
  for (int i = 0; i < 20; ++i) {
    in_twist.header.stamp = ekf->now();
    pub_twist->publish(in_twist);
    rclcpp::spin_some(ekf);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  ASSERT_FALSE(ekf->test_current_pose_ptr_ == nullptr);
  ASSERT_FALSE(ekf->test_current_twist_ptr_ == nullptr);

  double ekf_vx = ekf->test_current_twist_ptr_->twist.linear.x;
  bool is_succeeded = !(std::isnan(ekf_vx) || std::isinf(ekf_vx));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";
  ASSERT_TRUE(std::fabs(ekf_vx - vx) < 0.1) <<
    "ekf vel x: " << ekf_vx << ", should be close to " << vx;

  /* test for invalid value */
  in_twist.twist.linear.x = NAN;  // check for invalid values
  for (int i = 0; i < 10; ++i) {
    in_twist.header.stamp = ekf->now();
    pub_twist->publish(in_twist);
    rclcpp::spin_some(ekf);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  ekf_vx = ekf->test_current_twist_ptr_->twist.linear.x;
  is_succeeded = !(std::isnan(ekf_vx) || std::isinf(ekf_vx));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";

  ekf->resetCurrentPoseAndTwist();
}

TEST_F(EKFLocalizerTestSuite, measurementUpdatePoseWithCovariance)
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("use_pose_with_covariance", true);
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  auto ekf = std::make_shared<TestEKFLocalizerNode>("EKFLocalizerTestSuite", node_options);

  auto pub_pose =
    ekf->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/in_pose_with_covariance",
    1);
  geometry_msgs::msg::PoseWithCovarianceStamped in_pose;
  in_pose.header.frame_id = "world";
  in_pose.pose.pose.position.x = 1.0;
  in_pose.pose.pose.position.y = 2.0;
  in_pose.pose.pose.position.z = 3.0;
  in_pose.pose.pose.orientation.x = 0.0;
  in_pose.pose.pose.orientation.y = 0.0;
  in_pose.pose.pose.orientation.z = 0.0;
  in_pose.pose.pose.orientation.w = 1.0;
  for (int i = 0; i < 36; ++i) {
    in_pose.pose.covariance[i] = 0.1;
  }

  /* test for valid value */
  const double pos_x = 99.3;
  in_pose.pose.pose.position.x = pos_x;  // for vaild value

  for (int i = 0; i < 20; ++i) {
    in_pose.header.stamp = ekf->now();
    pub_pose->publish(in_pose);
    rclcpp::spin_some(ekf);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  ASSERT_FALSE(ekf->test_current_pose_ptr_ == nullptr);
  ASSERT_FALSE(ekf->test_current_twist_ptr_ == nullptr);

  double ekf_x = ekf->test_current_pose_ptr_->pose.position.x;
  bool is_succeeded = !(std::isnan(ekf_x) || std::isinf(ekf_x));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";
  ASSERT_TRUE(std::fabs(ekf_x - pos_x) < 0.1) <<
    "ekf pos x: " << ekf_x << " should be close to " << pos_x;

  /* test for invalid value */
  in_pose.pose.pose.position.x = NAN;  // check for invalid values
  for (int i = 0; i < 10; ++i) {
    in_pose.header.stamp = ekf->now();
    pub_pose->publish(in_pose);
    rclcpp::spin_some(ekf);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  is_succeeded = !(std::isnan(ekf_x) || std::isinf(ekf_x));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";

  ekf->resetCurrentPoseAndTwist();
}

TEST_F(EKFLocalizerTestSuite, measurementUpdateTwistWithCovariance)
{
  rclcpp::NodeOptions node_options;
  auto ekf = std::make_shared<TestEKFLocalizerNode>("EKFLocalizerTestSuite", node_options);

  auto pub_twist =
    ekf->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/in_twist_with_covariance", 1);
  geometry_msgs::msg::TwistWithCovarianceStamped in_twist;
  in_twist.header.frame_id = "base_link";

  /* test for valid value */
  const double vx = 12.3;
  in_twist.twist.twist.linear.x = vx;  // for vaild value
  for (int i = 0; i < 36; ++i) {
    in_twist.twist.covariance[i] = 0.1;
  }
  for (int i = 0; i < 10; ++i) {
    in_twist.header.stamp = ekf->now();
    pub_twist->publish(in_twist);
    rclcpp::spin_some(ekf);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  ASSERT_FALSE(ekf->test_current_pose_ptr_ == nullptr);
  ASSERT_FALSE(ekf->test_current_twist_ptr_ == nullptr);

  double ekf_vx = ekf->test_current_twist_ptr_->twist.linear.x;
  bool is_succeeded = !(std::isnan(ekf_vx) || std::isinf(ekf_vx));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";
  ASSERT_TRUE((ekf_vx - vx) < 0.1) << "vel x should be close to " << vx;

  /* test for invalid value */
  in_twist.twist.twist.linear.x = NAN;  // check for invalid values
  for (int i = 0; i < 10; ++i) {
    in_twist.header.stamp = ekf->now();
    pub_twist->publish(in_twist);
    rclcpp::spin_some(ekf);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  ekf_vx = ekf->test_current_twist_ptr_->twist.linear.x;
  is_succeeded = !(std::isnan(ekf_vx) || std::isinf(ekf_vx));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";
}
