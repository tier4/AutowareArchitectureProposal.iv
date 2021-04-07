// Copyright 2020 Tier IV, Inc.
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
#include <utility>
#include <fstream>
#include <chrono>
#include <string>
#include <vector>

#include "path_saver/path_saver_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp/time.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


PathSaverNode::PathSaverNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("path_saver"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  dist_threshold_ = declare_parameter("dist_threshold", 1.0);
  time_threshold_ = declare_parameter("time_threshold", 0.1);
  twist_sub_ =
    create_subscription<geometry_msgs::msg::TwistStamped>(
    "input/twist", rclcpp::QoS{1},
    std::bind(&PathSaverNode::callbackTwist, this, std::placeholders::_1));
  auto timer_callback = std::bind(&PathSaverNode::timerCallback, this);
  const double update_hz = 10.0;
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / update_hz));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
  std::string file_name;
  file_name = declare_parameter("path_file_name", "");
  ofs_.open(file_name, std::ios::trunc);
  ofs_ <<
    "x, y, z, roll[rad], pitch[rad], yaw[rad], linear_velocity[m/s], angular_velocity[rad/s]" <<
    std::endl;
}

void PathSaverNode::callbackTwist(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  twist_ptr_ = msg;
}


void PathSaverNode::updateCurrentPose()
{
  using namespace std::chrono_literals;
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), (5000ms).count(),
      "cannot get map to base_link transform. %s", ex.what());
    return;
  }
  geometry_msgs::msg::PoseStamped ps;
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  current_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(ps);
}


void PathSaverNode::timerCallback()
{
  updateCurrentPose();
  if (!current_pose_ptr_) {return;}
  if (last_saved_time_ == nullptr) {
    last_saved_time_ = std::make_shared<rclcpp::Time>(0, 0, RCL_ROS_TIME);
  } else {
    const double x = last_saved_pose_.position.x - current_pose_ptr_->pose.position.x;
    const double y = last_saved_pose_.position.y - current_pose_ptr_->pose.position.y;
    const double z = last_saved_pose_.position.z - current_pose_ptr_->pose.position.z;
    const double dist = std::sqrt(x * x + y * y + z * z);
    const double velocity = last_saved_twist_.linear.x;
    const double dist_from_velocity = velocity * time_threshold_;
    if (dist < dist_threshold_ && dist_from_velocity < dist_threshold_) {return;}
  }
  // save
  double roll, pitch, yaw;
  {
    tf2::Quaternion tf2_quaternion;
    tf2::fromMsg(current_pose_ptr_->pose.orientation, tf2_quaternion);
    tf2::Matrix3x3 tf2_matrix(tf2_quaternion);
    tf2_matrix.getRPY(roll, pitch, yaw);
  }
  auto cpos = current_pose_ptr_->pose.position;
  ofs_ << cpos.x << ", " << cpos.y << ", " <<
    cpos.z << ", " << roll << ", " << pitch << ", " << yaw << ", " <<
    twist_ptr_->twist.linear.x << ", " << twist_ptr_->twist.angular.z << std::endl;
  // last saved data
  *last_saved_time_ = now();
  last_saved_pose_ = current_pose_ptr_->pose;
  last_saved_twist_ = twist_ptr_->twist;
}


RCLCPP_COMPONENTS_REGISTER_NODE(PathSaverNode)
