// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <string>
#include <memory>
#include <functional>
#include <utility>

#include "steering_offset_estimator/steering_offset_estimator_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp/time.hpp"

SteeringOffsetEstimator::SteeringOffsetEstimator(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("steering_offset_estimator", node_options), estimated_steer_offset_(0)
{
  update_hz_ = declare_parameter("update_hz", 0.1);
  wheel_base_ = declare_parameter("wheel_base", 0.285);
  covariance_ = declare_parameter("initial_covariance", 1000);
  forgetting_factor_ = declare_parameter("forgetting_factor", 0.999);
  valid_min_velocity_ = declare_parameter("valid_min_velocity", 0.285);
  valid_max_steer_ = declare_parameter("valid_max_steer", 0.05);

  steering_offset_pub_ =
    create_publisher<autoware_vehicle_msgs::msg::SteeringOffset>(
    "output/streering_offset",
    rclcpp::QoS{1}.transient_local());
  steering_offset_cov_pub_ =
    create_publisher<autoware_vehicle_msgs::msg::SteeringOffsetCovariance>(
    "output/streering_offset_covariance",
    rclcpp::QoS{1}.transient_local());
  twist_sub_ =
    create_subscription<geometry_msgs::msg::TwistStamped>(
    "input/twist", rclcpp::QoS{1},
    std::bind(&SteeringOffsetEstimator::callbackTwist, this, std::placeholders::_1));
  steer_sub_ =
    create_subscription<autoware_vehicle_msgs::msg::Steering>(
    "input/steer", rclcpp::QoS{1},
    std::bind(&SteeringOffsetEstimator::callbackSteer, this, std::placeholders::_1));

  auto timer_callback = std::bind(&SteeringOffsetEstimator::timerCallback, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / update_hz_));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}


void SteeringOffsetEstimator::callbackTwist(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  twist_ptr_ = msg;
}

void SteeringOffsetEstimator::callbackSteer(
  const autoware_vehicle_msgs::msg::Steering::ConstSharedPtr msg)
{
  steer_ptr_ = msg;
}

void SteeringOffsetEstimator::timerCallback()
{
  if (updateSteeringOffset()) {
    autoware_vehicle_msgs::msg::SteeringOffset msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = now();
    msg.data = estimated_steer_offset_;
    steering_offset_pub_->publish(msg);

    autoware_vehicle_msgs::msg::SteeringOffsetCovariance cov_msg;
    cov_msg.header.frame_id = "base_link";
    cov_msg.header.stamp = now();
    cov_msg.data = covariance_;
    steering_offset_cov_pub_->publish(cov_msg);
  }
}

bool SteeringOffsetEstimator::updateSteeringOffset()
{
  // RLS; recursive least-squares algorithm
  if (!twist_ptr_ || !steer_ptr_) {
    // null input
    return false;
  }
  const double vel = twist_ptr_->twist.linear.x;
  const double wz = twist_ptr_->twist.angular.z;
  const double steer = steer_ptr_->data;
  if (std::fabs(vel) < valid_min_velocity_ || std::fabs(steer) > valid_max_steer_) {
    // invalid velocity/steer value for estimating steering offset
    return false;
  }
  // use following approximation: tan(a+b) = tan(a) + tan(b) = a + b
  const double phi = vel / wheel_base_;
  covariance_ = (covariance_ - (covariance_ * phi * phi * covariance_) /
    (forgetting_factor_ + phi * covariance_ * phi)) /
    forgetting_factor_;
  const double coef = (covariance_ * phi) / (forgetting_factor_ + phi * covariance_ * phi);
  const double measured_wz_offset = wz - phi * steer;
  const double error_wz_offset = measured_wz_offset - phi * estimated_steer_offset_;
  estimated_steer_offset_ = estimated_steer_offset_ + coef * error_wz_offset;
  return true;
}

RCLCPP_COMPONENTS_REGISTER_NODE(SteeringOffsetEstimator)
