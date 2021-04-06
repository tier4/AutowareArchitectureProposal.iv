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


#ifndef STEERING_OFFSET_ESTIMATOR__STEERING_OFFSET_ESTIMATOR_NODE_HPP_
#define STEERING_OFFSET_ESTIMATOR__STEERING_OFFSET_ESTIMATOR_NODE_HPP_

#include <memory>
#include <functional>
#include <string>

#include "autoware_vehicle_msgs/msg/steering_offset.hpp"
#include "autoware_vehicle_msgs/msg/steering_offset_covariance.hpp"
#include "autoware_vehicle_msgs/msg/steering.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class SteeringOffsetEstimator : public rclcpp::Node
{
public:
  explicit SteeringOffsetEstimator(const rclcpp::NodeOptions & node_options);

private:
  double wheel_base_;
  double update_hz_;
  double estimated_steer_offset_;
  double covariance_;
  double forgetting_factor_;
  double valid_min_velocity_;
  double valid_max_steer_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_ptr_;
  autoware_vehicle_msgs::msg::Steering::ConstSharedPtr steer_ptr_;

  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringOffset>::SharedPtr steering_offset_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringOffsetCovariance>::SharedPtr
    steering_offset_cov_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Steering>::SharedPtr steer_sub_;

  void callbackTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void callbackSteer(const autoware_vehicle_msgs::msg::Steering::ConstSharedPtr msg);
  bool updateSteeringOffset();
  void timerCallback();
};

#endif  //  STEERING_OFFSET_ESTIMATOR__STEERING_OFFSET_ESTIMATOR_NODE_HPP_
