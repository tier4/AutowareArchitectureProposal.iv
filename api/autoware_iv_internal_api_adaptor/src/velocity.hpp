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

#ifndef VELOCITY_HPP_
#define VELOCITY_HPP_

#include <autoware_api_utils/autoware_api_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_external_api_msgs/srv/pause_driving.hpp>
#include <autoware_external_api_msgs/srv/set_velocity_limit.hpp>
#include <autoware_planning_msgs/msg/velocity_limit.hpp>

namespace internal_api
{
class Velocity : public rclcpp::Node
{
public:
  explicit Velocity(const rclcpp::NodeOptions & options);

private:
  using PauseDriving = autoware_external_api_msgs::srv::PauseDriving;
  using SetVelocityLimit = autoware_external_api_msgs::srv::SetVelocityLimit;
  using VelocityLimit = autoware_planning_msgs::msg::VelocityLimit;

  // ros interface
  autoware_api_utils::Service<PauseDriving>::SharedPtr srv_pause_;
  autoware_api_utils::Service<SetVelocityLimit>::SharedPtr srv_velocity_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr pub_api_velocity_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr pub_planning_velocity_;
  rclcpp::Subscription<VelocityLimit>::SharedPtr sub_planning_velocity_;

  // class constants
  static constexpr double kVelocityEpsilon = 1e-5;

  // class state
  bool is_ready_;
  double velocity_limit_;

  // ros callback
  void setPauseDriving(
    const autoware_external_api_msgs::srv::PauseDriving::Request::SharedPtr request,
    const autoware_external_api_msgs::srv::PauseDriving::Response::SharedPtr response);
  void setVelocityLimit(
    const autoware_external_api_msgs::srv::SetVelocityLimit::Request::SharedPtr request,
    const autoware_external_api_msgs::srv::SetVelocityLimit::Response::SharedPtr response);
  void onVelocityLimit(const autoware_planning_msgs::msg::VelocityLimit::SharedPtr msg);

  // class method
  void publishApiVelocity(double velocity);
  void publishPlanningVelocity(double velocity);
};

}  // namespace internal_api

#endif  // VELOCITY_HPP_
