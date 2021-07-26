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

#include "rclcpp/rclcpp.hpp"
#include "autoware_api_utils/autoware_api_utils.hpp"
#include "autoware_external_api_msgs/srv/pause_driving.hpp"
#include "autoware_external_api_msgs/srv/set_velocity_limit.hpp"

namespace external_api
{

class Velocity : public rclcpp::Node
{
public:
  explicit Velocity(const rclcpp::NodeOptions & options);

private:
  using PauseDriving = autoware_external_api_msgs::srv::PauseDriving;
  using SetVelocityLimit = autoware_external_api_msgs::srv::SetVelocityLimit;

  // ros interface
  rclcpp::CallbackGroup::SharedPtr group_;
  autoware_api_utils::Service<PauseDriving>::SharedPtr srv_pause_;
  autoware_api_utils::Client<PauseDriving>::SharedPtr cli_pause_;
  autoware_api_utils::Service<SetVelocityLimit>::SharedPtr srv_velocity_;
  autoware_api_utils::Client<SetVelocityLimit>::SharedPtr cli_velocity_;

  // ros callback
  void setPauseDriving(
    const autoware_external_api_msgs::srv::PauseDriving::Request::SharedPtr request,
    const autoware_external_api_msgs::srv::PauseDriving::Response::SharedPtr response);
  void setVelocityLimit(
    const autoware_external_api_msgs::srv::SetVelocityLimit::Request::SharedPtr request,
    const autoware_external_api_msgs::srv::SetVelocityLimit::Response::SharedPtr response);
};

}  // namespace external_api

#endif  // VELOCITY_HPP_
