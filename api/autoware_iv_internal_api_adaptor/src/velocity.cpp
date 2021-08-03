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

#include "velocity.hpp"

namespace internal_api
{

Velocity::Velocity(const rclcpp::NodeOptions & options)
: Node("external_api_route", options)
{
  using namespace std::placeholders;
  autoware_api_utils::ServiceProxyNodeInterface proxy(this);

  srv_pause_ = proxy.create_service<autoware_external_api_msgs::srv::PauseDriving>(
    "/api/autoware/set/pause_driving",
    std::bind(&Velocity::setPauseDriving, this, _1, _2));
  srv_velocity_ = proxy.create_service<autoware_external_api_msgs::srv::SetVelocityLimit>(
    "/api/autoware/set/velocity_limit",
    std::bind(&Velocity::setVelocityLimit, this, _1, _2));

  pub_api_velocity_ = create_publisher<autoware_planning_msgs::msg::VelocityLimit>(
    "/api/autoware/get/velocity_limit", rclcpp::QoS(1).transient_local());
  pub_planning_velocity_ = create_publisher<autoware_planning_msgs::msg::VelocityLimit>(
    "/planning/scenario_planning/max_velocity", rclcpp::QoS(1).transient_local());
  sub_planning_velocity_ = create_subscription<autoware_planning_msgs::msg::VelocityLimit>(
    "/planning/scenario_planning/current_max_velocity", rclcpp::QoS(1).transient_local(),
    std::bind(&Velocity::onVelocityLimit, this, _1));

  is_ready_ = false;
  velocity_limit_ = 0.0;
}

void Velocity::setPauseDriving(
  const autoware_external_api_msgs::srv::PauseDriving::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::PauseDriving::Response::SharedPtr response)
{
  if (!is_ready_) {
    response->status = autoware_api_utils::response_error("It is not ready to set velocity.");
    return;
  }
  publishPlanningVelocity(request->pause ? 0.0 : velocity_limit_);
  response->status = autoware_api_utils::response_success();
}

void Velocity::setVelocityLimit(
  const autoware_external_api_msgs::srv::SetVelocityLimit::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::SetVelocityLimit::Response::SharedPtr response)
{
  if (!is_ready_) {
    response->status = autoware_api_utils::response_error("It is not ready to set velocity.");
    return;
  }
  publishPlanningVelocity(request->velocity);
  response->status = autoware_api_utils::response_success();
}

void Velocity::onVelocityLimit(const autoware_planning_msgs::msg::VelocityLimit::SharedPtr msg)
{
  // store the velocity for releasing the stop
  if (kVelocityEpsilon < msg->max_velocity) {
    velocity_limit_ = msg->max_velocity;
  }
  is_ready_ = true;
  publishApiVelocity(msg->max_velocity);
}

void Velocity::publishApiVelocity(double velocity)
{
  autoware_planning_msgs::msg::VelocityLimit msg;
  msg.stamp = now();
  msg.max_velocity = velocity;
  pub_api_velocity_->publish(msg);
}

void Velocity::publishPlanningVelocity(double velocity)
{
  autoware_planning_msgs::msg::VelocityLimit msg;
  msg.stamp = now();
  msg.max_velocity = velocity;
  pub_planning_velocity_->publish(msg);
}

}  // namespace internal_api

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(internal_api::Velocity)
