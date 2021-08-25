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

namespace external_api
{

Velocity::Velocity(const rclcpp::NodeOptions & options)
: Node("external_api_velocity", options)
{
  using namespace std::placeholders;
  autoware_api_utils::ServiceProxyNodeInterface proxy(this);

  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_pause_ = proxy.create_service<autoware_external_api_msgs::srv::PauseDriving>(
    "/api/external/set/pause_driving",
    std::bind(&Velocity::setPauseDriving, this, _1, _2),
    rmw_qos_profile_services_default, group_);
  cli_pause_ = proxy.create_client<autoware_external_api_msgs::srv::PauseDriving>(
    "/api/autoware/set/pause_driving",
    rmw_qos_profile_services_default);
  srv_velocity_ = proxy.create_service<autoware_external_api_msgs::srv::SetVelocityLimit>(
    "/api/external/set/velocity_limit",
    std::bind(&Velocity::setVelocityLimit, this, _1, _2),
    rmw_qos_profile_services_default, group_);
  cli_velocity_ = proxy.create_client<autoware_external_api_msgs::srv::SetVelocityLimit>(
    "/api/autoware/set/velocity_limit",
    rmw_qos_profile_services_default);
}

void Velocity::setPauseDriving(
  const autoware_external_api_msgs::srv::PauseDriving::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::PauseDriving::Response::SharedPtr response)
{
  auto [status, resp] = cli_pause_->call(request);
  if (!autoware_api_utils::is_success(status)) {
    response->status = status;
    return;
  }
  response->status = resp->status;
}

void Velocity::setVelocityLimit(
  const autoware_external_api_msgs::srv::SetVelocityLimit::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::SetVelocityLimit::Response::SharedPtr response)
{
  auto [status, resp] = cli_velocity_->call(request);
  if (!autoware_api_utils::is_success(status)) {
    response->status = status;
    return;
  }
  response->status = resp->status;
}

}  // namespace external_api

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::Velocity)
