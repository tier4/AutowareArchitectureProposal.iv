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

#include "route.hpp"
#include <memory>

namespace external_api
{

Route::Route(const rclcpp::NodeOptions & options)
: Node("external_api_route", options)
{
  using namespace std::placeholders;
  autoware_api_utils::ServiceProxyNodeInterface proxy(this);

  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_set_route_ = proxy.create_service<autoware_external_api_msgs::srv::SetRoute>(
    "/api/external/set/route",
    std::bind(&Route::setRoute, this, _1, _2),
    rmw_qos_profile_services_default, group_);
  srv_clear_route_ = proxy.create_service<autoware_external_api_msgs::srv::ClearRoute>(
    "/api/external/set/clear_route",
    std::bind(&Route::clearRoute, this, _1, _2),
    rmw_qos_profile_services_default, group_);

  cli_set_route_ = proxy.create_client<autoware_external_api_msgs::srv::SetRoute>(
    "/api/autoware/set/route");
  cli_clear_route_ = proxy.create_client<autoware_external_api_msgs::srv::ClearRoute>(
    "/api/autoware/set/clear_route");

  pub_get_route_ = create_publisher<autoware_external_api_msgs::msg::Route>(
    "/api/external/get/route", rclcpp::QoS(1).transient_local());
  sub_get_route_ = create_subscription<autoware_external_api_msgs::msg::Route>(
    "/api/autoware/get/route", rclcpp::QoS(1).transient_local(),
    std::bind(&Route::onRoute, this, _1));
  sub_autoware_state_ = create_subscription<autoware_system_msgs::msg::AutowareState>(
    "/autoware/state", rclcpp::QoS(1),
    std::bind(&Route::onAutowareState, this, _1));

  waiting_for_route_ = false;
}

void Route::setRoute(
  const autoware_external_api_msgs::srv::SetRoute::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::SetRoute::Response::SharedPtr response)
{
  if (!waiting_for_route_) {
    response->status = autoware_api_utils::response_error("It is not ready to set route.");
    return;
  }

  auto [status, resp] = cli_set_route_->call(request);
  if (!autoware_api_utils::is_success(status)) {
    response->status = status;
    return;
  }
  response->status = resp->status;
}

void Route::clearRoute(
  const autoware_external_api_msgs::srv::ClearRoute::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::ClearRoute::Response::SharedPtr response)
{
  // TODO(Takagi, Isamu): add a check after changing the state transition
  auto [status, resp] = cli_clear_route_->call(request);
  if (!autoware_api_utils::is_success(status)) {
    response->status = status;
    return;
  }
  response->status = resp->status;
}

void Route::onRoute(
  const autoware_external_api_msgs::msg::Route::ConstSharedPtr message)
{
  pub_get_route_->publish(*message);
}

void Route::onAutowareState(
  const autoware_system_msgs::msg::AutowareState::SharedPtr message)
{
  using autoware_system_msgs::msg::AutowareState;
  waiting_for_route_ = (message->state == AutowareState::WAITING_FOR_ROUTE);
}

}  // namespace external_api

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::Route)
