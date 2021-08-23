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

#ifndef ROUTE_HPP_
#define ROUTE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "autoware_api_utils/autoware_api_utils.hpp"
#include "autoware_external_api_msgs/srv/set_route.hpp"
#include "autoware_external_api_msgs/srv/clear_route.hpp"
#include "autoware_external_api_msgs/msg/route.hpp"
#include "autoware_system_msgs/msg/autoware_state.hpp"

namespace external_api
{

class Route : public rclcpp::Node
{
public:
  explicit Route(const rclcpp::NodeOptions & options);

private:
  using SetRoute = autoware_external_api_msgs::srv::SetRoute;
  using ClearRoute = autoware_external_api_msgs::srv::ClearRoute;
  using RouteMsg = autoware_external_api_msgs::msg::Route;
  using AutowareState = autoware_system_msgs::msg::AutowareState;

  // ros interface
  rclcpp::CallbackGroup::SharedPtr group_;
  autoware_api_utils::Service<SetRoute>::SharedPtr srv_set_route_;
  autoware_api_utils::Client<SetRoute>::SharedPtr cli_set_route_;
  autoware_api_utils::Service<ClearRoute>::SharedPtr srv_clear_route_;
  autoware_api_utils::Client<ClearRoute>::SharedPtr cli_clear_route_;
  rclcpp::Publisher<RouteMsg>::SharedPtr pub_get_route_;
  rclcpp::Subscription<RouteMsg>::SharedPtr sub_get_route_;
  rclcpp::Subscription<AutowareState>::SharedPtr sub_autoware_state_;

  // class state
  bool waiting_for_route_;

  // ros callback
  void setRoute(
    const autoware_external_api_msgs::srv::SetRoute::Request::SharedPtr request,
    const autoware_external_api_msgs::srv::SetRoute::Response::SharedPtr response);
  void clearRoute(
    const autoware_external_api_msgs::srv::ClearRoute::Request::SharedPtr request,
    const autoware_external_api_msgs::srv::ClearRoute::Response::SharedPtr response);
  void onRoute(
    const autoware_external_api_msgs::msg::Route::ConstSharedPtr message);
  void onAutowareState(
    const autoware_system_msgs::msg::AutowareState::SharedPtr message);
};

}  // namespace external_api

#endif  // ROUTE_HPP_
