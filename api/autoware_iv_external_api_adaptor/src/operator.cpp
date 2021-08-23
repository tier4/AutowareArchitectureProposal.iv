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

#include "operator.hpp"

namespace external_api
{

Operator::Operator(const rclcpp::NodeOptions & options)
: Node("external_api_operator", options)
{
  using namespace std::placeholders;
  autoware_api_utils::ServiceProxyNodeInterface proxy(this);

  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_set_operator_ = proxy.create_service<autoware_external_api_msgs::srv::SetOperator>(
    "/api/external/set/operator",
    std::bind(&Operator::setOperator, this, _1, _2),
    rmw_qos_profile_services_default, group_);
  srv_set_observer_ = proxy.create_service<autoware_external_api_msgs::srv::SetObserver>(
    "/api/external/set/observer",
    std::bind(&Operator::setObserver, this, _1, _2),
    rmw_qos_profile_services_default, group_);
  cli_set_operator_ = proxy.create_client<autoware_external_api_msgs::srv::SetOperator>(
    "/api/autoware/set/operator",
    rmw_qos_profile_services_default);
  cli_set_observer_ = proxy.create_client<autoware_external_api_msgs::srv::SetObserver>(
    "/api/autoware/set/observer",
    rmw_qos_profile_services_default);

  pub_get_operator_ = create_publisher<autoware_external_api_msgs::msg::Operator>(
    "/api/external/get/operator", rclcpp::QoS(1));
  pub_get_observer_ = create_publisher<autoware_external_api_msgs::msg::Observer>(
    "/api/external/get/observer", rclcpp::QoS(1));
  sub_get_operator_ = create_subscription<autoware_external_api_msgs::msg::Operator>(
    "/api/autoware/get/operator", rclcpp::QoS(1),
    std::bind(&Operator::onOperator, this, _1));
  sub_get_observer_ = create_subscription<autoware_external_api_msgs::msg::Observer>(
    "/api/autoware/get/observer", rclcpp::QoS(1),
    std::bind(&Operator::onObserver, this, _1));
}

void Operator::setOperator(
  const autoware_external_api_msgs::srv::SetOperator::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::SetOperator::Response::SharedPtr response)
{
  const auto [status, resp] = cli_set_operator_->call(request);
  if (!autoware_api_utils::is_success(status)) {
    response->status = status;
    return;
  }
  response->status = resp->status;
}

void Operator::setObserver(
  const autoware_external_api_msgs::srv::SetObserver::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::SetObserver::Response::SharedPtr response)
{
  const auto [status, resp] = cli_set_observer_->call(request);
  if (!autoware_api_utils::is_success(status)) {
    response->status = status;
    return;
  }
  response->status = resp->status;
}

void Operator::onOperator(
  const autoware_external_api_msgs::msg::Operator::ConstSharedPtr message)
{
  pub_get_operator_->publish(*message);
}

void Operator::onObserver(
  const autoware_external_api_msgs::msg::Observer::ConstSharedPtr message)
{
  pub_get_observer_->publish(*message);
}

}  // namespace external_api

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::Operator)
