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

#include "service.hpp"

namespace external_api
{

Service::Service(const rclcpp::NodeOptions & options)
: Node("external_api_service", options)
{
  using namespace std::placeholders;
  autoware_api_utils::ServiceProxyNodeInterface proxy(this);

  srv_set_service_ = proxy.create_service<autoware_external_api_msgs::srv::SetService>(
    "/api/external/set/service",
    std::bind(&Service::setService, this, _1, _2));
  pub_get_service_ = create_publisher<autoware_external_api_msgs::msg::Service>(
    "/api/external/get/service", rclcpp::QoS(1).transient_local());

  pub_get_service_->publish(
    autoware_external_api_msgs::build<autoware_external_api_msgs::msg::Service>()
    .mode(autoware_external_api_msgs::msg::Service::NOT_IN_SERVICE));
}

void Service::setService(
  const autoware_external_api_msgs::srv::SetService::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::SetService::Response::SharedPtr response)
{
  pub_get_service_->publish(request->mode);
  response->status = autoware_api_utils::response_success();
}

}  // namespace external_api

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::Service)
