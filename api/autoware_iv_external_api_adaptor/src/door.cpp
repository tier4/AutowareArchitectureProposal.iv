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

#include "door.hpp"

namespace external_api
{

Door::Door(const rclcpp::NodeOptions & options)
: Node("external_api_door", options)
{
  using namespace std::placeholders;
  autoware_api_utils::ServiceProxyNodeInterface proxy(this);

  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_ = proxy.create_service<autoware_external_api_msgs::srv::SetDoor>(
    "/api/external/set/door",
    std::bind(&Door::setDoor, this, _1, _2),
    rmw_qos_profile_services_default, group_);
  cli_ = proxy.create_client<autoware_external_api_msgs::srv::SetDoor>(
    "/api/vehicle/set/door",
    rmw_qos_profile_services_default);
}

void Door::setDoor(
  const autoware_external_api_msgs::srv::SetDoor::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::SetDoor::Response::SharedPtr response)
{
  auto [status, resp] = cli_->call(request);
  if (!autoware_api_utils::is_success(status)) {
    response->status = status;
    return;
  }
  response->status = resp->status;
}

}  // namespace external_api

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::Door)
