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

#include "version.hpp"

namespace external_api
{

Version::Version(const rclcpp::NodeOptions & options)
: Node("external_api_version", options)
{
  using namespace std::placeholders;
  autoware_api_utils::ServiceProxyNodeInterface proxy(this);

  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_ = proxy.create_service<autoware_external_api_msgs::srv::GetVersion>(
    "/api/external/get/version",
    std::bind(&Version::getVersion, this, _1, _2),
    rmw_qos_profile_services_default, group_);
}

void Version::getVersion(
  const autoware_external_api_msgs::srv::GetVersion::Request::SharedPtr,
  const autoware_external_api_msgs::srv::GetVersion::Response::SharedPtr response)
{
  response->version = "0.1.0";
  response->status = autoware_api_utils::response_success();
}

}  // namespace external_api

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::Version)
