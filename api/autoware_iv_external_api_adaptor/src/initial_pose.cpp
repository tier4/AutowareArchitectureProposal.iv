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

#include "initial_pose.hpp"
#include <memory>

namespace external_api
{

InitialPose::InitialPose(const rclcpp::NodeOptions & options)
: Node("external_api_initial_pose", options)
{
  using namespace std::placeholders;
  autoware_api_utils::ServiceProxyNodeInterface proxy(this);

  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_set_initialize_pose_ = proxy.create_service<InitializePose>(
    "/api/external/set/initialize_pose",
    std::bind(&InitialPose::setInitializePose, this, _1, _2),
    rmw_qos_profile_services_default, group_);
  srv_set_initialize_pose_auto_ = proxy.create_service<InitializePoseAuto>(
    "/api/external/set/initialize_pose_auto",
    std::bind(&InitialPose::setInitializePoseAuto, this, _1, _2),
    rmw_qos_profile_services_default, group_);
  cli_set_initialize_pose_ = proxy.create_client<InitializePose>(
    "/api/autoware/set/initialize_pose",
    rmw_qos_profile_services_default);
  cli_set_initialize_pose_auto_ = proxy.create_client<InitializePoseAuto>(
    "/api/autoware/set/initialize_pose_auto",
    rmw_qos_profile_services_default);
}

void InitialPose::setInitializePose(
  const autoware_external_api_msgs::srv::InitializePose::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::InitializePose::Response::SharedPtr response)
{
  const auto [status, resp] = cli_set_initialize_pose_->call(request);
  if (!autoware_api_utils::is_success(status)) {
    response->status = status;
    return;
  }
  response->status = resp->status;
}

void InitialPose::setInitializePoseAuto(
  const autoware_external_api_msgs::srv::InitializePoseAuto::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::InitializePoseAuto::Response::SharedPtr response)
{
  const auto [status, resp] = cli_set_initialize_pose_auto_->call(request);
  if (!autoware_api_utils::is_success(status)) {
    response->status = status;
    return;
  }
  response->status = resp->status;
}

}  // namespace external_api

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::InitialPose)
