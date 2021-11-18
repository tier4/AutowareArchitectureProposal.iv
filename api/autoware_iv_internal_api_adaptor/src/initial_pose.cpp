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

namespace internal_api
{
InitialPose::InitialPose(const rclcpp::NodeOptions & options)
: Node("internal_api_initial_pose", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  autoware_api_utils::ServiceProxyNodeInterface proxy(this);

  init_simulator_pose_ = declare_parameter("init_simulator_pose", false);
  init_localization_pose_ = declare_parameter("init_localization_pose", false);

  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_set_initialize_pose_ = proxy.create_service<InitializePose>(
    "/api/autoware/set/initialize_pose", std::bind(&InitialPose::setInitializePose, this, _1, _2),
    rmw_qos_profile_services_default, group_);
  srv_set_initialize_pose_auto_ = proxy.create_service<InitializePoseAuto>(
    "/api/autoware/set/initialize_pose_auto",
    std::bind(&InitialPose::setInitializePoseAuto, this, _1, _2), rmw_qos_profile_services_default,
    group_);

  if (init_localization_pose_) {
    cli_set_initialize_pose_ = proxy.create_client<PoseWithCovarianceStampedSrv>(
      "/localization/util/initialize_pose", rmw_qos_profile_services_default);
    cli_set_initialize_pose_auto_ = proxy.create_client<InitializePoseAuto>(
      "/localization/util/initialize_pose_auto", rmw_qos_profile_services_default);
  }

  if (init_simulator_pose_) {
    cli_set_simulator_pose_ = proxy.create_client<InitializePose>(
      "/api/simulator/set/pose", rmw_qos_profile_services_default);
    pub_initialpose2d_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose2d", rclcpp::QoS(1));
  }
}

void InitialPose::setInitializePose(
  const autoware_external_api_msgs::srv::InitializePose::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::InitializePose::Response::SharedPtr response)
{
  response->status = autoware_api_utils::response_ignored("No processing.");

  if (init_simulator_pose_) {
    const auto [status, resp] = cli_set_simulator_pose_->call(request);
    if (!autoware_api_utils::is_success(status)) {
      response->status = status;
      return;
    }
    pub_initialpose2d_->publish(request->pose);
    response->status = resp->status;
  }

  if (init_localization_pose_) {
    const auto req = std::make_shared<PoseWithCovarianceStampedSrv::Request>();
    req->pose_with_covariance = request->pose;
    const auto [status, resp] = cli_set_initialize_pose_->call(req);
    if (!autoware_api_utils::is_success(status)) {
      response->status = status;
      return;
    }
    if (resp->success) {
      response->status = autoware_api_utils::response_success();
    } else {
      response->status = autoware_api_utils::response_error("Internal service failed.");
    }
  }
}

void InitialPose::setInitializePoseAuto(
  const autoware_external_api_msgs::srv::InitializePoseAuto::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::InitializePoseAuto::Response::SharedPtr response)
{
  response->status = autoware_api_utils::response_ignored("No processing.");

  if (init_localization_pose_) {
    const auto [status, resp] = cli_set_initialize_pose_auto_->call(request);
    if (!autoware_api_utils::is_success(status)) {
      response->status = status;
      return;
    }
    response->status = resp->status;
  }
}

}  // namespace internal_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(internal_api::InitialPose)
