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
#include "autoware_planning_msgs/msg/route_section.hpp"

namespace
{

autoware_planning_msgs::msg::RouteSection convertRouteSection(
  const autoware_external_api_msgs::msg::RouteSection & section)
{
  return autoware_planning_msgs::build<autoware_planning_msgs::msg::RouteSection>()
         .lane_ids(section.lane_ids)
         .preferred_lane_id(section.preferred_lane_id)
         .continued_lane_ids(section.continued_lane_ids);
}

autoware_planning_msgs::msg::Route convertRoute(
  const autoware_external_api_msgs::msg::Route & route)
{
  autoware_planning_msgs::msg::Route::_route_sections_type route_sections;
  route_sections.reserve(route.route_sections.size());
  for (const auto & section : route.route_sections) {
    route_sections.push_back(convertRouteSection(section));
  }

  return autoware_planning_msgs::build<autoware_planning_msgs::msg::Route>()
         .header(route.goal_pose.header)
         .goal_pose(route.goal_pose.pose)
         .route_sections(route_sections);
}

}  // namespace

namespace internal_api
{

Route::Route(const rclcpp::NodeOptions & options)
: Node("external_api_route", options)
{
  using namespace std::placeholders;
  autoware_api_utils::ServiceProxyNodeInterface proxy(this);

  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_clear_route_ = proxy.create_service<autoware_external_api_msgs::srv::ClearRoute>(
    "/api/autoware/set/clear_route",
    std::bind(&Route::clearRoute, this, _1, _2),
    rmw_qos_profile_services_default, group_);
  srv_set_route_ = proxy.create_service<autoware_external_api_msgs::srv::SetRoute>(
    "/api/autoware/set/route",
    std::bind(&Route::setRoute, this, _1, _2),
    rmw_qos_profile_services_default, group_);
  srv_set_goal_ = proxy.create_service<autoware_external_api_msgs::srv::SetPose>(
    "/api/autoware/set/goal",
    std::bind(&Route::setGoal, this, _1, _2),
    rmw_qos_profile_services_default, group_);
  srv_set_checkpoint_ = proxy.create_service<autoware_external_api_msgs::srv::SetPose>(
    "/api/autoware/set/checkpoint",
    std::bind(&Route::setCheckpoint, this, _1, _2),
    rmw_qos_profile_services_default, group_);

  cli_clear_route_ = proxy.create_client<std_srvs::srv::Trigger>(
    "/autoware/reset_route");
  pub_set_route_ = create_publisher<autoware_planning_msgs::msg::Route>(
    "/planning/mission_planning/route", rclcpp::QoS(1).transient_local());
  pub_set_goal_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/planning/mission_planning/goal", rclcpp::QoS(1));
  pub_set_checkpoint_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/planning/mission_planning/checkpoint", rclcpp::QoS(1));
}

void Route::clearRoute(
  const autoware_external_api_msgs::srv::ClearRoute::Request::SharedPtr,
  const autoware_external_api_msgs::srv::ClearRoute::Response::SharedPtr response)
{
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto [status, resp] = cli_clear_route_->call(req);
  if (!autoware_api_utils::is_success(status)) {
    response->status = status;
    return;
  }
  if (resp->success) {
    response->status = autoware_api_utils::response_success(resp->message);
  } else {
    response->status = autoware_api_utils::response_error(resp->message);
  }
}

void Route::setRoute(
  const autoware_external_api_msgs::srv::SetRoute::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::SetRoute::Response::SharedPtr response)
{
  pub_set_route_->publish(convertRoute(request->route));
  response->status = autoware_api_utils::response_success();
}

void Route::setGoal(
  const autoware_external_api_msgs::srv::SetPose::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::SetPose::Response::SharedPtr response)
{
  pub_set_goal_->publish(request->pose);
  response->status = autoware_api_utils::response_success();
}

void Route::setCheckpoint(
  const autoware_external_api_msgs::srv::SetPose::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::SetPose::Response::SharedPtr response)
{
  pub_set_checkpoint_->publish(request->pose);
  response->status = autoware_api_utils::response_success();
}

}  // namespace internal_api

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(internal_api::Route)
