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

#include <autoware_auto_mapping_msgs/msg/had_map_segment.hpp>
#include <autoware_auto_mapping_msgs/msg/map_primitive.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <autoware_external_api_msgs/msg/route.hpp>
#include <autoware_external_api_msgs/msg/route_section.hpp>

#include <memory>

namespace
{
auto convertRoute(const autoware_external_api_msgs::msg::Route & route)
{
  // there is no input for start_pose
  autoware_auto_planning_msgs::msg::HADMapRoute msg;
  msg.header = route.goal_pose.header;
  msg.goal_pose = route.goal_pose.pose;
  for (const auto & section : route.route_sections) {
    autoware_auto_mapping_msgs::msg::HADMapSegment segment;
    segment.preferred_primitive_id = section.preferred_lane_id;
    for (const auto & lane_id : section.lane_ids) {
      autoware_auto_mapping_msgs::msg::MapPrimitive primitive;
      primitive.id = lane_id;
      segment.primitives.push_back(primitive);
    }
    msg.segments.push_back(segment);
  }
  return msg;
}

auto convertRoute(const autoware_auto_planning_msgs::msg::HADMapRoute & route)
{
  // there is no input for continued_lane_ids
  autoware_external_api_msgs::msg::Route msg;
  msg.goal_pose.header = route.header;
  msg.goal_pose.pose = route.goal_pose;
  for (const auto & segment : route.segments) {
    autoware_external_api_msgs::msg::RouteSection section;
    section.preferred_lane_id = segment.preferred_primitive_id;
    for (const auto & primitive : segment.primitives) {
      section.lane_ids.push_back(primitive.id);
    }
    msg.route_sections.push_back(section);
  }
  return msg;
}

}  // namespace

namespace internal_api
{
Route::Route(const rclcpp::NodeOptions & options) : Node("external_api_route", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  autoware_api_utils::ServiceProxyNodeInterface proxy(this);

  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_clear_route_ = proxy.create_service<autoware_external_api_msgs::srv::ClearRoute>(
    "/api/autoware/set/clear_route", std::bind(&Route::clearRoute, this, _1, _2),
    rmw_qos_profile_services_default, group_);
  srv_set_route_ = proxy.create_service<autoware_external_api_msgs::srv::SetRoute>(
    "/api/autoware/set/route", std::bind(&Route::setRoute, this, _1, _2),
    rmw_qos_profile_services_default, group_);
  srv_set_goal_ = proxy.create_service<autoware_external_api_msgs::srv::SetPose>(
    "/api/autoware/set/goal", std::bind(&Route::setGoal, this, _1, _2),
    rmw_qos_profile_services_default, group_);
  srv_set_checkpoint_ = proxy.create_service<autoware_external_api_msgs::srv::SetPose>(
    "/api/autoware/set/checkpoint", std::bind(&Route::setCheckpoint, this, _1, _2),
    rmw_qos_profile_services_default, group_);

  pub_get_route_ = create_publisher<autoware_external_api_msgs::msg::Route>(
    "/api/autoware/get/route", rclcpp::QoS(1).transient_local());

  cli_clear_route_ = proxy.create_client<std_srvs::srv::Trigger>("/autoware/reset_route");
  sub_planning_route_ = create_subscription<autoware_auto_planning_msgs::msg::HADMapRoute>(
    "/planning/mission_planning/route", rclcpp::QoS(1).transient_local(),
    std::bind(&Route::onRoute, this, _1));
  pub_planning_route_ = create_publisher<autoware_auto_planning_msgs::msg::HADMapRoute>(
    "/planning/mission_planning/route", rclcpp::QoS(1).transient_local());
  pub_planning_goal_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/planning/mission_planning/goal", rclcpp::QoS(1));
  pub_planning_checkpoint_ = create_publisher<geometry_msgs::msg::PoseStamped>(
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
  pub_planning_route_->publish(convertRoute(request->route));
  response->status = autoware_api_utils::response_success();
}

void Route::setGoal(
  const autoware_external_api_msgs::srv::SetPose::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::SetPose::Response::SharedPtr response)
{
  pub_planning_goal_->publish(request->pose);
  response->status = autoware_api_utils::response_success();
}

void Route::setCheckpoint(
  const autoware_external_api_msgs::srv::SetPose::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::SetPose::Response::SharedPtr response)
{
  pub_planning_checkpoint_->publish(request->pose);
  response->status = autoware_api_utils::response_success();
}

void Route::onRoute(const autoware_auto_planning_msgs::msg::HADMapRoute::ConstSharedPtr message)
{
  pub_get_route_->publish(convertRoute(*message));
}

}  // namespace internal_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(internal_api::Route)
