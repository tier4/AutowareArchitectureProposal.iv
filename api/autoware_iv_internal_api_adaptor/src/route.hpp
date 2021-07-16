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
#include "autoware_external_api_msgs/srv/set_pose.hpp"
#include "autoware_external_api_msgs/srv/set_route.hpp"
#include "autoware_planning_msgs/msg/route.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace internal_api
{

class Route : public rclcpp::Node
{
public:
  explicit Route(const rclcpp::NodeOptions & options);

private:
  // ros interface
  autoware_api_utils::Service<autoware_external_api_msgs::srv::SetRoute>::SharedPtr srv_route_;
  autoware_api_utils::Service<autoware_external_api_msgs::srv::SetPose>::SharedPtr srv_goal_;
  autoware_api_utils::Service<autoware_external_api_msgs::srv::SetPose>::SharedPtr srv_checkpoint_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Route>::SharedPtr pub_route_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_checkpoint_;

  // ros callback
  void setRoute(
    const autoware_external_api_msgs::srv::SetRoute::Request::SharedPtr request,
    const autoware_external_api_msgs::srv::SetRoute::Response::SharedPtr response);
  void setGoal(
    const autoware_external_api_msgs::srv::SetPose::Request::SharedPtr request,
    const autoware_external_api_msgs::srv::SetPose::Response::SharedPtr response);
  void setCheckpoint(
    const autoware_external_api_msgs::srv::SetPose::Request::SharedPtr request,
    const autoware_external_api_msgs::srv::SetPose::Response::SharedPtr response);
};

}  // namespace internal_api

#endif  // ROUTE_HPP_
