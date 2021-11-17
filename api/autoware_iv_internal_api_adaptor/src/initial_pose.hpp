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

#ifndef INITIAL_POSE_HPP_
#define INITIAL_POSE_HPP_

#include <autoware_api_utils/autoware_api_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_external_api_msgs/srv/initialize_pose.hpp>
#include <autoware_external_api_msgs/srv/initialize_pose_auto.hpp>
#include <autoware_localization_msgs/srv/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace internal_api
{
class InitialPose : public rclcpp::Node
{
public:
  explicit InitialPose(const rclcpp::NodeOptions & options);

private:
  using InitializePose = autoware_external_api_msgs::srv::InitializePose;
  using InitializePoseAuto = autoware_external_api_msgs::srv::InitializePoseAuto;
  using PoseWithCovarianceStampedSrv = autoware_localization_msgs::srv::PoseWithCovarianceStamped;

  // ros parameter
  bool init_simulator_pose_;
  bool init_localization_pose_;

  // ros interface
  rclcpp::CallbackGroup::SharedPtr group_;
  autoware_api_utils::Service<InitializePose>::SharedPtr srv_set_initialize_pose_;
  autoware_api_utils::Service<InitializePoseAuto>::SharedPtr srv_set_initialize_pose_auto_;
  autoware_api_utils::Client<PoseWithCovarianceStampedSrv>::SharedPtr cli_set_initialize_pose_;
  autoware_api_utils::Client<InitializePoseAuto>::SharedPtr cli_set_initialize_pose_auto_;
  autoware_api_utils::Client<InitializePose>::SharedPtr cli_set_simulator_pose_;

  // TODO(Takagi, Isamu): workaround for topic check
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initialpose2d_;

  // ros callback
  void setInitializePose(
    const autoware_external_api_msgs::srv::InitializePose::Request::SharedPtr request,
    const autoware_external_api_msgs::srv::InitializePose::Response::SharedPtr response);
  void setInitializePoseAuto(
    const autoware_external_api_msgs::srv::InitializePoseAuto::Request::SharedPtr request,
    const autoware_external_api_msgs::srv::InitializePoseAuto::Response::SharedPtr response);
};

}  // namespace internal_api

#endif  // INITIAL_POSE_HPP_
