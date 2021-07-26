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

#ifndef DOOR_HPP_
#define DOOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "autoware_api_utils/autoware_api_utils.hpp"
#include "autoware_external_api_msgs/srv/set_door.hpp"

namespace external_api
{

class Door : public rclcpp::Node
{
public:
  explicit Door(const rclcpp::NodeOptions & options);

private:
  // ros interface
  rclcpp::CallbackGroup::SharedPtr group_;
  autoware_api_utils::Service<autoware_external_api_msgs::srv::SetDoor>::SharedPtr srv_;
  autoware_api_utils::Client<autoware_external_api_msgs::srv::SetDoor>::SharedPtr cli_;

  // ros callback
  void setDoor(
    const autoware_external_api_msgs::srv::SetDoor::Request::SharedPtr request,
    const autoware_external_api_msgs::srv::SetDoor::Response::SharedPtr response);
};

}  // namespace external_api

#endif  // DOOR_HPP_
