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

#ifndef VERSION_HPP_
#define VERSION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "autoware_api_utils/autoware_api_utils.hpp"
#include "autoware_external_api_msgs/srv/get_version.hpp"

namespace external_api
{

class Version : public rclcpp::Node
{
public:
  explicit Version(const rclcpp::NodeOptions & options);

private:
  // ros interface
  rclcpp::CallbackGroup::SharedPtr group_;
  autoware_api_utils::Service<autoware_external_api_msgs::srv::GetVersion>::SharedPtr srv_;

  // ros callback
  void getVersion(
    const autoware_external_api_msgs::srv::GetVersion::Request::SharedPtr request,
    const autoware_external_api_msgs::srv::GetVersion::Response::SharedPtr response);
};

}  // namespace external_api

#endif  // VERSION_HPP_
