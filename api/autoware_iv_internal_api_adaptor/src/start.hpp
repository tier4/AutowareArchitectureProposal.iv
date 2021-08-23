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

#ifndef START_HPP_
#define START_HPP_

#include "rclcpp/rclcpp.hpp"
#include "autoware_api_utils/autoware_api_utils.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace internal_api
{

class Start : public rclcpp::Node
{
public:
  explicit Start(const rclcpp::NodeOptions & options);

private:
  using Trigger = std_srvs::srv::Trigger;

  // ros interface
  autoware_api_utils::Service<Trigger>::SharedPtr srv_set_request_start_;

  // ros callback
  void setRequestStart(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    const std_srvs::srv::Trigger::Response::SharedPtr response);
};

}  // namespace internal_api

#endif  // START_HPP_
