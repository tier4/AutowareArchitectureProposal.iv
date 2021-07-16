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

#include "diagnostics.hpp"

namespace external_api
{

Diagnostics::Diagnostics(const rclcpp::NodeOptions & options)
: Node("external_api_diagnostics", options)
{
  using namespace std::literals::chrono_literals;

  pub_ = create_publisher<autoware_external_api_msgs::msg::ClassifiedDiagnostics>(
    "/api/external/get/diagnostics", rclcpp::QoS(1));
  timer_ = rclcpp::create_timer(
    this, get_clock(), 200ms, std::bind(&Diagnostics::onTimer, this));
}

void Diagnostics::onTimer()
{
  autoware_external_api_msgs::msg::ClassifiedDiagnostics msg;
  msg.stamp = now();
  pub_->publish(msg);
}

}  // namespace external_api

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::Diagnostics)
