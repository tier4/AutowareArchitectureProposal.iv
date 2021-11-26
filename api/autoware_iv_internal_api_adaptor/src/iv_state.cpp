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

#include "iv_state.hpp"

#include <autoware_iv_auto_msgs_converter/autoware_iv_auto_msgs_converter.hpp>

namespace internal_api
{
IVState::IVState(const rclcpp::NodeOptions & options) : Node("external_api_iv_state", options)
{
  using std::placeholders::_1;

  pub_state_ = create_publisher<AutowareStateIV>("/api/autoware/get/iv_state", rclcpp::QoS(1));
  sub_state_ = create_subscription<AutowareStateAuto>(
    "/autoware/state", rclcpp::QoS(1), std::bind(&IVState::onState, this, _1));
  sub_emergency_ = create_subscription<EmergencyStateAuto>(
    "/system/emergency/emergency_state", rclcpp::QoS(1),
    std::bind(&IVState::onEmergency, this, _1));
}

void IVState::onState(const AutowareStateAuto::ConstSharedPtr message)
{
  auto state = autoware_iv_auto_msgs_converter::convert(*message);
  if (emergency_) {
    switch (emergency_->state) {
      case EmergencyStateAuto::MRM_OPERATING:
      case EmergencyStateAuto::MRM_SUCCEEDED:
      case EmergencyStateAuto::MRM_FAILED:
        state.state = AutowareStateIV::EMERGENCY;
        break;
    }
  }
  pub_state_->publish(state);
}

void IVState::onEmergency(const EmergencyStateAuto::ConstSharedPtr message)
{
  emergency_ = message;
}

}  // namespace internal_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(internal_api::IVState)
