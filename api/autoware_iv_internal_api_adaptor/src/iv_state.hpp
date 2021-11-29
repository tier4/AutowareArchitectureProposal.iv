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

#ifndef IV_STATE_HPP_
#define IV_STATE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <autoware_auto_system_msgs/msg/emergency_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <autoware_vehicle_msgs/msg/control_mode.hpp>

namespace internal_api
{
class IVState : public rclcpp::Node
{
public:
  explicit IVState(const rclcpp::NodeOptions & options);

private:
  using EmergencyStateAuto = autoware_auto_system_msgs::msg::EmergencyState;
  using AutowareStateAuto = autoware_auto_system_msgs::msg::AutowareState;
  using AutowareStateIV = autoware_system_msgs::msg::AutowareState;
  rclcpp::Subscription<EmergencyStateAuto>::SharedPtr sub_emergency_;
  rclcpp::Subscription<AutowareStateAuto>::SharedPtr sub_state_;
  rclcpp::Publisher<AutowareStateIV>::SharedPtr pub_state_;

  using ControlModeAuto = autoware_auto_vehicle_msgs::msg::ControlModeReport;
  using ControlModeIV = autoware_vehicle_msgs::msg::ControlMode;
  rclcpp::Subscription<ControlModeAuto>::SharedPtr sub_control_mode_;
  rclcpp::Publisher<ControlModeIV>::SharedPtr pub_control_mode_;

  void onState(const AutowareStateAuto::ConstSharedPtr message);
  void onEmergency(const EmergencyStateAuto::ConstSharedPtr message);
  void onControlMode(const ControlModeAuto::ConstSharedPtr message);

  EmergencyStateAuto::ConstSharedPtr emergency_;
};

}  // namespace internal_api

#endif  // IV_STATE_HPP_
