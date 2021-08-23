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

#ifndef OPERATOR_HPP_
#define OPERATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "autoware_api_utils/autoware_api_utils.hpp"
#include "autoware_external_api_msgs/srv/set_operator.hpp"
#include "autoware_external_api_msgs/srv/set_observer.hpp"
#include "autoware_external_api_msgs/msg/operator.hpp"
#include "autoware_external_api_msgs/msg/observer.hpp"
#include "autoware_control_msgs/srv/remote_command_select.hpp"
#include "autoware_control_msgs/msg/remote_command_selector_mode.hpp"
#include "autoware_control_msgs/msg/gate_mode.hpp"
#include "autoware_vehicle_msgs/msg/engage.hpp"
#include "autoware_vehicle_msgs/msg/control_mode.hpp"

namespace internal_api
{

class Operator : public rclcpp::Node
{
public:
  explicit Operator(const rclcpp::NodeOptions & options);

private:
  using SetOperator = autoware_external_api_msgs::srv::SetOperator;
  using SetObserver = autoware_external_api_msgs::srv::SetObserver;
  using GetOperator = autoware_external_api_msgs::msg::Operator;
  using GetObserver = autoware_external_api_msgs::msg::Observer;
  using RemoteCommandSelect = autoware_control_msgs::srv::RemoteCommandSelect;
  using RemoteCommandSelectorMode = autoware_control_msgs::msg::RemoteCommandSelectorMode;
  using GateMode = autoware_control_msgs::msg::GateMode;
  using VehicleEngage = autoware_vehicle_msgs::msg::Engage;
  using VehicleControlMode = autoware_vehicle_msgs::msg::ControlMode;

  // ros interface
  rclcpp::CallbackGroup::SharedPtr group_;
  autoware_api_utils::Service<SetOperator>::SharedPtr srv_set_operator_;
  autoware_api_utils::Service<SetObserver>::SharedPtr srv_set_observer_;
  autoware_api_utils::Client<RemoteCommandSelect>::SharedPtr cli_external_select_;
  rclcpp::Publisher<GateMode>::SharedPtr pub_gate_mode_;
  rclcpp::Publisher<VehicleEngage>::SharedPtr pub_vehicle_engage_;
  rclcpp::Publisher<GetOperator>::SharedPtr pub_operator_;
  rclcpp::Publisher<GetObserver>::SharedPtr pub_observer_;
  rclcpp::Subscription<RemoteCommandSelectorMode>::SharedPtr sub_external_select_;
  rclcpp::Subscription<GateMode>::SharedPtr sub_gate_mode_;
  rclcpp::Subscription<VehicleControlMode>::SharedPtr sub_vehicle_control_mode_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ros callback
  void setOperator(
    const autoware_external_api_msgs::srv::SetOperator::Request::SharedPtr request,
    const autoware_external_api_msgs::srv::SetOperator::Response::SharedPtr response);
  void setObserver(
    const autoware_external_api_msgs::srv::SetObserver::Request::SharedPtr request,
    const autoware_external_api_msgs::srv::SetObserver::Response::SharedPtr response);
  void onExternalSelect(
    const autoware_control_msgs::msg::RemoteCommandSelectorMode::ConstSharedPtr message);
  void onGateMode(
    const autoware_control_msgs::msg::GateMode::ConstSharedPtr message);
  void onVehicleControlMode(
    const autoware_vehicle_msgs::msg::ControlMode::ConstSharedPtr message);
  void onTimer();

  // class field
  autoware_control_msgs::msg::RemoteCommandSelectorMode::ConstSharedPtr external_select_;
  autoware_control_msgs::msg::GateMode::ConstSharedPtr gate_mode_;
  autoware_vehicle_msgs::msg::ControlMode::ConstSharedPtr vehicle_control_mode_;

  // class method
  void publishOperator();
  void publishObserver();
  void setVehicleEngage(bool engage);
  void setGateMode(autoware_control_msgs::msg::GateMode::_data_type data);
  autoware_external_api_msgs::msg::ResponseStatus setExternalSelect(
    autoware_control_msgs::msg::RemoteCommandSelectorMode::_data_type data);
};

}  // namespace internal_api

#endif  // OPERATOR_HPP_
