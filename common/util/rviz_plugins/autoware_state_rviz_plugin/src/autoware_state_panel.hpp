//
//  Copyright 2020 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef AUTOWARE_STATE_PANEL_HPP_
#define AUTOWARE_STATE_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <autoware_control_msgs/msg/gate_mode.hpp>
#include <autoware_external_api_msgs/msg/engage_status.hpp>
#include <autoware_external_api_msgs/srv/engage.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <autoware_vehicle_msgs/msg/shift_stamped.hpp>

namespace rviz_plugins
{
class AutowareStatePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit AutowareStatePanel(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:
  void onClickAutowareEngage();

protected:
  void onGateMode(const autoware_control_msgs::msg::GateMode::ConstSharedPtr msg);
  void onAutowareState(const autoware_system_msgs::msg::AutowareState::ConstSharedPtr msg);
  void onShift(const autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg);
  void onEngageStatus(const autoware_external_api_msgs::msg::EngageStatus::ConstSharedPtr msg);

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Subscription<autoware_control_msgs::msg::GateMode>::SharedPtr sub_gate_mode_;
  rclcpp::Subscription<autoware_system_msgs::msg::AutowareState>::SharedPtr sub_autoware_state_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr sub_gear_;
  rclcpp::Subscription<autoware_external_api_msgs::msg::EngageStatus>::SharedPtr sub_engage_;

  rclcpp::Client<autoware_external_api_msgs::srv::Engage>::SharedPtr client_engage_;

  QLabel * gate_mode_label_ptr_;
  QLabel * autoware_state_label_ptr_;
  QLabel * gear_label_ptr_;
  QLabel * engage_status_label_ptr_;
  QPushButton * engage_button_ptr_;
};

}  // namespace rviz_plugins

#endif  // AUTOWARE_STATE_PANEL_HPP_
