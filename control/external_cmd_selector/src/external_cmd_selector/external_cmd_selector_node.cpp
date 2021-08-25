// Copyright 2020 Tier IV, Inc.
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

#include <chrono>
#include <utility>
#include <memory>
#include <string>

#include "external_cmd_selector/external_cmd_selector_node.hpp"

ExternalCmdSelector::ExternalCmdSelector(const rclcpp::NodeOptions & node_options)
: Node("external_cmd_selector", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Parameter
  double update_rate = declare_parameter("update_rate", 10.0);
  std::string initial_selector_mode = declare_parameter("initial_selector_mode", "local");

  // Publisher
  pub_current_selector_mode_ =
    create_publisher<ExternalCommandSelectorMode>("~/output/current_selector_mode", 1);
  pub_control_cmd_ =
    create_publisher<ExternalControlCommand>("~/output/control_cmd", 1);
  pub_shift_cmd_ =
    create_publisher<ShiftCommand>("~/output/shift_cmd", 1);
  pub_turn_signal_cmd_ =
    create_publisher<TurnSignalCommand>("~/output/turn_signal_cmd", 1);
  pub_heartbeat_ =
    create_publisher<EmergencyMode>("~/output/heartbeat", 1);

  // Callback Groups
  callback_group_subscribers_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_services_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  auto subscriber_option = rclcpp::SubscriptionOptions();
  subscriber_option.callback_group = callback_group_subscribers_;

  // Subscriber
  sub_local_control_cmd_ = create_subscription<ExternalControlCommand>(
    "~/input/local/control_cmd", 1,
    std::bind(&ExternalCmdSelector::onLocalControlCmd, this, _1), subscriber_option);
  sub_local_shift_cmd_ = create_subscription<ShiftCommand>(
    "~/input/local/shift_cmd", 1,
    std::bind(&ExternalCmdSelector::onLocalShiftCmd, this, _1), subscriber_option);
  sub_local_turn_signal_cmd_ = create_subscription<TurnSignalCommand>(
    "~/input/local/turn_signal_cmd", 1,
    std::bind(&ExternalCmdSelector::onLocalTurnSignalCmd, this, _1), subscriber_option);
  sub_local_heartbeat_ = create_subscription<EmergencyMode>(
    "~/input/local/heartbeat", 1,
    std::bind(&ExternalCmdSelector::onLocalHeartbeat, this, _1), subscriber_option);

  sub_remote_control_cmd_ = create_subscription<ExternalControlCommand>(
    "~/input/remote/control_cmd", 1,
    std::bind(&ExternalCmdSelector::onRemoteControlCmd, this, _1), subscriber_option);
  sub_remote_shift_cmd_ = create_subscription<ShiftCommand>(
    "~/input/remote/shift_cmd", 1,
    std::bind(&ExternalCmdSelector::onRemoteShiftCmd, this, _1), subscriber_option);
  sub_remote_turn_signal_cmd_ = create_subscription<TurnSignalCommand>(
    "~/input/remote/turn_signal_cmd", 1,
    std::bind(&ExternalCmdSelector::onRemoteTurnSignalCmd, this, _1), subscriber_option);
  sub_remote_heartbeat_ = create_subscription<EmergencyMode>(
    "~/input/remote/heartbeat", 1,
    std::bind(&ExternalCmdSelector::onRemoteHeartbeat, this, _1), subscriber_option);

  // Service
  srv_select_external_command_ = create_service<ExternalCommandSelect>(
    "~/service/select_external_command",
    std::bind(&ExternalCmdSelector::onSelectExternalCommandService, this, _1, _2),
    rmw_qos_profile_services_default, callback_group_services_);

  // Initialize mode
  auto convert_selector_mode = [](const std::string & mode_text)
    {
      if (mode_text == "local") {
        return ExternalCommandSelectorMode::LOCAL;
      }
      if (mode_text == "remote") {
        return ExternalCommandSelectorMode::REMOTE;
      }
      throw std::invalid_argument("unknown selector mode");
    };
  current_selector_mode_.data = convert_selector_mode(initial_selector_mode);

  // Timer
  auto timer_callback = std::bind(&ExternalCmdSelector::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / update_rate));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    get_clock(), period, std::move(timer_callback),
    get_node_base_interface()->get_context());
  get_node_timers_interface()->add_timer(timer_, callback_group_subscribers_);
}

void ExternalCmdSelector::onLocalControlCmd(const ExternalControlCommand::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != ExternalCommandSelectorMode::LOCAL) {
    return;
  }
  pub_control_cmd_->publish(*msg);
}

void ExternalCmdSelector::onLocalShiftCmd(const ShiftCommand::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != ExternalCommandSelectorMode::LOCAL) {
    return;
  }
  pub_shift_cmd_->publish(*msg);
}

void ExternalCmdSelector::onLocalTurnSignalCmd(const TurnSignalCommand::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != ExternalCommandSelectorMode::LOCAL) {
    return;
  }
  pub_turn_signal_cmd_->publish(*msg);
}

void ExternalCmdSelector::onLocalHeartbeat(const EmergencyMode::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != ExternalCommandSelectorMode::LOCAL) {
    return;
  }
  pub_heartbeat_->publish(*msg);
}

void ExternalCmdSelector::onRemoteControlCmd(const ExternalControlCommand::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != ExternalCommandSelectorMode::REMOTE) {
    return;
  }
  pub_control_cmd_->publish(*msg);
}

void ExternalCmdSelector::onRemoteShiftCmd(const ShiftCommand::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != ExternalCommandSelectorMode::REMOTE) {
    return;
  }
  pub_shift_cmd_->publish(*msg);
}

void ExternalCmdSelector::onRemoteTurnSignalCmd(const TurnSignalCommand::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != ExternalCommandSelectorMode::REMOTE) {
    return;
  }
  pub_turn_signal_cmd_->publish(*msg);
}

void ExternalCmdSelector::onRemoteHeartbeat(const EmergencyMode::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != ExternalCommandSelectorMode::REMOTE) {
    return;
  }
  pub_heartbeat_->publish(*msg);
}

bool ExternalCmdSelector::onSelectExternalCommandService(
  const ExternalCommandSelect::Request::SharedPtr req,
  const ExternalCommandSelect::Response::SharedPtr res)
{
  current_selector_mode_.data = req->mode.data;
  res->success = true;
  res->message = "Success.";
  return true;
}

void ExternalCmdSelector::onTimer()
{
  pub_current_selector_mode_->publish(current_selector_mode_);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ExternalCmdSelector)
