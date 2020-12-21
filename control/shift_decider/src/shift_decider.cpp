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

#include <cstddef>
#include <functional>
#include <memory>
#include <utility>

#include "shift_decider/shift_decider.hpp"

#include "rclcpp/timer.hpp"

ShiftDecider::ShiftDecider()
: Node("shift_decider")
{
  using std::placeholders::_1;

  static constexpr std::size_t queue_size = 1;
  rclcpp::QoS durable_qos(queue_size);
  durable_qos.transient_local();

  pub_shift_cmd_ =
    create_publisher<autoware_vehicle_msgs::msg::ShiftStamped>("output/shift_cmd", durable_qos);
  sub_control_cmd_ = create_subscription<autoware_control_msgs::msg::ControlCommandStamped>(
    "input/control_cmd", queue_size, std::bind(&ShiftDecider::onControlCmd, this, _1));

  initTimer(0.1);
}

void ShiftDecider::onControlCmd(autoware_control_msgs::msg::ControlCommandStamped::SharedPtr msg)
{
  control_cmd_ = msg;
}

void ShiftDecider::onTimer()
{
  if (!control_cmd_) {return;}

  updateCurrentShiftCmd();
  pub_shift_cmd_->publish(shift_cmd_);
}

void ShiftDecider::updateCurrentShiftCmd()
{
  shift_cmd_.header.stamp = now();
  static constexpr double vel_threshold = 0.01;  // to prevent chattering
  if (control_cmd_->control.velocity > vel_threshold) {
    shift_cmd_.shift.data = autoware_vehicle_msgs::msg::Shift::DRIVE;
  } else if (control_cmd_->control.velocity < -vel_threshold) {
    shift_cmd_.shift.data = autoware_vehicle_msgs::msg::Shift::REVERSE;
  }
}

void ShiftDecider::initTimer(double period_s)
{
  auto timer_callback = std::bind(&ShiftDecider::onTimer, this);
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period_ns, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}
