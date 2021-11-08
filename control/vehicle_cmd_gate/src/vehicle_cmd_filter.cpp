// Copyright 2015-2019 Autoware Foundation
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
#include "vehicle_cmd_gate/vehicle_cmd_filter.hpp"

#include <algorithm>
#include <cmath>

VehicleCmdFilter::VehicleCmdFilter() {}

void VehicleCmdFilter::limitLongitudinalWithVel(
  autoware_auto_control_msgs::msg::AckermannControlCommand & input) const
{
  input.longitudinal.speed = std::max(std::min(input.longitudinal.speed, vel_lim_), -vel_lim_);
}

void VehicleCmdFilter::limitLongitudinalWithAcc(
  const float dt, autoware_auto_control_msgs::msg::AckermannControlCommand & input) const
{
  input.longitudinal.acceleration =
    std::max(std::min(input.longitudinal.acceleration, lon_acc_lim_), -lon_acc_lim_);
  input.longitudinal.speed =
    limitDiff(input.longitudinal.speed, prev_cmd_.longitudinal.speed, lon_acc_lim_ * dt);
}

void VehicleCmdFilter::VehicleCmdFilter::limitLongitudinalWithJerk(
  const float dt, autoware_auto_control_msgs::msg::AckermannControlCommand & input) const
{
  input.longitudinal.acceleration = limitDiff(
    input.longitudinal.acceleration, prev_cmd_.longitudinal.acceleration, lon_jerk_lim_ * dt);
}

void VehicleCmdFilter::limitLateralWithLatAcc(
  [[maybe_unused]] const float dt,
  autoware_auto_control_msgs::msg::AckermannControlCommand & input) const
{
  float latacc = calcLatAcc(input);
  if (std::fabs(latacc) > lat_acc_lim_) {
    constexpr float min_v = 0.001;
    float v_sq = std::max(input.longitudinal.speed * input.longitudinal.speed, min_v);
    float steer_lim = std::atan(lat_acc_lim_ * wheel_base_ / v_sq);
    input.lateral.steering_tire_angle = latacc > 0.0 ? steer_lim : -steer_lim;
  }
}

void VehicleCmdFilter::limitLateralWithLatJerk(
  const float dt, autoware_auto_control_msgs::msg::AckermannControlCommand & input) const
{
  float curr_latacc = calcLatAcc(input);
  float prev_latacc = calcLatAcc(prev_cmd_);

  const float latacc_max = prev_latacc + lat_jerk_lim_ * dt;
  const float latacc_min = prev_latacc - lat_jerk_lim_ * dt;

  if (curr_latacc > latacc_max) {
    input.lateral.steering_tire_angle = calcSteerFromLatacc(input.longitudinal.speed, latacc_max);
  } else if (curr_latacc < latacc_min) {
    input.lateral.steering_tire_angle = calcSteerFromLatacc(input.longitudinal.speed, latacc_min);
  }
}

auto VehicleCmdFilter::calcSteerFromLatacc(const float v, const float latacc) const
{
  constexpr float min_v = 0.001;
  const float v_sq = std::max(v * v, min_v);
  return std::atan(latacc * wheel_base_ / v_sq);
}

auto VehicleCmdFilter::calcLatAcc(
  const autoware_auto_control_msgs::msg::AckermannControlCommand & cmd) const
{
  float v = cmd.longitudinal.speed;
  return v * v * std::tan(cmd.lateral.steering_tire_angle) / wheel_base_;
}

auto VehicleCmdFilter::limitDiff(const float curr, const float prev, const float diff_lim) const
{
  float diff = std::max(std::min(curr - prev, diff_lim), -diff_lim);
  return prev + diff;
}
