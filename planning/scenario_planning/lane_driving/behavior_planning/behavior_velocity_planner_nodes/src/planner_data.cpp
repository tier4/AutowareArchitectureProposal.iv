// Copyright 2021 The Autoware Foundation
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

#include "behavior_velocity_planner_nodes/planner_data.hpp"

namespace autoware
{
namespace planning
{
namespace behavior_velocity_planner_nodes
{

bool PlannerData::isVehicleStopped(
  const double stop_duration) const
{
  if (velocity_buffer.empty()) {return false;}

  // Get velocities within stop_duration
  const auto now = rclcpp::Clock{RCL_ROS_TIME}.now();
  std::vector<double> vs;
  for (const auto & velocity : velocity_buffer) {
    vs.push_back(velocity.twist.linear.x);

    const auto time_diff = now - velocity.header.stamp;
    if (time_diff.seconds() >= stop_duration) {break;}
  }

  // Check all velocities
  constexpr double stop_velocity = 0.1;
  for (const auto & v : vs) {
    if (v >= stop_velocity) {return false;}
  }

  return true;
}

void PlannerData::updateCurrentAcc()
{
  if (prev_velocity_) {
    const double dv = current_velocity->twist.linear.x - prev_velocity_->twist.linear.x;
    const double dt = std::max(
      (rclcpp::Time(current_velocity->header.stamp) - rclcpp::Time(prev_velocity_->header.stamp))
        .seconds(),
      1e-03);
    const double accel = dv / dt;
    // apply lowpass filter
    current_accel = accel_lowpass_gain_ * accel + (1.0 - accel_lowpass_gain_) * prev_accel_;
  } else {
    current_accel = 0.0;
  }

  prev_velocity_ = current_velocity;
  prev_accel_ = current_accel;
}
}  // namespace behavior_velocity_planner_nodes
}  // namespace planning
}  // namespace autoware
