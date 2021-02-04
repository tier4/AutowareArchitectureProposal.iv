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

#include <deque>
#include <vector>

#define FMT_HEADER_ONLY
#include "fmt/format.h"

#include "autoware_state_monitor/state_machine.hpp"

namespace
{
double calcDistance2d(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

double calcDistance2d(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2)
{
  return calcDistance2d(p1.position, p2.position);
}

bool isNearGoal(
  const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Pose & goal_pose,
  const double th_dist)
{
  return calcDistance2d(current_pose, goal_pose) < th_dist;
}

bool isStopped(
  const std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> & twist_buffer,
  const double th_stopped_velocity_mps)
{
  for (const auto & twist : twist_buffer) {
    if (std::abs(twist->twist.linear.x) > th_stopped_velocity_mps) {
      return false;
    }
  }
  return true;
}

template<class T>
std::vector<T> filterConfigByModuleName(const std::vector<T> & configs, const char * module_name)
{
  std::vector<T> filtered;

  for (const auto & config : configs) {
    if (config.module == module_name) {
      filtered.push_back(config);
    }
  }

  return filtered;
}

}  // namespace

bool StateMachine::isModuleInitialized(const char * module_name) const
{
  const auto non_received_topics =
    filterConfigByModuleName(state_input_.topic_stats.non_received_list, module_name);
  const auto non_set_params =
    filterConfigByModuleName(state_input_.param_stats.non_set_list, module_name);
  const auto non_received_tfs =
    filterConfigByModuleName(state_input_.tf_stats.non_received_list, module_name);

  if (non_received_topics.empty() && non_set_params.empty() && non_received_tfs.empty()) {
    return true;
  }

  for (const auto & topic_config : non_received_topics) {
    const auto msg = fmt::format("topic `{}` is not received yet", topic_config.name);
    msgs_.push_back(msg);
  }

  for (const auto & param_config : non_set_params) {
    const auto msg = fmt::format("param `{}` is not set", param_config.name);
    msgs_.push_back(msg);
  }

  for (const auto & tf_config : non_received_tfs) {
    const auto msg =
      fmt::format("tf from `{}` to `{}` is not received yet", tf_config.from, tf_config.to);
    msgs_.push_back(msg);
  }

  {
    const auto msg = fmt::format("module `{}` is not initialized", module_name);
    msgs_.push_back(msg);
  }

  return false;
}

bool StateMachine::isVehicleInitialized() const
{
  if (!isModuleInitialized(ModuleName::map)) {
    return false;
  }

  if (!isModuleInitialized(ModuleName::vehicle)) {
    return false;
  }

  if (!isModuleInitialized(ModuleName::sensing)) {
    return false;
  }

  if (!isModuleInitialized(ModuleName::localization)) {
    return false;
  }

  if (!isModuleInitialized(ModuleName::perception)) {
    return false;
  }

  // TODO(Kenji Miyake): Check if the vehicle is on a lane?

  return true;
}

bool StateMachine::hasRoute() const {return state_input_.route != nullptr;}

bool StateMachine::isRouteReceived() const {return state_input_.route != executing_route_;}

bool StateMachine::isPlanningCompleted() const
{
  if (!isModuleInitialized(ModuleName::planning)) {
    return false;
  }

  if (!isModuleInitialized(ModuleName::control)) {
    return false;
  }

  return true;
}

bool StateMachine::isEngaged() const
{
  if (!state_input_.autoware_engage) {
    return false;
  }

  if (state_input_.autoware_engage->is_engaged != 1) {
    return false;
  }

  if (!state_input_.vehicle_control_mode) {
    return false;
  }

  if (state_input_.vehicle_control_mode->data == autoware_vehicle_msgs::msg::ControlMode::MANUAL) {
    return false;
  }

  return true;
}

bool StateMachine::isOverridden() const {return !isEngaged();}

bool StateMachine::isEmergency() const
{
  if (!state_input_.emergency_mode) {
    return false;
  }

  return state_input_.emergency_mode->is_emergency;
}

bool StateMachine::hasArrivedGoal() const
{
  const auto is_near_goal = isNearGoal(
    state_input_.current_pose->pose, *state_input_.goal_pose, state_param_.th_arrived_distance_m);
  const auto is_stopped =
    isStopped(state_input_.twist_buffer, state_param_.th_stopped_velocity_mps);

  if (is_near_goal && is_stopped) {
    return true;
  }

  return false;
}

bool StateMachine::isFinalizing() const {return state_input_.is_finalizing;}

AutowareState StateMachine::updateState(const StateInput & state_input)
{
  msgs_ = {};
  state_input_ = state_input;
  autoware_state_ = judgeAutowareState();
  return autoware_state_;
}

AutowareState StateMachine::judgeAutowareState() const
{
  if (isFinalizing()) {
    return AutowareState::Finalizing;
  }

  if (autoware_state_ != AutowareState::Emergency && isEmergency()) {
    state_before_emergency_ = autoware_state_;
    return AutowareState::Emergency;
  }

  switch (autoware_state_) {
    case (AutowareState::InitializingVehicle): {
        if (isVehicleInitialized()) {
          if (!flags_.waiting_after_initializing) {
            flags_.waiting_after_initializing = true;
            times_.initializing_completed = state_input_.current_time;
            break;
          }

          // Wait after initialize completed to avoid sync error
          constexpr double wait_time_after_initializing = 1.0;
          const auto time_from_initializing =
            state_input_.current_time - times_.initializing_completed;
          if (time_from_initializing.seconds() > wait_time_after_initializing) {
            flags_.waiting_after_initializing = false;
            return AutowareState::WaitingForRoute;
          }
        }

        break;
      }

    case (AutowareState::WaitingForRoute): {
        if (isRouteReceived()) {
          return AutowareState::Planning;
        }

        if (hasRoute() && isEngaged() && !hasArrivedGoal()) {
          return AutowareState::Driving;
        }

        break;
      }

    case (AutowareState::Planning): {
        executing_route_ = state_input_.route;

        if (isPlanningCompleted()) {
          if (!flags_.waiting_after_planning) {
            flags_.waiting_after_planning = true;
            times_.planning_completed = state_input_.current_time;
            break;
          }

          // Wait after planning completed to avoid sync error
          constexpr double wait_time_after_planning = 1.0;
          const auto time_from_planning = state_input_.current_time - times_.planning_completed;
          if (time_from_planning.seconds() > wait_time_after_planning) {
            flags_.waiting_after_planning = false;
            return AutowareState::WaitingForEngage;
          }
        }

        break;
      }

    case (AutowareState::WaitingForEngage): {
        if (isRouteReceived()) {
          return AutowareState::Planning;
        }

        if (hasArrivedGoal()) {
          times_.arrived_goal = state_input_.current_time;
          return AutowareState::ArrivedGoal;
        }

        break;
      }

    case (AutowareState::Driving): {
        if (isRouteReceived()) {
          return AutowareState::Planning;
        }

        if (isOverridden()) {
          return AutowareState::WaitingForEngage;
        }

        if (hasArrivedGoal()) {
          times_.arrived_goal = state_input_.current_time;
          return AutowareState::ArrivedGoal;
        }

        break;
      }

    case (AutowareState::ArrivedGoal): {
        constexpr double wait_time_after_arrived_goal = 2.0;
        const auto time_from_arrived_goal = state_input_.current_time - times_.arrived_goal;
        if (time_from_arrived_goal.seconds() > wait_time_after_arrived_goal) {
          return AutowareState::WaitingForRoute;
        }

        break;
      }

    case (AutowareState::Emergency): {
        if (!isEmergency()) {
          return state_before_emergency_;
        }

        break;
      }

    case (AutowareState::Finalizing): {
        break;
      }

    default: {
        throw std::runtime_error("invalid state");
      }
  }

  // continue previous state when break
  return autoware_state_;
}
