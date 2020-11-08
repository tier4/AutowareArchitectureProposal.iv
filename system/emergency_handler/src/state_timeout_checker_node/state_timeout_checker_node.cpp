/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <emergency_handler/state_timeout_checker_node.h>

namespace
{
bool isTimeout(const ros::Time & state_change_time, const double th_timeout)
{
  if (th_timeout == 0.0) {
    return false;
  }

  const auto time_diff = ros::Time::now() - state_change_time;
  return time_diff.toSec() >= th_timeout;
}
}  // namespace

StateTimeoutChecker::StateTimeoutChecker()
{
  // Parameter
  private_nh_.param("update_rate", update_rate_, 10.0);
  private_nh_.param("timeout/InitializingVehicle", th_timeout_.InitializingVehicle, 60.0);
  private_nh_.param("timeout/WaitingForRoute", th_timeout_.WaitingForRoute, 60.0);
  private_nh_.param("timeout/Planning", th_timeout_.Planning, 60.0);

  // Subscriber
  sub_autoware_state_ =
    private_nh_.subscribe("input/autoware_state", 1, &StateTimeoutChecker::onAutowareState, this);

  // Publisher
  pub_is_state_timeout_ = private_nh_.advertise<std_msgs::Bool>("output/is_state_timeout", 1);

  // Timer
  timer_ = private_nh_.createTimer(ros::Rate(update_rate_), &StateTimeoutChecker::onTimer, this);
}

void StateTimeoutChecker::onAutowareState(const autoware_system_msgs::AutowareState::ConstPtr & msg)
{
  if (!autoware_state_ || autoware_state_->state != msg->state) {
    state_change_time_ = ros::Time::now();
  }

  autoware_state_ = msg;
  autoware_state_received_time_ = ros::Time::now();
}

bool StateTimeoutChecker::isDataReady()
{
  if (!autoware_state_) {
    ROS_INFO_THROTTLE(5.0, "waiting for autoware_state msg...");
    return false;
  }

  return true;
}

void StateTimeoutChecker::onTimer(const ros::TimerEvent & event)
{
  if (!isDataReady()) {
    return;
  }

  // Check timeout
  std_msgs::Bool is_state_timeout;
  is_state_timeout.data = isStateTimeout();
  pub_is_state_timeout_.publish(is_state_timeout);
}

bool StateTimeoutChecker::isStateTimeout()
{
  using autoware_system_msgs::AutowareState;

  constexpr double th_autoware_state_timeout = 30.0;
  if ((ros::Time::now() - autoware_state_received_time_).toSec() > th_autoware_state_timeout) {
    return true;
  }

  if (autoware_state_->state == AutowareState::InitializingVehicle)
    return isTimeout(state_change_time_, th_timeout_.InitializingVehicle);

  if (autoware_state_->state == AutowareState::WaitingForRoute)
    return isTimeout(state_change_time_, th_timeout_.WaitingForRoute);

  if (autoware_state_->state == AutowareState::Planning)
    return isTimeout(state_change_time_, th_timeout_.Planning);

  return false;
}
