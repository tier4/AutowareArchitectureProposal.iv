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

#pragma once

#include <string>

#include <ros/ros.h>

#include <autoware_system_msgs/AutowareState.h>
#include <std_msgs/Bool.h>

struct TimeoutThreshold
{
  double InitializingVehicle;
  double WaitingForRoute;
  double Planning;
};

class StateTimeoutChecker
{
public:
  StateTimeoutChecker();

private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  // Parameter
  double update_rate_;
  TimeoutThreshold th_timeout_;

  // Subscriber
  ros::Subscriber sub_autoware_state_;
  ros::Time autoware_state_received_time_;

  autoware_system_msgs::AutowareState::ConstPtr autoware_state_;

  void onAutowareState(const autoware_system_msgs::AutowareState::ConstPtr & msg);

  // Publisher
  ros::Publisher pub_is_state_timeout_;

  // Timer
  ros::Timer timer_;

  bool isDataReady();
  void onTimer(const ros::TimerEvent & event);

  // Algorithm
  ros::Time state_change_time_;

  bool isStateTimeout();
};
