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

#include <ros/ros.h>

#include <std_msgs/Bool.h>

template <class HeartbeatMsg>
class HeaderlessHeartbeatChecker
{
public:
  HeaderlessHeartbeatChecker(const char * topic_name, const double timeout) : timeout_(timeout)
  {
    sub_heartbeat_ =
      private_nh_.subscribe(topic_name, 1, &HeaderlessHeartbeatChecker::onHeartbeat, this);
  }

  bool isTimeout()
  {
    const auto time_from_last_heartbeat = ros::Time::now() - last_heartbeat_time_;
    return time_from_last_heartbeat.toSec() > timeout_;
  }

private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  // Parameter
  double timeout_;

  // Subscriber
  ros::Subscriber sub_heartbeat_;
  ros::Time last_heartbeat_time_ = ros::Time(0);

  void onHeartbeat(const typename HeartbeatMsg::ConstPtr & msg)
  {
    last_heartbeat_time_ = ros::Time::now();
  }
};
