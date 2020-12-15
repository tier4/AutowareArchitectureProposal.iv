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

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <topic_tools/shape_shifter.h>

#include "topic_state_monitor/TopicStateMonitorConfig.h"
#include "topic_state_monitor/topic_state_monitor.h"

namespace topic_state_monitor
{
struct NodeParam
{
  double update_rate;
};

class TopicStateMonitorNode
{
public:
  TopicStateMonitorNode();

private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  // Parameter
  NodeParam node_param_;
  Param param_;

  // Dynamic Reconfigure
  void onConfig(const TopicStateMonitorConfig & config, const uint32_t level);
  dynamic_reconfigure::Server<TopicStateMonitorConfig> dynamic_reconfigure_;

  // Core
  std::unique_ptr<TopicStateMonitor> topic_state_monitor_;

  // Subscriber
  ros::Subscriber sub_topic_;

  void onTopic(const topic_tools::ShapeShifter::ConstPtr & msg);

  // Timer
  void onTimer(const ros::TimerEvent & event);
  ros::Timer timer_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;

  void checkTopicStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);
};
}  // namespace topic_state_monitor
