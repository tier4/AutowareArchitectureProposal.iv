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

#include "topic_state_monitor/topic_state_monitor_node.h"

namespace topic_state_monitor
{
TopicStateMonitorNode::TopicStateMonitorNode()
{
  // Parameter
  private_nh_.param("update_rate", node_param_.update_rate, 10.0);

  if (!private_nh_.getParam("topic", param_.topic)) {
    throw std::runtime_error("A ROS parameter `topic` was not given.");
  }
  if (!private_nh_.getParam("diag_name", param_.diag_name)) {
    throw std::runtime_error("A ROS parameter `diag_name` was not given.");
  }
  private_nh_.param("warn_rate", param_.warn_rate, 0.5);
  private_nh_.param("error_rate", param_.error_rate, 0.1);
  private_nh_.param("timeout", param_.timeout, 1.0);
  private_nh_.param("window_size", param_.window_size, 10);

  // Dynamic Reconfigure
  dynamic_reconfigure_.setCallback(boost::bind(&TopicStateMonitorNode::onConfig, this, _1, _2));

  // Core
  topic_state_monitor_ = std::make_unique<TopicStateMonitor>();
  topic_state_monitor_->setParam(param_);

  // Subscriber
  sub_topic_ = nh_.subscribe(param_.topic, 10, &TopicStateMonitorNode::onTopic, this);

  // Diagnostic Updater
  updater_.setHardwareID("topic_state_monitor");
  updater_.add(param_.diag_name, this, &TopicStateMonitorNode::checkTopicStatus);

  // Timer
  timer_ = private_nh_.createTimer(
    ros::Rate(node_param_.update_rate), &TopicStateMonitorNode::onTimer, this);
}

void TopicStateMonitorNode::onConfig(const TopicStateMonitorConfig & config, const uint32_t level)
{
  param_.warn_rate = config.warn_rate;
  param_.error_rate = config.error_rate;
  param_.timeout = config.timeout;
  param_.window_size = config.window_size;
}

void TopicStateMonitorNode::onTopic(const topic_tools::ShapeShifter::ConstPtr & msg)
{
  topic_state_monitor_->update();
}

void TopicStateMonitorNode::onTimer(const ros::TimerEvent & event)
{
  // Publish diagnostics
  updater_.force_update();
}

void TopicStateMonitorNode::checkTopicStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::DiagnosticStatus;

  // Get information
  const auto topic_status = topic_state_monitor_->getTopicStatus();
  const auto last_message_time = topic_state_monitor_->getLastMessageTime();
  const auto topic_rate = topic_state_monitor_->getTopicRate();

  // Add topic name
  stat.addf("topic", "%s", param_.topic.c_str());

  // Judge level
  int8_t level = DiagnosticStatus::OK;
  if (topic_status == TopicStatus::Ok) {
    level = DiagnosticStatus::OK;
    stat.add("status", "OK");
  } else if (topic_status == TopicStatus::NotReceived) {
    level = DiagnosticStatus::ERROR;
    stat.add("status", "NotReceived");
  } else if (topic_status == TopicStatus::WarnRate) {
    level = DiagnosticStatus::WARN;
    stat.add("status", "WarnRate");
  } else if (topic_status == TopicStatus::ErrorRate) {
    level = DiagnosticStatus::ERROR;
    stat.add("status", "ErrorRate");
  } else if (topic_status == TopicStatus::Timeout) {
    level = DiagnosticStatus::ERROR;
    stat.add("status", "Timeout");
  }

  // Add key-value
  stat.addf("warn_rate", "%.2f [Hz]", param_.warn_rate);
  stat.addf("error_rate", "%.2f [Hz]", param_.error_rate);
  stat.addf("timeout", "%.2f [s]", param_.timeout);
  stat.addf("measured_rate", "%.2f [Hz]", topic_rate);
  stat.addf("now", "%.2f [s]", ros::Time::now().toSec());
  stat.addf("last_message_time", "%.2f [s]", last_message_time.toSec());

  // Create message
  std::string msg;
  if (level == diagnostic_msgs::DiagnosticStatus::OK) {
    msg = "OK";
  } else if (level == diagnostic_msgs::DiagnosticStatus::WARN) {
    msg = "Warn";
  } else if (level == diagnostic_msgs::DiagnosticStatus::ERROR) {
    msg = "Error";
  }

  // Add summary
  stat.summary(level, msg);
}

}  // namespace topic_state_monitor
