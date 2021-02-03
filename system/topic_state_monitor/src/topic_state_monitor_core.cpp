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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "topic_state_monitor/topic_state_monitor_core.hpp"

namespace
{
template<typename T>
void update_param(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & name, T & value)
{
  auto it = std::find_if(
    parameters.cbegin(), parameters.cend(),
    [&name](const rclcpp::Parameter & parameter) {return parameter.get_name() == name;});
  if (it != parameters.cend()) {
    value = it->template get_value<T>();
  }
}
}  // namespace

namespace topic_state_monitor
{
TopicStateMonitorNode::TopicStateMonitorNode()
: Node("topic_state_monitor"),
  updater_(this)
{
  using std::placeholders::_1;
  // Parameter
  node_param_.update_rate = declare_parameter("update_rate", 10.0);
  param_.topic = declare_parameter("topic").get<std::string>();
  param_.topic_type = declare_parameter("topic_type").get<std::string>();
  param_.diag_name = declare_parameter("diag_name").get<std::string>();
  param_.warn_rate = declare_parameter("warn_rate", 0.5);
  param_.error_rate = declare_parameter("error_rate", 0.1);
  param_.timeout = declare_parameter("timeout", 1.0);
  param_.window_size = declare_parameter("window_size", 10);

  // Parameter Reconfigure
  set_param_res_ =
    this->add_on_set_parameters_callback(std::bind(&TopicStateMonitorNode::onParameter, this, _1));


  // Core
  topic_state_monitor_ = std::make_unique<TopicStateMonitor>(*this);
  topic_state_monitor_->setParam(param_);

  // Subscriber
  sub_topic_ = rclcpp_generic::GenericSubscription::create(
    get_node_topics_interface(), param_.topic, param_.topic_type, 1,
    [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      topic_state_monitor_->update();
    });

  // Diagnostic Updater
  updater_.setHardwareID("topic_state_monitor");
  updater_.add(param_.diag_name, this, &TopicStateMonitorNode::checkTopicStatus);

  // Timer
  auto timer_callback = std::bind(&TopicStateMonitorNode::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / node_param_.update_rate));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

rcl_interfaces::msg::SetParametersResult TopicStateMonitorNode::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    update_param(parameters, "warn_rate", param_.warn_rate);
    update_param(parameters, "error_rate", param_.error_rate);
    update_param(parameters, "timeout", param_.timeout);
    update_param(parameters, "window_size", param_.window_size);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

void TopicStateMonitorNode::onTimer()
{
  // Publish diagnostics
  updater_.force_update();
}

void TopicStateMonitorNode::checkTopicStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

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
  stat.addf("now", "%.2f [s]", this->now().seconds());
  stat.addf("last_message_time", "%.2f [s]", last_message_time.seconds());

  // Create message
  std::string msg;
  if (level == DiagnosticStatus::OK) {
    msg = "OK";
  } else if (level == DiagnosticStatus::WARN) {
    msg = "Warn";
  } else if (level == DiagnosticStatus::ERROR) {
    msg = "Error";
  }

  // Add summary
  stat.summary(level, msg);
}

}  // namespace topic_state_monitor
