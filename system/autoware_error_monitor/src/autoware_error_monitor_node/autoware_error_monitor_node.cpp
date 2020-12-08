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

#include <autoware_error_monitor/autoware_error_monitor_node.h>

#include <regex>

#include <fmt/format.h>

#include <autoware_error_monitor/diagnostics_filter.h>

namespace
{
int str2level(const std::string & level_str)
{
  using diagnostic_msgs::DiagnosticStatus;
  using std::regex_constants::icase;

  if (std::regex_match(level_str, std::regex("warn", icase))) return DiagnosticStatus::WARN;
  if (std::regex_match(level_str, std::regex("error", icase))) return DiagnosticStatus::ERROR;
  if (std::regex_match(level_str, std::regex("stale", icase))) return DiagnosticStatus::STALE;

  throw std::runtime_error(fmt::format("invalid level: {}", level_str));
};

bool isOverLevel(const int & diag_level, const std::string & failure_level_str)
{
  if (failure_level_str == "none") {
    return false;
  }

  return diag_level >= str2level(failure_level_str);
};

std::vector<diagnostic_msgs::DiagnosticStatus> & getTargetDiagnosticsRef(
  const int hazard_level, autoware_system_msgs::HazardStatus * hazard_status)
{
  using autoware_system_msgs::HazardStatus;

  if (hazard_level == HazardStatus::NO_FAULT) return hazard_status->diagnostics_nf;
  if (hazard_level == HazardStatus::SAFE_FAULT) return hazard_status->diagnostics_sf;
  if (hazard_level == HazardStatus::LATENT_FAULT) return hazard_status->diagnostics_lf;
  if (hazard_level == HazardStatus::SINGLE_POINT_FAULT) return hazard_status->diagnostics_spf;

  throw std::runtime_error(fmt::format("invalid hazard level: {}", hazard_level));
}
}  // namespace

AutowareErrorMonitorNode::AutowareErrorMonitorNode()
{
  // Parameter
  private_nh_.param("update_rate", update_rate_, 10.0);
  private_nh_.param("ignore_missing_diagnostics", ignore_missing_diagnostics_, false);
  private_nh_.param("add_leaf_diagnostics", add_leaf_diagnostics_, true);
  loadRequiredModules(KeyName::autonomous_driving);
  loadRequiredModules(KeyName::remote_control);

  // Subscriber
  sub_diag_array_ =
    private_nh_.subscribe("input/diag_array", 1, &AutowareErrorMonitorNode::onDiagArray, this);

  // Publisher
  pub_driving_capability_ =
    private_nh_.advertise<autoware_system_msgs::DrivingCapability>("output/driving_capability", 1);

  // Timer
  timer_ =
    private_nh_.createTimer(ros::Rate(update_rate_), &AutowareErrorMonitorNode::onTimer, this);
}

void AutowareErrorMonitorNode::loadRequiredModules(const std::string & key)
{
  const auto param_key = std::string("required_modules/") + key;

  XmlRpc::XmlRpcValue xml;
  if (!private_nh_.getParam(param_key, xml)) {
    throw std::runtime_error(fmt::format("no parameter found: {}", param_key));
  }

  RequiredModules required_modules;
  required_modules.reserve(xml.size());

  for (size_t i = 0; i < xml.size(); ++i) {
    required_modules.emplace_back(xml[i]);
  }

  required_modules_map_.insert(std::make_pair(key, required_modules));
}

void AutowareErrorMonitorNode::onDiagArray(const diagnostic_msgs::DiagnosticArray::ConstPtr & msg)
{
  diag_array_ = msg;

  const auto & header = msg->header;

  for (const auto & diag : msg->status) {
    if (diag_buffer_map_.count(diag.name) == 0) {
      diag_buffer_map_.insert(std::make_pair(diag.name, DiagBuffer{}));
    }

    auto & diag_buffer = diag_buffer_map_.at(diag.name);
    diag_buffer.push_back(DiagStamped{header, diag});

    while (diag_buffer.size() > diag_buffer_size_) {
      diag_buffer.pop_front();
    }
  }
}

bool AutowareErrorMonitorNode::isDataReady()
{
  if (!diag_array_) {
    ROS_INFO_THROTTLE(5.0, "waiting for diag_array msg...");
    return false;
  }

  return true;
}
void AutowareErrorMonitorNode::onTimer(const ros::TimerEvent & event)
{
  if (!isDataReady()) {
    return;
  }

  autoware_system_msgs::DrivingCapability driving_capability;

  driving_capability.autonomous_driving = judgeHazardStatus(KeyName::autonomous_driving);
  driving_capability.remote_control = judgeHazardStatus(KeyName::remote_control);

  pub_driving_capability_.publish(driving_capability);
}

boost::optional<DiagStamped> AutowareErrorMonitorNode::getLatestDiag(const std::string & diag_name)
{
  if (diag_buffer_map_.count(diag_name) == 0) {
    return {};
  }

  const auto & diag_buffer = diag_buffer_map_.at(diag_name);

  if (diag_buffer.empty()) {
    return {};
  }

  return diag_buffer.back();
}

int AutowareErrorMonitorNode::getHazardLevel(
  const DiagConfig & required_module, const int diag_level)
{
  using autoware_system_msgs::HazardStatus;

  if (isOverLevel(diag_level, required_module.spf_at)) return HazardStatus::SINGLE_POINT_FAULT;
  if (isOverLevel(diag_level, required_module.lf_at)) return HazardStatus::LATENT_FAULT;
  if (isOverLevel(diag_level, required_module.sf_at)) return HazardStatus::SAFE_FAULT;

  return HazardStatus::NO_FAULT;
}

void AutowareErrorMonitorNode::appendHazardDiag(
  const DiagConfig & required_module, const diagnostic_msgs::DiagnosticStatus & hazard_diag,
  autoware_system_msgs::HazardStatus * hazard_status)
{
  const auto hazard_level = getHazardLevel(required_module, hazard_diag.level);

  auto & target_diagnostics_ref = getTargetDiagnosticsRef(hazard_level, hazard_status);
  target_diagnostics_ref.push_back(hazard_diag);

  if (add_leaf_diagnostics_) {
    for (const auto & diag :
         diagnostics_filter::extractLeafChildrenDiagnostics(hazard_diag, diag_array_->status)) {
      target_diagnostics_ref.push_back(diag);
    }
  }

  hazard_status->level = std::max(hazard_status->level, hazard_level);
}

autoware_system_msgs::HazardStatus AutowareErrorMonitorNode::judgeHazardStatus(
  const std::string & key)
{
  using autoware_system_msgs::HazardStatus;
  using diagnostic_msgs::DiagnosticStatus;

  autoware_system_msgs::HazardStatus hazard_status;

  for (const auto & required_module : required_modules_map_.at(key)) {
    const auto & diag_name = required_module.name;

    const auto latest_diag = getLatestDiag(diag_name);

    // no diag found
    if (!latest_diag) {
      if (!ignore_missing_diagnostics_) {
        DiagnosticStatus missing_diag;

        missing_diag.name = diag_name;
        missing_diag.hardware_id = "autoware_error_monitor";
        missing_diag.level = DiagnosticStatus::STALE;
        missing_diag.message = "no diag found";

        appendHazardDiag(required_module, missing_diag, &hazard_status);
      }

      continue;
    }

    // diag level high
    {
      appendHazardDiag(required_module, latest_diag->status, &hazard_status);
    }

    // diag timeout
    {
      const auto time_diff = ros::Time::now() - latest_diag->header.stamp;
      if (time_diff.toSec() > diag_timeout_sec_) {
        DiagnosticStatus timeout_diag = latest_diag->status;
        timeout_diag.level = DiagnosticStatus::STALE;
        timeout_diag.message = "timeout";

        appendHazardDiag(required_module, timeout_diag, &hazard_status);
      }
    }
  }

  return hazard_status;
}
