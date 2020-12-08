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
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/optional.hpp>

#include <ros/ros.h>

#include <autoware_system_msgs/DrivingCapability.h>
#include <diagnostic_msgs/DiagnosticArray.h>

struct DiagStamped
{
  std_msgs::Header header;
  diagnostic_msgs::DiagnosticStatus status;
};

using DiagBuffer = std::deque<DiagStamped>;

struct DiagConfig
{
  explicit DiagConfig(XmlRpc::XmlRpcValue value)
  : name(static_cast<std::string>(value["name"])),
    sf_at(static_cast<std::string>(value["sf_at"])),
    lf_at(static_cast<std::string>(value["lf_at"])),
    spf_at(static_cast<std::string>(value["spf_at"]))
  {
    // Set default values
    if (sf_at == "") sf_at = "none";
    if (lf_at == "") lf_at = "warn";
    if (spf_at == "") spf_at = "error";
  }

  std::string name;
  std::string sf_at;
  std::string lf_at;
  std::string spf_at;
};

using RequiredModules = std::vector<DiagConfig>;

struct KeyName
{
  static constexpr const char * autonomous_driving = "autonomous_driving";
  static constexpr const char * remote_control = "remote_control";
};

class AutowareErrorMonitorNode
{
public:
  AutowareErrorMonitorNode();

private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  // Parameter
  double update_rate_;
  bool ignore_missing_diagnostics_;
  bool add_leaf_diagnostics_;
  std::unordered_map<std::string, RequiredModules> required_modules_map_;

  void loadRequiredModules(const std::string & key);

  // Timer
  ros::Timer timer_;

  bool isDataReady();
  void onTimer(const ros::TimerEvent & event);

  // Subscriber
  ros::Subscriber sub_diag_array_;

  void onDiagArray(const diagnostic_msgs::DiagnosticArray::ConstPtr & msg);

  const size_t diag_buffer_size_ = 100;
  std::unordered_map<std::string, DiagBuffer> diag_buffer_map_;
  diagnostic_msgs::DiagnosticArray::ConstPtr diag_array_;

  // Publisher
  ros::Publisher pub_driving_capability_;

  // Algorithm
  boost::optional<DiagStamped> getLatestDiag(const std::string & diag_name);
  int getHazardLevel(const DiagConfig & required_module, const int diag_level);
  void appendHazardDiag(
    const DiagConfig & required_module, const diagnostic_msgs::DiagnosticStatus & diag,
    autoware_system_msgs::HazardStatus * hazard_status);
  autoware_system_msgs::HazardStatus judgeHazardStatus(const std::string & key);

  const double diag_timeout_sec_ = 1.0;
};
