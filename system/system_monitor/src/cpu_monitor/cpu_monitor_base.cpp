// Copyright 2020 Autoware Foundation
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

/**
 * @file cpu_monitor_base.cpp
 * @brief CPU monitor base class
 */

#include <algorithm>
#include <regex>
#include <string>

#include "boost/filesystem.hpp"
#include "boost/process.hpp"
#include "boost/property_tree/json_parser.hpp"
#include "boost/property_tree/ptree.hpp"
#include "boost/thread.hpp"

#include "fmt/format.h"

#include "system_monitor/cpu_monitor/cpu_monitor_base.hpp"

namespace bp = boost::process;
namespace fs = boost::filesystem;
namespace pt = boost::property_tree;

CPUMonitorBase::CPUMonitorBase(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options),
  updater_(this),
  hostname_(),
  num_cores_(0),
  temps_(),
  freqs_(),
  mpstat_exists_(false),
  temp_warn_(declare_parameter<float>("temp_warn", 90.0)),
  temp_error_(declare_parameter<float>("temp_error", 95.0)),
  usage_warn_(declare_parameter<float>("usage_warn", 0.90)),
  usage_error_(declare_parameter<float>("usage_error", 1.00)),
  usage_avg_(declare_parameter<bool>("usage_avg", true)),
  load1_warn_(declare_parameter<float>("load1_warn", 0.90)),
  load5_warn_(declare_parameter<float>("load5_warn", 0.80))
{
  gethostname(hostname_, sizeof(hostname_));
  num_cores_ = boost::thread::hardware_concurrency();

  // Check if command exists
  fs::path p = bp::search_path("mpstat");
  mpstat_exists_ = (p.empty()) ? false : true;

  updater_.setHardwareID(hostname_);
  updater_.add("CPU Temperature", this, &CPUMonitorBase::checkTemp);
  updater_.add("CPU Usage", this, &CPUMonitorBase::checkUsage);
  updater_.add("CPU Load Average", this, &CPUMonitorBase::checkLoad);
  updater_.add("CPU Thermal Throttling", this, &CPUMonitorBase::checkThrottling);
  updater_.add("CPU Frequency", this, &CPUMonitorBase::checkFrequency);
}

void CPUMonitorBase::update()
{
  updater_.force_update();
}

void CPUMonitorBase::checkTemp(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (temps_.empty()) {
    stat.summary(DiagStatus::ERROR, "temperature files not found");
    return;
  }

  int level = DiagStatus::OK;
  std::string error_str = "";

  for (auto itr = temps_.begin(); itr != temps_.end(); ++itr) {
    // Read temperature file
    const fs::path path(itr->path_);
    fs::ifstream ifs(path, std::ios::in);
    if (!ifs) {
      stat.add("file open error", itr->path_);
      error_str = "file open error";
      continue;
    }

    float temp;
    ifs >> temp;
    ifs.close();
    temp /= 1000;
    stat.addf(itr->label_, "%.1f DegC", temp);

    if (temp >= temp_error_) {
      level = std::max(level, static_cast<int>(DiagStatus::ERROR));
    } else if (temp >= temp_warn_) {
      level = std::max(level, static_cast<int>(DiagStatus::WARN));
    }
  }

  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, error_str);
  } else {
    stat.summary(level, temp_dict_.at(level));
  }
}

void CPUMonitorBase::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!mpstat_exists_) {
    stat.summary(DiagStatus::ERROR, "mpstat error");
    stat.add(
      "mpstat", "Command 'mpstat' not found, but can be installed with: sudo apt install sysstat");
    return;
  }

  // Get CPU Usage
  bp::ipstream is_out;
  bp::ipstream is_err;
  bp::child c("mpstat -P ALL 1 1 -o JSON", bp::std_out > is_out, bp::std_err > is_err);
  c.wait();

  if (c.exit_code() != 0) {
    std::ostringstream os;
    is_err >> os.rdbuf();
    stat.summary(DiagStatus::ERROR, "mpstat error");
    stat.add("mpstat", os.str().c_str());
    return;
  }

  std::string cpu_name;
  float usr;
  float nice;
  float sys;
  float idle;
  float usage;
  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;

  pt::ptree pt;
  try {
    // Analyze JSON output
    read_json(is_out, pt);

    for (const pt::ptree::value_type & child1 : pt.get_child("sysstat.hosts")) {
      const pt::ptree & hosts = child1.second;

      for (const pt::ptree::value_type & child2 : hosts.get_child("statistics")) {
        const pt::ptree & statistics = child2.second;

        for (const pt::ptree::value_type & child3 : statistics.get_child("cpu-load")) {
          const pt::ptree & cpu_load = child3.second;

          if (boost::optional<std::string> v = cpu_load.get_optional<std::string>("cpu")) {
            cpu_name = v.get();
          }
          if (boost::optional<float> v = cpu_load.get_optional<float>("usr")) {usr = v.get();}
          if (boost::optional<float> v = cpu_load.get_optional<float>("nice")) {nice = v.get();}
          if (boost::optional<float> v = cpu_load.get_optional<float>("sys")) {sys = v.get();}
          if (boost::optional<float> v = cpu_load.get_optional<float>("idle")) {idle = v.get();}

          usage = (usr + nice) * 1e-2;

          level = DiagStatus::OK;
          if (usage >= usage_error_) {
            level = DiagStatus::ERROR;
          } else if (usage >= usage_warn_) {
            level = DiagStatus::WARN;
          }

          stat.add(fmt::format("CPU {}: status", cpu_name), load_dict_.at(level));
          stat.addf(fmt::format("CPU {}: usr", cpu_name), "%.2f%%", usr);
          stat.addf(fmt::format("CPU {}: nice", cpu_name), "%.2f%%", nice);
          stat.addf(fmt::format("CPU {}: sys", cpu_name), "%.2f%%", sys);
          stat.addf(fmt::format("CPU {}: idle", cpu_name), "%.2f%%", idle);

          if (usage_avg_ == true) {
            if (cpu_name == "all") {whole_level = level;}
          } else {
            whole_level = std::max(whole_level, level);
          }
        }
      }
    }
  } catch (const std::exception & e) {
    stat.summary(DiagStatus::ERROR, "mpstat exception");
    stat.add("mpstat", e.what());
    return;
  }

  stat.summary(whole_level, load_dict_.at(whole_level));
}

void CPUMonitorBase::checkLoad(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  double loadavg[3];

  std::ifstream ifs("/proc/loadavg", std::ios::in);

  if (!ifs) {
    stat.summary(DiagStatus::ERROR, "uptime error");
    stat.add("uptime", strerror(errno));
    return;
  }

  std::string line;

  if (!std::getline(ifs, line)) {
    stat.summary(DiagStatus::ERROR, "uptime error");
    stat.add("uptime", "format error");
    return;
  }

  if (sscanf(line.c_str(), "%lf %lf %lf", &loadavg[0], &loadavg[1], &loadavg[2]) != 3) {
    stat.summary(DiagStatus::ERROR, "uptime error");
    stat.add("uptime", "format error");
    return;
  }

  loadavg[0] /= num_cores_;
  loadavg[1] /= num_cores_;
  loadavg[2] /= num_cores_;

  int level = DiagStatus::OK;
  if (loadavg[0] > load1_warn_ || loadavg[1] > load5_warn_) {level = DiagStatus::WARN;}

  stat.summary(level, load_dict_.at(level));
  stat.addf("1min", "%.2f%%", loadavg[0] * 1e2);
  stat.addf("5min", "%.2f%%", loadavg[1] * 1e2);
  stat.addf("15min", "%.2f%%", loadavg[2] * 1e2);
}

void CPUMonitorBase::checkThrottling(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  RCLCPP_INFO(this->get_logger(), "CPUMonitorBase::checkThrottling not implemented.");
}

void CPUMonitorBase::checkFrequency(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (freqs_.empty()) {
    stat.summary(DiagStatus::ERROR, "frequency files not found");
    return;
  }

  for (auto itr = freqs_.begin(); itr != freqs_.end(); ++itr) {
    // Read scaling_cur_freq file
    const fs::path path(itr->path_);
    fs::ifstream ifs(path, std::ios::in);
    if (ifs) {
      std::string line;
      if (std::getline(ifs, line)) {
        stat.addf(
          fmt::format("CPU {}: clock", itr->index_), "%d MHz", std::stoi(line) / 1000);
      }
    }
    ifs.close();
  }

  stat.summary(DiagStatus::OK, "OK");
}

void CPUMonitorBase::getTempNames(void)
{
  RCLCPP_INFO(this->get_logger(), "CPUMonitorBase::getTempNames not implemented.");
}

void CPUMonitorBase::getFreqNames(void)
{
  const fs::path root("/sys/devices/system/cpu");

  for (const fs::path & path :
    boost::make_iterator_range(fs::directory_iterator(root), fs::directory_iterator()))
  {
    if (!fs::is_directory(path)) {continue;}

    std::cmatch match;
    const char * cpu_dir = path.generic_string().c_str();

    // /sys/devices/system/cpu[0-9] ?
    if (!std::regex_match(cpu_dir, match, std::regex(".*cpu(\\d+)"))) {continue;}

    // /sys/devices/system/cpu[0-9]/cpufreq/scaling_cur_freq
    cpu_freq_info freq;
    const fs::path freq_path = path / "cpufreq/scaling_cur_freq";
    freq.index_ = std::stoi(match[1].str());
    freq.path_ = freq_path.generic_string();
    freqs_.push_back(freq);
  }

  std::sort(
    freqs_.begin(), freqs_.end(), [](const cpu_freq_info & c1, const cpu_freq_info & c2) {
      return c1.index_ < c2.index_;
    }); // NOLINT
}
