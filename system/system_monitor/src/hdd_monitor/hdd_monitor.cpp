/*
 * Copyright 2020 Autoware Foundation. All rights reserved.
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

/**
 * @file hdd_monitor.cpp
 * @brief HDD monitor class
 */

#include <algorithm>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/process.hpp>
#include <boost/serialization/vector.hpp>

#include <fmt/format.h>

#include <hdd_reader/hdd_reader.h>
#include <system_monitor/hdd_monitor/hdd_monitor.h>

namespace bp = boost::process;

HDDMonitor::HDDMonitor(const ros::NodeHandle & nh, const ros::NodeHandle & pnh) : nh_(nh), pnh_(pnh)
{
  gethostname(hostname_, sizeof(hostname_));

  getHDDParams();
  pnh_.param<int>("hdd_reader_port", hdd_reader_port_, 7635);

  updater_.setHardwareID(hostname_);
  updater_.add("HDD Temperature", this, &HDDMonitor::checkTemp);
  updater_.add("HDD Usage", this, &HDDMonitor::checkUsage);
}

void HDDMonitor::run()
{
  ros::Rate rate(1.0);

  while (ros::ok()) {
    ros::spinOnce();
    updater_.force_update();
    rate.sleep();
  }
}

void HDDMonitor::checkTemp(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (hdd_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  // Create a new socket
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    stat.summary(DiagStatus::ERROR, "socket error");
    stat.add("socket", strerror(errno));
    return;
  }

  // Specify the receiving timeouts until reporting an error
  struct timeval tv;
  tv.tv_sec = 10;
  tv.tv_usec = 0;
  int ret = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "setsockopt error");
    stat.add("setsockopt", strerror(errno));
    close(sock);
    return;
  }

  // Connect the socket referred to by the file descriptor
  sockaddr_in addr;
  memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(hdd_reader_port_);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  ret = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "connect error");
    stat.add("connect", strerror(errno));
    close(sock);
    return;
  }

  std::ostringstream oss;
  boost::archive::text_oarchive oa(oss);
  oa & hdd_devices_;

  // Write list of devices to FD
  ret = write(sock, oss.str().c_str(), oss.str().length());
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "write error");
    stat.add("write", strerror(errno));
    ROS_ERROR("write error");
    close(sock);
    return;
  }

  // Receive messages from a socket
  char buf[1024] = "";
  ret = recv(sock, buf, sizeof(buf) - 1, 0);
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "recv error");
    stat.add("recv", strerror(errno));
    close(sock);
    return;
  }
  // No data received
  if (ret == 0) {
    stat.summary(DiagStatus::ERROR, "recv error");
    stat.add("recv", "No data received");
    close(sock);
    return;
  }

  // Close the file descriptor FD
  ret = close(sock);
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "close error");
    stat.add("close", strerror(errno));
    return;
  }

  // Restore HDD information list
  HDDInfoList list;

  try {
    std::istringstream iss(buf);
    boost::archive::text_iarchive oa(iss);
    oa >> list;
  } catch (const std::exception & e) {
    stat.summary(DiagStatus::ERROR, "recv error");
    stat.add("recv", e.what());
    return;
  }

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;
  std::string error_str = "";

  for (auto itr = hdd_params_.begin(); itr != hdd_params_.end(); ++itr, ++index) {
    // Retrieve HDD information
    auto itrh = list.find(itr->first);
    if (itrh == list.end()) {
      stat.add(fmt::format("HDD {}: status", index), "hdd_reader error");
      stat.add(fmt::format("HDD {}: name", index), itr->first.c_str());
      stat.add(fmt::format("HDD {}: hdd_reader", index), strerror(ENOENT));
      error_str = "hdd_reader error";
      continue;
    }

    if (itrh->second.error_code_ != 0) {
      stat.add(fmt::format("HDD {}: status", index), "hdd_reader error");
      stat.add(fmt::format("HDD {}: name", index), itr->first.c_str());
      stat.add(fmt::format("HDD {}: hdd_reader", index), strerror(itrh->second.error_code_));
      error_str = "hdd_reader error";
      continue;
    }

    float temp = static_cast<float>(itrh->second.temp_);

    level = DiagStatus::OK;
    if (temp >= itr->second.temp_error_)
      level = DiagStatus::ERROR;
    else if (temp >= itr->second.temp_warn_)
      level = DiagStatus::WARN;

    stat.add(fmt::format("HDD {}: status", index), temp_dict_.at(level));
    stat.add(fmt::format("HDD {}: name", index), itr->first.c_str());
    stat.add(fmt::format("HDD {}: model", index), itrh->second.model_.c_str());
    stat.add(fmt::format("HDD {}: serial", index), itrh->second.serial_.c_str());
    stat.addf(fmt::format("HDD {}: temperature", index), "%.1f DegC", temp);

    whole_level = std::max(whole_level, level);
  }

  if (!error_str.empty())
    stat.summary(DiagStatus::ERROR, error_str);
  else
    stat.summary(whole_level, temp_dict_.at(whole_level));
}

void HDDMonitor::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (hdd_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  int hdd_index = 0;
  int whole_level = DiagStatus::OK;
  std::string error_str = "";

  for (auto itr = hdd_params_.begin(); itr != hdd_params_.end(); ++itr, ++hdd_index) {
    // Get summary of disk space usage of ext4
    bp::ipstream is_out;
    bp::ipstream is_err;
    // Invoke shell to use shell wildcard expansion
    bp::child c(
      "/bin/sh", "-c", fmt::format("df -Pht ext4 {}*", itr->first.c_str()), bp::std_out > is_out,
      bp::std_err > is_err);
    c.wait();

    if (c.exit_code() != 0) {
      std::ostringstream os;
      is_err >> os.rdbuf();
      error_str = "df error";
      stat.add(fmt::format("HDD {}: status", hdd_index), "df error");
      stat.add(fmt::format("HDD {}: name", hdd_index), itr->first.c_str());
      stat.add(fmt::format("HDD {}: df", hdd_index), os.str().c_str());
      continue;
    }

    int level = DiagStatus::OK;
    std::string line;
    int index = 0;
    std::vector<std::string> list;
    float usage;

    while (std::getline(is_out, line) && !line.empty()) {
      // Skip header
      if (index <= 0) {
        ++index;
        continue;
      }

      boost::split(list, line, boost::is_space(), boost::token_compress_on);

      usage = std::atof(boost::trim_copy_if(list[4], boost::is_any_of("%")).c_str()) * 1e-2;

      level = DiagStatus::OK;
      if (usage >= itr->second.usage_error_)
        level = DiagStatus::ERROR;
      else if (usage >= itr->second.usage_warn_)
        level = DiagStatus::WARN;

      stat.add(fmt::format("HDD {}: status", hdd_index), usage_dict_.at(level));
      stat.add(fmt::format("HDD {}: filesystem", hdd_index), list[0].c_str());
      stat.add(fmt::format("HDD {}: size", hdd_index), list[1].c_str());
      stat.add(fmt::format("HDD {}: used", hdd_index), list[2].c_str());
      stat.add(fmt::format("HDD {}: avail", hdd_index), list[3].c_str());
      stat.add(fmt::format("HDD {}: use", hdd_index), list[4].c_str());
      stat.add(fmt::format("HDD {}: mounted on", hdd_index), list[5].c_str());

      whole_level = std::max(whole_level, level);
      ++index;
    }
  }

  if (!error_str.empty())
    stat.summary(DiagStatus::ERROR, error_str);
  else
    stat.summary(whole_level, usage_dict_.at(whole_level));
}

void HDDMonitor::getHDDParams()
{
  XmlRpc::XmlRpcValue params;

  pnh_.getParam("disks", params);
  if (params.getType() != XmlRpc::XmlRpcValue::TypeArray) return;

  for (int i = 0; i < params.size(); ++i) {
    std::string name;
    HDDParam param;

    // Skip no name
    if (!params[i]["name"].valid()) continue;

    if (params[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString)
      name = static_cast<std::string>(params[i]["name"]);

    if (params[i]["temp_warn"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      param.temp_warn_ = static_cast<double>(params[i]["temp_warn"]);

    if (params[i]["temp_error"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      param.temp_error_ = static_cast<double>(params[i]["temp_error"]);

    if (params[i]["usage_warn"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      param.usage_warn_ = static_cast<double>(params[i]["usage_warn"]);

    if (params[i]["usage_error"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      param.usage_error_ = static_cast<double>(params[i]["usage_error"]);

    hdd_params_[name] = param;

    hdd_devices_.push_back(name);
  }
}
