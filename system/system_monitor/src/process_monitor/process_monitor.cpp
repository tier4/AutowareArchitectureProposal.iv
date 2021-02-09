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
 * @file process_monitor.cpp
 * @brief Process monitor class
 */

#include <memory>
#include <regex>
#include <string>
#include <vector>

#include "fmt/format.h"

#include "system_monitor/process_monitor/process_monitor.hpp"

ProcessMonitor::ProcessMonitor(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options),
  updater_(this),
  num_of_procs_(declare_parameter<int>("num_of_procs", 5))
{
  int index;

  gethostname(hostname_, sizeof(hostname_));

  updater_.setHardwareID(hostname_);
  updater_.add("Tasks Summary", this, &ProcessMonitor::monitorProcesses);

  for (index = 0; index < num_of_procs_; ++index) {
    auto task = std::make_shared<DiagTask>(fmt::format("High-load Proc[{}]", index));
    load_tasks_.push_back(task);
    updater_.add(*task);
  }
  for (index = 0; index < num_of_procs_; ++index) {
    auto task = std::make_shared<DiagTask>(fmt::format("High-mem Proc[{}]", index));
    memory_tasks_.push_back(task);
    updater_.add(*task);
  }
}

void ProcessMonitor::update()
{
  updater_.force_update();
}

void ProcessMonitor::monitorProcesses(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  bp::ipstream is_err;
  bp::ipstream is_out;
  std::ostringstream os;

  // Get processes
  bp::child c("top -bn1 -o %CPU -w 128", bp::std_out > is_out, bp::std_err > is_err);
  c.wait();
  if (c.exit_code() != 0) {
    is_err >> os.rdbuf();
    stat.summary(DiagStatus::ERROR, "top error");
    stat.add("top", os.str().c_str());
    setErrorContent(&load_tasks_, "top error", "top", os.str().c_str());
    setErrorContent(&memory_tasks_, "top error", "top", os.str().c_str());
    return;
  }

  is_out >> os.rdbuf();
  std::string str = os.str();

  // Get task summary
  getTasksSummary(stat, str);
  // Remove header
  removeHeader(stat, str);

  // Get high load processes
  getHighLoadProcesses(str);

  // Get high memory processes
  getHighMemoryProcesses(str);
}

void ProcessMonitor::getTasksSummary(
  diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & output)
{
  bp::pipe p;
  std::string line;

  // Echo output for grep
  {
    bp::ipstream is_out;
    bp::ipstream is_err;
    bp::child c(fmt::format("echo {}", output), bp::std_out > p, bp::std_err > is_err);
    c.wait();
    if (c.exit_code() != 0) {
      std::ostringstream os;
      is_err >> os.rdbuf();
      stat.summary(DiagStatus::ERROR, "echo error");
      stat.add("echo", os.str().c_str());
      return;
    }
  }
  // Find matching pattern of summary
  {
    bp::ipstream is_out;
    bp::child c("grep Tasks:", bp::std_out > is_out, bp::std_in < p);
    c.wait();
    // no matching line
    if (c.exit_code() != 0) {
      stat.summary(DiagStatus::ERROR, "matching pattern not found");
      stat.add("name", "Tasks:");
      return;
    }

    std::getline(is_out, line);
    std::cmatch match;
    const std::regex filter(
      "^Tasks: (\\d+) total,\\s+(\\d+) running,\\s+(\\d+) sleeping,\\s+(\\d+) stopped,\\s+(\\d+) "
      "zombie");

    if (std::regex_match(line.c_str(), match, filter)) {
      stat.add("total", match[1].str());
      stat.add("running", match[2].str());
      stat.add("sleeping", match[3].str());
      stat.add("stopped", match[4].str());
      stat.add("zombie", match[5].str());
      stat.summary(DiagStatus::OK, "OK");
    } else {
      stat.summary(DiagStatus::ERROR, "invalid format");
    }
  }
}

void ProcessMonitor::removeHeader(
  diagnostic_updater::DiagnosticStatusWrapper & stat, std::string & output)
{
  bp::pipe p1;
  bp::pipe p2;
  std::ostringstream os;

  // Echo output for sed
  {
    bp::ipstream is_err;
    bp::child c(fmt::format("echo {}", output), bp::std_out > p1, bp::std_err > is_err);
    c.wait();
    if (c.exit_code() != 0) {
      is_err >> os.rdbuf();
      stat.summary(DiagStatus::ERROR, "echo error");
      stat.add("echo", os.str().c_str());
      return;
    }
  }
  // Remove %Cpu section
  {
    bp::ipstream is_err;
    bp::child c("sed \"/^%Cpu/d\"", bp::std_out > p2, bp::std_err > is_err, bp::std_in < p1);
    c.wait();
    // no matching line
    if (c.exit_code() != 0) {
      stat.summary(DiagStatus::ERROR, "sed error");
      stat.add("sed", "Failed to remove header");
      return;
    }
  }
  // Remove header
  {
    bp::ipstream is_out;
    bp::ipstream is_err;
    bp::child c("sed \"1,6d\"", bp::std_out > is_out, bp::std_err > is_err, bp::std_in < p2);
    c.wait();
    // no matching line
    if (c.exit_code() != 0) {
      stat.summary(DiagStatus::ERROR, "sed error");
      stat.add("sed", "Failed to remove header");
      return;
    }
    // overwrite
    is_out >> os.rdbuf();
    output = os.str();
  }
}

void ProcessMonitor::getHighLoadProcesses(const std::string & output)
{
  bp::pipe p;
  std::ostringstream os;

  // Echo output for sed
  bp::ipstream is_out;
  bp::ipstream is_err;
  bp::child c(fmt::format("echo {}", output), bp::std_out > p, bp::std_err > is_err);
  c.wait();
  if (c.exit_code() != 0) {
    is_err >> os.rdbuf();
    setErrorContent(&load_tasks_, "echo error", "echo", os.str().c_str());
    return;
  }

  // Get top-rated
  getTopratedProcesses(&load_tasks_, &p);
}

void ProcessMonitor::getHighMemoryProcesses(const std::string & output)
{
  bp::pipe p1;
  bp::pipe p2;
  std::ostringstream os;

  // Echo output for sed
  {
    bp::ipstream is_out;
    bp::ipstream is_err;
    bp::child c(fmt::format("echo {}", output), bp::std_out > p1, bp::std_err > is_err);
    c.wait();
    if (c.exit_code() != 0) {
      is_err >> os.rdbuf();
      setErrorContent(&memory_tasks_, "echo error", "echo", os.str().c_str());
      return;
    }
  }
  // Sort by memory usage
  {
    bp::ipstream is_out;
    bp::ipstream is_err;
    bp::child c("sort -r -k 10", bp::std_out > p2, bp::std_err > is_err, bp::std_in < p1);
    c.wait();
    if (c.exit_code() != 0) {
      is_err >> os.rdbuf();
      setErrorContent(&memory_tasks_, "sort error", "sort", os.str().c_str());
      return;
    }
  }

  // Get top-rated
  getTopratedProcesses(&memory_tasks_, &p2);
}

void ProcessMonitor::getTopratedProcesses(
  std::vector<std::shared_ptr<DiagTask>> * tasks, bp::pipe * p)
{
  if (tasks == nullptr || p == nullptr) {return;}

  bp::ipstream is_out;
  bp::ipstream is_err;
  std::ostringstream os;

  bp::child c(
    fmt::format("sed -n \"1,{} p\"", num_of_procs_), bp::std_out > is_out, bp::std_err > is_err,
    bp::std_in < *p);

  c.wait();
  // Failed to modify line
  if (c.exit_code() != 0) {
    is_err >> os.rdbuf();
    setErrorContent(tasks, "sed error", "sed", os.str().c_str());
    return;
  }

  std::vector<std::string> list;
  std::string line;
  int index = 0;

  while (std::getline(is_out, line) && !line.empty()) {
    boost::trim_left(line);
    boost::split(list, line, boost::is_space(), boost::token_compress_on);

    tasks->at(index)->setDiagnosticsStatus(DiagStatus::OK, "OK");
    tasks->at(index)->setProcessId(list[0]);
    tasks->at(index)->setUserName(list[1]);
    tasks->at(index)->setPriority(list[2]);
    tasks->at(index)->setNiceValue(list[3]);
    tasks->at(index)->setVirtualImage(list[4]);
    tasks->at(index)->setResidentSize(list[5]);
    tasks->at(index)->setSharedMemSize(list[6]);
    tasks->at(index)->setProcessStatus(list[7]);
    tasks->at(index)->setCPUUsage(list[8]);
    tasks->at(index)->setMemoryUsage(list[9]);
    tasks->at(index)->setCPUTime(list[10]);
    tasks->at(index)->setCommandName(list[11]);
    ++index;
  }
}

void ProcessMonitor::setErrorContent(
  std::vector<std::shared_ptr<DiagTask>> * tasks, const std::string & message,
  const std::string & error_command, const std::string & content)
{
  if (tasks == nullptr) {return;}

  for (auto itr = tasks->begin(); itr != tasks->end(); ++itr) {
    (*itr)->setDiagnosticsStatus(DiagStatus::ERROR, message);
    (*itr)->setErrorContent(error_command, content);
  }
}
