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

#ifndef TRAJECTORY_LOADER__NODE_HPP_
#define TRAJECTORY_LOADER__NODE_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory>

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"


class TrajectoryLoaderNode : public rclcpp::Node
{
private:
  using csv = std::vector<std::vector<std::string>>;
  using association = std::map<std::string /* label */, int /* row */>;

  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_;

  bool publish(
    const std_msgs::msg::Header & header, const association & label_row_association_map,
    const csv & file_data);
  bool loadData(
    const std::string & file_name, association & label_row_association_map, csv & file_data);
  std::vector<std::string> split(const std::string & input, char delimiter);
  void deleteHeadSpace(std::string & string);
  void deleteUnit(std::string & string);

public:
  TrajectoryLoaderNode();
  ~TrajectoryLoaderNode() {}
};

#endif  // TRAJECTORY_LOADER__NODE_HPP_
