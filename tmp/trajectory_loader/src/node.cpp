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

#include <fstream>
#include <vector>
#include <string>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "trajectory_loader/node.hpp"

TrajectoryLoaderNode::TrajectoryLoaderNode()
: Node("trajectory_loader_node")
{
  pub_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>("trajectory", 1);

  // param
  std::string file_name, frame_id;

  file_name = this->declare_parameter("trajectory_file_name", std::string("trajectory_file_name"));
  frame_id = this->declare_parameter("frame_id", std::string("map"));

  csv file_data;
  association label_row_association_map;

  // load csv file
  rclcpp::Rate loop_rate(1);
  while (!loadData(file_name, label_row_association_map, file_data) && rclcpp::ok()) {
    loop_rate.sleep();
  }

  // publish
  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  header.stamp = this->now();
  publish(header, label_row_association_map, file_data);
}

bool TrajectoryLoaderNode::publish(
  const std_msgs::msg::Header & header, const association & label_row_association_map,
  const csv & file_data)
{
  autoware_planning_msgs::msg::Trajectory msg;
  msg.header = header;
  for (size_t i = 0; i < file_data.size(); ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    double roll, pitch, yaw;

    for (size_t j = 0; j < file_data.at(i).size(); ++j) {
      if (label_row_association_map.at("x") == static_cast<int>(j)) {
        point.pose.position.x = std::stof(file_data.at(i).at(j));
      } else if (label_row_association_map.at("y") == static_cast<int>(j)) {
        point.pose.position.y = std::stof(file_data.at(i).at(j));
      } else if (label_row_association_map.at("z") == static_cast<int>(j)) {
        point.pose.position.z = std::stof(file_data.at(i).at(j));
      } else if (label_row_association_map.at("yaw") == static_cast<int>(j)) {
        yaw = std::stof(file_data.at(i).at(j));
      } else if (label_row_association_map.at("linear_velocity") == static_cast<int>(j)) {
        point.twist.linear.x = std::stof(file_data.at(i).at(j));
      } else if (label_row_association_map.at("angular_velocity") == static_cast<int>(j)) {
        point.twist.angular.z = std::stof(file_data.at(i).at(j));
      } else if (label_row_association_map.at("linear_acceleration") == static_cast<int>(j)) {
        point.accel.linear.x = std::stof(file_data.at(i).at(j));
      } else if (label_row_association_map.at("angular_acceleration") == static_cast<int>(j)) {
        point.accel.angular.z = std::stof(file_data.at(i).at(j));
      }
    }
    roll = 0;
    pitch = 0;
    tf2::Quaternion tf2_quaternion;
    tf2_quaternion.setRPY(roll, pitch, yaw);
    tf2::convert(point.pose.orientation, tf2_quaternion);
    msg.points.push_back(point);
  }
  pub_->publish(msg);
  return true;
}

bool TrajectoryLoaderNode::loadData(
  const std::string & file_name, association & label_row_association_map, csv & file_data)
{
  file_data.clear();
  label_row_association_map.clear();
  std::ifstream ifs(file_name);

  /*
   * open file
   */
  if (!ifs) {
    // todo name arg of get logger
    RCLCPP_ERROR(get_logger(), "%s cannot load", file_name.c_str());
    return false;
  }

  /*
   * create label-row association map
   */
  std::string line;
  if (std::getline(ifs, line)) {
    std::vector<std::string> str_vec = split(line, ',');
    for (size_t i = 0; i < str_vec.size(); ++i) {
      deleteUnit(str_vec.at(i));
      deleteHeadSpace(str_vec.at(i));
      label_row_association_map[str_vec.at(i)] = static_cast<int>(i);
    }
  } else {
    RCLCPP_ERROR(get_logger(), "cannot create association map");
    return false;
  }

  /*
   * create file data
   */
  while (std::getline(ifs, line)) {
    std::vector<std::string> str_vec = split(line, ',');
    file_data.push_back(str_vec);
  }

  return true;
}

std::vector<std::string> TrajectoryLoaderNode::split(const std::string & input, char delimiter)
{
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

void TrajectoryLoaderNode::deleteHeadSpace(std::string & string)
{
  while (string.find_first_of(' ') == 0) {
    string.erase(string.begin());
    if (string.empty()) {break;}
  }
}

void TrajectoryLoaderNode::deleteUnit(std::string & string)
{
  size_t start_pos, end_pos;
  start_pos = string.find_first_of('[');
  end_pos = string.find_last_of(']');
  if (start_pos != std::string::npos && end_pos != std::string::npos && start_pos < end_pos) {
    string.erase(start_pos, (end_pos + 1) - start_pos);
  }
}
