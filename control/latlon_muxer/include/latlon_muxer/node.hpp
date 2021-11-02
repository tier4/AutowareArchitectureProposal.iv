// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef LATLON_MUXER__NODE_HPP_
#define LATLON_MUXER__NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control_command_stamped.hpp>

#include <memory>
#include <string>

class LatLonMuxer : public rclcpp::Node
{
public:
  explicit LatLonMuxer(const rclcpp::NodeOptions & node_options);

private:
  void latCtrlCmdCallback(const autoware_control_msgs::msg::ControlCommandStamped::SharedPtr msg);
  void lonCtrlCmdCallback(const autoware_control_msgs::msg::ControlCommandStamped::SharedPtr msg);
  void publishCmd();
  bool checkTimeout();

  rclcpp::Publisher<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr control_cmd_pub_;
  rclcpp::Subscription<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr
    lat_control_cmd_sub_;
  rclcpp::Subscription<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr
    lon_control_cmd_sub_;

  std::shared_ptr<autoware_control_msgs::msg::ControlCommandStamped> lat_cmd_;
  std::shared_ptr<autoware_control_msgs::msg::ControlCommandStamped> lon_cmd_;
  double timeout_thr_sec_;
};

#endif  // LATLON_MUXER__NODE_HPP_
