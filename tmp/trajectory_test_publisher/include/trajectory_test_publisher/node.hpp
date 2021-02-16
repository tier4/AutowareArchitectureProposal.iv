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

#ifndef TRAJECTORY_TEST_PUBLISHER__NODE_HPP_
#define TRAJECTORY_TEST_PUBLISHER__NODE_HPP_

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

class TrajectoryTestPublisherNode : public rclcpp::Node
{
private:
  // publisher
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr traj_pub_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // timer function
  void timerCallback();

public:
  TrajectoryTestPublisherNode();
  ~TrajectoryTestPublisherNode() {}
};

#endif  // TRAJECTORY_TEST_PUBLISHER__NODE_HPP_
