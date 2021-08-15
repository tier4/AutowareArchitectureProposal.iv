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
/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#ifndef PURE_PURSUIT__PURE_PURSUIT_NODE_HPP_
#define PURE_PURSUIT__PURE_PURSUIT_NODE_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "autoware_control_msgs/msg/control_command_stamped.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "boost/optional.hpp"  // To be replaced by std::optional in C++17
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "pure_pursuit/pure_pursuit_viz.hpp"
#include "pure_pursuit/pure_pursuit.hpp"

struct Param
{
  // Global Parameters
  double wheel_base;

  // Node Parameters
  double ctrl_period;

  // Algorithm Parameters
  double lookahead_distance_ratio;
  double min_lookahead_distance;
  double reverse_min_lookahead_distance;  // min_lookahead_distance in reverse gear
};

struct TargetValues
{
  double kappa;
  double velocity;
  double acceleration;
};

struct DebugData
{
  geometry_msgs::msg::Point next_target;
};

class PurePursuitNode : public rclcpp::Node
{
public:
  explicit PurePursuitNode(const rclcpp::NodeOptions & node_options);

private:
  // Subscriber
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_current_velocity_;

  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_velocity_;

  bool isDataReady();

  void onTrajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
  void onCurrentVelocity(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  boost::optional<geometry_msgs::msg::PoseStamped> current_pose_;

  // Publisher
  rclcpp::Publisher<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr pub_ctrl_cmd_;

  void publishCommand(const TargetValues & targets);

  // Debug Publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_marker_;

  void publishDebugMarker() const;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();

  // Parameter
  Param param_;

  // Algorithm
  std::unique_ptr<planning_utils::PurePursuit> pure_pursuit_;

  boost::optional<TargetValues> calcTargetValues();
  boost::optional<autoware_planning_msgs::msg::TrajectoryPoint> calcTargetPoint() const;

  // Debug
  mutable DebugData debug_data_;
};

#endif  // PURE_PURSUIT__PURE_PURSUIT_NODE_HPP_
