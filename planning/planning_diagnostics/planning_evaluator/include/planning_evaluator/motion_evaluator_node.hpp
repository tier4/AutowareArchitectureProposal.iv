// Copyright 2021 Tier IV, Inc.
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

#ifndef PLANNING_EVALUATOR__MOTION_EVALUATOR_NODE_HPP_
#define PLANNING_EVALUATOR__MOTION_EVALUATOR_NODE_HPP_

#include "planning_evaluator/metrics_calculator.hpp"
#include "planning_evaluator/stat.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace planning_diagnostics
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

/**
 * @brief Node for planning evaluation
 */
class MotionEvaluatorNode : public rclcpp::Node
{
public:
  explicit MotionEvaluatorNode(const rclcpp::NodeOptions & node_options);
  ~MotionEvaluatorNode();

  /**
   * @brief callback on vehicle twist message
   * @param [in] twist_msg twist message
   */
  void onTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

private:
  geometry_msgs::msg::Pose getCurrentEgoPose() const;

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  // Parameters
  std::string output_file_str_;

  // Calculator
  MetricsCalculator metrics_calculator_;
  // Metrics
  std::vector<Metric> metrics_;
  std::deque<rclcpp::Time> stamps_;
  Trajectory accumulated_trajectory_;
};
}  // namespace planning_diagnostics

#endif  // PLANNING_EVALUATOR__MOTION_EVALUATOR_NODE_HPP_
