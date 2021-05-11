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

#include <algorithm>
#include <memory>
#include <utility>

#include "vehicle_info_util/vehicle_info_util.hpp"
#include "pure_pursuit/pure_pursuit_node.hpp"
#include "pure_pursuit/util/planning_utils.hpp"
#include "pure_pursuit/util/tf_utils.hpp"
#include "pure_pursuit/pure_pursuit_viz.hpp"

namespace
{
autoware_control_msgs::msg::ControlCommand createControlCommand(
  const double kappa, const double velocity, const double acceleration, const double wheel_base)
{
  autoware_control_msgs::msg::ControlCommand cmd;
  cmd.velocity = velocity;
  cmd.acceleration = acceleration;
  cmd.steering_angle = planning_utils::convertCurvatureToSteeringAngle(wheel_base, kappa);
  return cmd;
}

double calcLookaheadDistance(
  const double velocity, const double lookahead_distance_ratio, const double min_lookahead_distance)
{
  const double lookahead_distance = lookahead_distance_ratio * std::abs(velocity);
  return std::max(lookahead_distance, min_lookahead_distance);
}

}  // namespace

PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions & node_options)
: Node("pure_pursuit", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  pure_pursuit_ = std::make_unique<planning_utils::PurePursuit>();

  // Vehicle Parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  param_.wheel_base = vehicle_info.wheel_base_m;

  // Node Parameters
  param_.ctrl_period = this->declare_parameter<double>("control_period", 0.02);

  // Algorithm Parameters
  param_.lookahead_distance_ratio =
    this->declare_parameter<double>("lookahead_distance_ratio", 2.2);
  param_.min_lookahead_distance = this->declare_parameter<double>("min_lookahead_distance", 2.5);
  param_.reverse_min_lookahead_distance = this->declare_parameter<double>(
    "reverse_min_lookahead_distance", 7.0);

  // Subscribers
  using std::placeholders::_1;
  sub_trajectory_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/reference_trajectory", 1, std::bind(&PurePursuitNode::onTrajectory, this, _1));
  sub_current_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "input/current_velocity", 1, std::bind(&PurePursuitNode::onCurrentVelocity, this, _1));

  // Publishers
  pub_ctrl_cmd_ = this->create_publisher<autoware_control_msgs::msg::ControlCommandStamped>(
    "output/control_raw", 1);

  // Debug Publishers
  pub_debug_marker_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("debug/marker", 0);

  // Timer
  {
    auto timer_callback = std::bind(&PurePursuitNode::onTimer, this);
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(param_.ctrl_period));
    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
      this->get_clock(), period, std::move(timer_callback),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_, nullptr);
  }


  //  Wait for first current pose
  tf_utils::waitForTransform(tf_buffer_, "map", "base_link");
}

bool PurePursuitNode::isDataReady()
{
  if (!current_velocity_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for current_velocity...");
    return false;
  }

  if (!trajectory_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for trajectory...");
    return false;
  }

  if (!current_pose_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for current_pose...");
    return false;
  }

  return true;
}

void PurePursuitNode::onCurrentVelocity(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  current_velocity_ = msg;
}

void PurePursuitNode::onTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  trajectory_ = msg;
}

void PurePursuitNode::onTimer()
{
  current_pose_ = tf_utils::getCurrentPose(tf_buffer_);

  if (!isDataReady()) {
    return;
  }

  const auto target_values = calcTargetValues();

  if (target_values) {
    publishCommand(*target_values);
    publishDebugMarker();
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "failed to solve pure_pursuit");
    publishCommand({0.0, 0.0, 0.0});
  }
}

void PurePursuitNode::publishCommand(const TargetValues & targets)
{
  autoware_control_msgs::msg::ControlCommandStamped cmd;
  cmd.header.stamp = get_clock()->now();
  cmd.control =
    createControlCommand(targets.kappa, targets.velocity, targets.acceleration, param_.wheel_base);
  pub_ctrl_cmd_->publish(cmd);
}

void PurePursuitNode::publishDebugMarker() const
{
  visualization_msgs::msg::MarkerArray marker_array;

  marker_array.markers.push_back(createNextTargetMarker(debug_data_.next_target));
  marker_array.markers.push_back(
    createTrajectoryCircleMarker(debug_data_.next_target, current_pose_->pose));

  pub_debug_marker_->publish(marker_array);
}

boost::optional<TargetValues> PurePursuitNode::calcTargetValues()
{
  // Ignore invalid trajectory
  if (trajectory_->points.size() < 3) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "received path size is < 3, ignored");
    return {};
  }

  // Calculate target point for velocity/acceleration
  const auto target_point = calcTargetPoint();
  if (!target_point) {
    return {};
  }

  const double target_vel = target_point->twist.linear.x;
  const double target_acc = target_point->accel.linear.x;

  // Calculate lookahead distance
  const bool is_reverse = (target_vel < 0);
  const double min_lookahead_distance =
    is_reverse ? param_.reverse_min_lookahead_distance : param_.min_lookahead_distance;
  const double lookahead_distance = calcLookaheadDistance(
    current_velocity_->twist.linear.x, param_.lookahead_distance_ratio, min_lookahead_distance);

  // Set PurePursuit data
  pure_pursuit_->setCurrentPose(current_pose_->pose);
  pure_pursuit_->setWaypoints(planning_utils::extractPoses(*trajectory_));
  pure_pursuit_->setLookaheadDistance(lookahead_distance);

  // Run PurePursuit
  const auto pure_pursuit_result = pure_pursuit_->run();
  if (!pure_pursuit_result.first) {
    return {};
  }

  const auto kappa = pure_pursuit_result.second;

  // Set debug data
  debug_data_.next_target = pure_pursuit_->getLocationOfNextTarget();

  return TargetValues{kappa, target_vel, target_acc};
}

boost::optional<autoware_planning_msgs::msg::TrajectoryPoint> PurePursuitNode::calcTargetPoint()
const
{
  const auto closest_idx_result = planning_utils::findClosestIdxWithDistAngThr(
    planning_utils::extractPoses(*trajectory_), current_pose_->pose, 3.0, M_PI_4);

  if (!closest_idx_result.first) {
    RCLCPP_ERROR(get_logger(), "cannot find closest waypoint");
    return {};
  }

  return trajectory_->points.at(closest_idx_result.second);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PurePursuitNode)
