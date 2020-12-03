/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#ifndef LANE_CHANGE_PLANNER_DATA_MANAGER_H
#define LANE_CHANGE_PLANNER_DATA_MANAGER_H

// ROS
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_listener.h>

// Autoware
#include <autoware_lanelet2_msgs/msg/map_bin.hpp>
#include <autoware_perception_msgs/msg/dynamic_object_array.hpp>
#include <autoware_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/route.hpp>
#include <lane_change_planner/parameters.hpp>

// lanelet
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

// other
#include <memory>

namespace lane_change_planner
{
class SelfPoseLinstener
{
public:
  SelfPoseLinstener(const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr & clock);
  bool getSelfPose(geometry_msgs::msg::PoseStamped & self_pose);
  bool isSelfPoseReady();

private:
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
};

struct BoolStamped
{
  explicit BoolStamped(bool in_data) : data(in_data) {}
  bool data = false;
  rclcpp::Time stamp;
};

class DataManager
{
private:
  /*
   * Cache
   */
  autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr perception_ptr_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr vehicle_velocity_ptr_;
  BoolStamped lane_change_approval_;
  BoolStamped force_lane_change_;
  geometry_msgs::msg::PoseStamped self_pose_;

  // ROS parameters
  LaneChangerParameters parameters_;
  bool is_parameter_set_;

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  /*
   * SelfPoseLinstener
   */
  std::shared_ptr<SelfPoseLinstener> self_pose_listener_ptr_;

public:
  DataManager(const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr & clock);
  ~DataManager() = default;

  // callbacks
  void perceptionCallback(
    const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_perception_msg);
  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr input_twist_msg);
  void setLaneChangerParameters(const LaneChangerParameters & parameters);
  void laneChangeApprovalCallback(const std_msgs::msg::Bool::ConstSharedPtr input_approval_msg);
  void forceLaneChangeSignalCallback(const std_msgs::msg::Bool::ConstSharedPtr input_approval_msg);

  // getters
  autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr getDynamicObjects();
  geometry_msgs::msg::PoseStamped getCurrentSelfPose();
  geometry_msgs::msg::TwistStamped::ConstSharedPtr getCurrentSelfVelocity();
  LaneChangerParameters getLaneChangerParameters();
  bool getLaneChangeApproval();
  bool getForceLaneChangeSignal();
  rclcpp::Logger & getLogger();
  rclcpp::Clock::SharedPtr getClock();

  bool isDataReady();
};
}  // namespace lane_change_planner

#endif  // LANE_CHANGE_PLANNER_DATA_MANAGER_H
