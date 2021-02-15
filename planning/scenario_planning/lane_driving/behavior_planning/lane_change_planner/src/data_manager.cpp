// Copyright 2019 Autoware Foundation. All rights reserved.
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

#include "lane_change_planner/data_manager.hpp"
#include <string>
#include <memory>
#include "lanelet2_extension/utility/message_conversion.hpp"

namespace lane_change_planner
{
DataManager::DataManager(const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr & clock)
: lane_change_approval_(false), force_lane_change_(false), is_parameter_set_(false),
  logger_(logger), clock_(clock)
{
  self_pose_listener_ptr_ = std::make_shared<SelfPoseListener>(logger, clock);
}

void DataManager::perceptionCallback(
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_perception_msg_ptr)
{
  perception_ptr_ = input_perception_msg_ptr;
}

void DataManager::velocityCallback(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr input_twist_msg_ptr)
{
  vehicle_velocity_ptr_ = input_twist_msg_ptr;
}

void DataManager::laneChangeApprovalCallback(
  const std_msgs::msg::Bool::ConstSharedPtr input_approval_msg)
{
  lane_change_approval_.data = input_approval_msg->data;
  lane_change_approval_.stamp = clock_->now();
}

void DataManager::forceLaneChangeSignalCallback(
  const std_msgs::msg::Bool::ConstSharedPtr input_force_lane_change_msg)
{
  force_lane_change_.data = input_force_lane_change_msg->data;
  force_lane_change_.stamp = clock_->now();
}

void DataManager::setLaneChangerParameters(const LaneChangerParameters & parameters)
{
  is_parameter_set_ = true;
  parameters_ = parameters;
}

autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr DataManager::getDynamicObjects()
{
  return perception_ptr_;
}

geometry_msgs::msg::TwistStamped::ConstSharedPtr DataManager::getCurrentSelfVelocity()
{
  return vehicle_velocity_ptr_;
}

geometry_msgs::msg::PoseStamped DataManager::getCurrentSelfPose()
{
  self_pose_listener_ptr_->getSelfPose(self_pose_);
  return self_pose_;
}

LaneChangerParameters DataManager::getLaneChangerParameters() {return parameters_;}

bool DataManager::getLaneChangeApproval()
{
  constexpr double timeout = 0.5;
  if (clock_->now() - lane_change_approval_.stamp > rclcpp::Duration::from_seconds(timeout)) {
    return false;
  }

  return lane_change_approval_.data;
}

bool DataManager::getForceLaneChangeSignal()
{
  constexpr double timeout = 0.5;
  if (clock_->now() - force_lane_change_.stamp > rclcpp::Duration::from_seconds(timeout)) {
    return false;
  } else {
    return force_lane_change_.data;
  }
}

bool DataManager::isDataReady()
{
  if (!perception_ptr_) {
    return false;
  }
  if (!vehicle_velocity_ptr_) {
    return false;
  }
  if (!self_pose_listener_ptr_->isSelfPoseReady()) {
    return false;
  }
  return true;
}

SelfPoseListener::SelfPoseListener(
  const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr & clock)
: tf_buffer_(clock), tf_listener_(tf_buffer_), logger_(logger), clock_(clock)
{
}

bool SelfPoseListener::isSelfPoseReady()
{
  return tf_buffer_.canTransform("map", "base_link", tf2::TimePointZero);
}

bool SelfPoseListener::getSelfPose(geometry_msgs::msg::PoseStamped & self_pose)
{
  try {
    geometry_msgs::msg::TransformStamped transform;
    std::string map_frame = "map";
    transform = tf_buffer_.lookupTransform(map_frame, "base_link", tf2::TimePointZero);
    self_pose.pose.position.x = transform.transform.translation.x;
    self_pose.pose.position.y = transform.transform.translation.y;
    self_pose.pose.position.z = transform.transform.translation.z;
    self_pose.pose.orientation.x = transform.transform.rotation.x;
    self_pose.pose.orientation.y = transform.transform.rotation.y;
    self_pose.pose.orientation.z = transform.transform.rotation.z;
    self_pose.pose.orientation.w = transform.transform.rotation.w;
    self_pose.header.stamp = clock_->now();
    self_pose.header.frame_id = map_frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM_THROTTLE(logger_, *clock_, 1000, "failed to find self pose :" << ex.what());
    return false;
  }
}
}  // namespace lane_change_planner
