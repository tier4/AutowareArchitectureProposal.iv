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

#ifndef BEV_OPTICAL_FLOW__UTILS_HPP_
#define BEV_OPTICAL_FLOW__UTILS_HPP_

#include <memory>
#include <string>

#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_listener.h"

#include "autoware_perception_msgs/msg/dynamic_object_with_feature_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace bev_optical_flow
{
class Utils
{
public:
  explicit Utils(rclcpp::Node & node);

  geometry_msgs::msg::Point pixel2point(
    const geometry_msgs::msg::Point point, const geometry_msgs::msg::Vector3 twist,
    const cv::Size & image_size, float map2baselink_angle);
  geometry_msgs::msg::Point pixel2point(
    const cv::Point2f & pixel, const cv::Size & image_size, float map2baselink_angle, int depth);
  float getMap2BaseAngle(const rclcpp::Time & stamp);
  geometry_msgs::msg::Vector3 mptopic2kph(
    const geometry_msgs::msg::Vector3 & twist, double topic_rate);
  geometry_msgs::msg::Vector3 kph2mptopic(
    const geometry_msgs::msg::Vector3 & twist, double topic_rate);
  geometry_msgs::msg::Vector3 kph2mps(const geometry_msgs::msg::Vector3 & twist);
  cv::Point2f getVehicleVel(const rclcpp::Time & current_stamp, const rclcpp::Time & prev_stamp);
  geometry_msgs::msg::Twist getObjCoordsTwist(
    const geometry_msgs::msg::Pose & obj_pose, const geometry_msgs::msg::Twist & base_coords_twist);

private:
  template<typename T>
  T get_param(rclcpp::Node & node, std::string p, const T default_value)
  {
    if (node.has_parameter(p)) {
      return node.get_parameter(p).get_value<T>();
    } else {
      return node.declare_parameter(p, default_value);
    }
  }

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  float grid_size_;
  float point_radius_;
  float z_max_;
  float z_min_;
  std::string world_frame_;
  std::string target_frame_;
};
}  // namespace bev_optical_flow

#endif  // BEV_OPTICAL_FLOW__UTILS_HPP_
