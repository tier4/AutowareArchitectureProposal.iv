/*
 * Copyright 2020 TierIV. All rights reserved.
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

#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_perception_msgs/DynamicObjectWithFeatureArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>

namespace bev_optical_flow
{
class Utils
{
public:
  Utils();

  geometry_msgs::Point pixel2point(
    const geometry_msgs::Point point,
    const geometry_msgs::Vector3 twist,
    const cv::Size& image_size,
    float map2baselink_angle);
  geometry_msgs::Point pixel2point(const cv::Point2f& pixel,
    const cv::Size& image_size,
    float map2baselink_angle,
    int depth);
  float getMap2BaseAngle(ros::Time stamp);
  geometry_msgs::Vector3 mptopic2kph(
    const geometry_msgs::Vector3& twist,
    double topic_rate);
  geometry_msgs::Vector3 kph2mptopic(
    const geometry_msgs::Vector3& twist,
    double topic_rate);
  geometry_msgs::Vector3 kph2mps(const geometry_msgs::Vector3& twist);
  cv::Point2f getVehicleVel(
    const ros::Time& current_stamp,
    const ros::Time& prev_stamp);
  geometry_msgs::Twist getObjCoordsTwist(
    const geometry_msgs::Pose& obj_pose,
    const geometry_msgs::Twist& base_coords_twist);
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  float grid_size_;
  float point_radius_;
  float z_max_;
  float z_min_;
  std::string world_frame_;
  std::string target_frame_;
};
} // bev_optical_flow
