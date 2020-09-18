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

#include "bev_optical_flow/utils.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace bev_optical_flow
{
Utils::Utils() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_) {
  pnh_.param<float>("grid_size", grid_size_, 0.25);
  pnh_.param<float>("point_radius", point_radius_, 50);
  pnh_.param<float>("z_max", z_max_, 3.0);
  pnh_.param<float>("z_min", z_min_, -3.0);
  pnh_.param<std::string>("world_frame", world_frame_, "map");
  pnh_.param<std::string>("target_frame", target_frame_, "base_link");
}

geometry_msgs::Vector3 Utils::mptopic2kph(const geometry_msgs::Vector3& twist,
  double topic_rate)
{
  // convert twist to [km/h] from [m/topic_rate]
  geometry_msgs::Vector3 converted_twist;
  converted_twist.x = (3600.0 / 1000) * twist.x / topic_rate;
  converted_twist.y = (3600.0 / 1000) * twist.y / topic_rate;
  converted_twist.z = (3600.0 / 1000) * twist.z / topic_rate;
  return converted_twist;
}

geometry_msgs::Vector3 Utils::kph2mptopic(
  const geometry_msgs::Vector3& twist,
  double topic_rate)
{
  // convert twist to [km/h] from [m/topic_rate]
  geometry_msgs::Vector3 converted_twist;
  converted_twist.x = (1000.0 / 3600) * twist.x * topic_rate;
  converted_twist.y = (1000.0 / 3600) * twist.y * topic_rate;
  converted_twist.z = (1000.0 / 3600) * twist.z * topic_rate;
  return converted_twist;
}

geometry_msgs::Vector3 Utils::kph2mps(const geometry_msgs::Vector3& twist)
{
  // convert twist to [km/h] from [m/topic_rate]
  geometry_msgs::Vector3 converted_twist;
  converted_twist.x = (1000.0 / 3600) * twist.x;
  converted_twist.y = (1000.0 / 3600) * twist.y;
  converted_twist.z = (1000.0 / 3600) * twist.z;
  return converted_twist;
}

geometry_msgs::Point Utils::pixel2point(
  const geometry_msgs::Point point,
  const geometry_msgs::Vector3 twist,
  const cv::Size& image_size,
  float map2baselink_angle)
{
  return pixel2point
    (cv::Point2f(point.x + twist.x, point.y + twist.y),
      image_size, map2baselink_angle, point.z);
}

geometry_msgs::Twist Utils::getObjCoordsTwist(
  const geometry_msgs::Pose& obj_pose,
  const geometry_msgs::Twist& base_coords_twist)
{
  Eigen::Affine3d base2obj_transform;
  tf::poseMsgToEigen(obj_pose, base2obj_transform);
  Eigen::Matrix3d base2obj_rot = base2obj_transform.rotation();
  Eigen::Vector3d obj_coords_vector =
    base2obj_rot.inverse() * Eigen::Vector3d(base_coords_twist.linear.x,
      base_coords_twist.linear.y,
      base_coords_twist.linear.z);
  geometry_msgs::Twist obj_coords_twist;
  obj_coords_twist.linear.x = obj_coords_vector.x();
  obj_coords_twist.linear.y = obj_coords_vector.y();
  obj_coords_twist.linear.z = obj_coords_vector.z();
  return obj_coords_twist;
}

geometry_msgs::Point Utils::pixel2point(
  const cv::Point2f& pixel,
  const cv::Size& image_size,
  float map2baselink_angle,
  int depth)
{
  // affine transform image coords to rotated coords
  Eigen::Affine2f image2rotated =
    Eigen::Translation<float, 2>(static_cast<int>(image_size.width * 0.5),
      static_cast<int>(image_size.height * 0.5)) *
    Eigen::Rotation2Df(180 * M_PI / 180).toRotationMatrix();

  // affine transform rotated coords to baselink coords
  Eigen::Affine2f rotated2base =
    Eigen::Translation<float, 2>(0, 0) *
    Eigen::Rotation2Df(map2baselink_angle).toRotationMatrix();

  Eigen::Vector2f basecoords_p =
    rotated2base *
    image2rotated *
    Eigen::Vector2f(pixel.x, pixel.y) *
    grid_size_;

  geometry_msgs::Point point;
  point.x = basecoords_p.y();
  point.y = basecoords_p.x();
  point.z = depth * (z_max_ - z_min_) / 255;
  return point;
}

float Utils::getMap2BaseAngle(ros::Time stamp)
{
  // get yaw angle from map to base_link
  float map2baselink_angle = 0;
  try {
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped = tf_buffer_.lookupTransform
      (world_frame_, target_frame_, stamp, ros::Duration(0.5));
    tf2::Quaternion quaternion;
    tf2::fromMsg(transform_stamped.transform.rotation, quaternion);
    double r, p, y;
    tf2::Matrix3x3(quaternion).getRPY(r, p, y);
    map2baselink_angle = y;
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }
  return map2baselink_angle;
}

cv::Point2f Utils::getVehicleVel(
  const ros::Time& current_stamp,
  const ros::Time& prev_stamp)
{
  cv::Point2f image_translation;
  try {
    geometry_msgs::TransformStamped map2currentbase;
    map2currentbase = tf_buffer_.lookupTransform
      (world_frame_, target_frame_, current_stamp, ros::Duration(0.5));
    geometry_msgs::TransformStamped map2prevbase;
    map2prevbase = tf_buffer_.lookupTransform
      (world_frame_, target_frame_, prev_stamp, ros::Duration(0.5));

    cv::Point2f vehicle_translation;
    vehicle_translation =
      cv::Point2f((map2currentbase.transform.translation.x -
          map2prevbase.transform.translation.x),
        (map2currentbase.transform.translation.y -
          map2prevbase.transform.translation.y));
    image_translation= cv::Point2f(-vehicle_translation.y / grid_size_,
      -vehicle_translation.x / grid_size_);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    return image_translation;
  }

  return image_translation;
}
} // bev_optical_flow
