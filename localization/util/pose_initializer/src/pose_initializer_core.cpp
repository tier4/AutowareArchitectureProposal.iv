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

#include <chrono>
#include <functional>

#include "pose_initializer/pose_initializer_core.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl_conversions/pcl_conversions.h>

double getGroundHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcdmap, const tf2::Vector3 & point)
{
  constexpr double radius = 1.0 * 1.0;
  const double x = point.getX();
  const double y = point.getY();

  double height = INFINITY;
  for (const auto & p : pcdmap->points) {
    const double dx = x - p.x;
    const double dy = y - p.y;
    const double sd = (dx * dx) + (dy * dy);
    if (sd < radius) {
      height = std::min(height, static_cast<double>(p.z));
    }
  }
  return std::isfinite(height) ? height : point.getZ();
}

PoseInitializer::PoseInitializer()
: Node("pose_initializer"), tf2_listener_(tf2_buffer_), map_frame_("map")
{
  // We can't use _1 because pcl leaks an alias to boost::placeholders::_1, so it would be ambiguous
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&PoseInitializer::callbackInitialPose, this, std::placeholders::_1));
  map_points_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pointcloud_map", 1,
    std::bind(&PoseInitializer::callbackMapPoints, this, std::placeholders::_1));

  const bool use_first_gnss_topic = this->declare_parameter("use_first_gnss_topic", true);
  if (use_first_gnss_topic) {
    gnss_pose_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "gnss_pose_cov", 1,
      std::bind(&PoseInitializer::callbackGNSSPoseCov, this, std::placeholders::_1));
  }

  initial_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose3d", 10);

  ndt_client_ = this->create_client<autoware_localization_srvs::srv::PoseWithCovarianceStamped>(
    "ndt_align_srv");
  ndt_client_->wait_for_service(std::chrono::seconds(1));  // TODO

  gnss_service_ = this->create_service<autoware_localization_srvs::srv::PoseWithCovarianceStamped>(
    "pose_initializer_srv",
    std::bind(
      &PoseInitializer::serviceInitial, this, std::placeholders::_1, std::placeholders::_2));
}

PoseInitializer::~PoseInitializer() {}

void PoseInitializer::callbackMapPoints(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr map_points_msg_ptr)
{
  std::string map_frame_ = map_points_msg_ptr->header.frame_id;
  map_ptr_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*map_points_msg_ptr, *map_ptr_);
}

void PoseInitializer::serviceInitial(
  const std::shared_ptr<autoware_localization_srvs::srv::PoseWithCovarianceStamped::Request> req,
  std::shared_ptr<autoware_localization_srvs::srv::PoseWithCovarianceStamped::Response> res)
{
  // TODO(nikolai)
  // gnss_pose_sub_.shutdown();  // get only first topic

  auto add_height_pose_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  getHeight(req->pose_with_cov, add_height_pose_msg_ptr);

  // TODO
  add_height_pose_msg_ptr->pose.covariance[0] = 1.0;
  add_height_pose_msg_ptr->pose.covariance[1 * 6 + 1] = 1.0;
  add_height_pose_msg_ptr->pose.covariance[2 * 6 + 2] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[3 * 6 + 3] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[4 * 6 + 4] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[5 * 6 + 5] = 1.0;

  auto aligned_pose_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  const bool succeeded_align = callAlignService(add_height_pose_msg_ptr, aligned_pose_msg_ptr);

  if (succeeded_align) {
    initial_pose_pub_->publish(*aligned_pose_msg_ptr);
  } else {
    throw std::runtime_error("Aligning poses did not succeed.");
  }
}

void PoseInitializer::callbackInitialPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_cov_msg_ptr)
{
  // TODO(nikolai)
  // gnss_pose_sub_.shutdown();  // get only first topic

  auto add_height_pose_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  getHeight(*pose_cov_msg_ptr, add_height_pose_msg_ptr);

  // TODO
  add_height_pose_msg_ptr->pose.covariance[0] = 2.0;
  add_height_pose_msg_ptr->pose.covariance[1 * 6 + 1] = 2.0;
  add_height_pose_msg_ptr->pose.covariance[2 * 6 + 2] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[3 * 6 + 3] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[4 * 6 + 4] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[5 * 6 + 5] = 0.3;

  auto aligned_pose_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  const bool succeeded_align = callAlignService(add_height_pose_msg_ptr, aligned_pose_msg_ptr);

  if (succeeded_align) {
    initial_pose_pub_->publish(*aligned_pose_msg_ptr);
  }
}

// NOTE Still not usable callback
void PoseInitializer::callbackGNSSPoseCov(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_cov_msg_ptr)
{
  // TODO(nikolai)
  // gnss_pose_sub_.shutdown();  // get only first topic

  // TODO check service is available

  auto add_height_pose_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  getHeight(*pose_cov_msg_ptr, add_height_pose_msg_ptr);

  // TODO
  add_height_pose_msg_ptr->pose.covariance[0] = 1.0;
  add_height_pose_msg_ptr->pose.covariance[1 * 6 + 1] = 1.0;
  add_height_pose_msg_ptr->pose.covariance[2 * 6 + 2] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[3 * 6 + 3] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[4 * 6 + 4] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[5 * 6 + 5] = 3.14;

  auto aligned_pose_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  const bool succeeded_align = callAlignService(add_height_pose_msg_ptr, aligned_pose_msg_ptr);

  if (succeeded_align) {
    initial_pose_pub_->publish(*aligned_pose_msg_ptr);
  }
}

bool PoseInitializer::getHeight(
  const geometry_msgs::msg::PoseWithCovarianceStamped & input_pose_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr output_pose_msg_ptr)
{
  std::string fixed_frame = input_pose_msg.header.frame_id;
  tf2::Vector3 point(
    input_pose_msg.pose.pose.position.x, input_pose_msg.pose.pose.position.y,
    input_pose_msg.pose.pose.position.z);

  if (map_ptr_) {
    tf2::Transform transform;
    try {
      const auto stamped = tf2_buffer_.lookupTransform(map_frame_, fixed_frame, tf2::TimePointZero);
      tf2::fromMsg(stamped.transform, transform);
    } catch (tf2::TransformException & exception) {
      RCLCPP_WARN_STREAM(get_logger(), "failed to lookup transform: " << exception.what());
    }

    point = transform * point;
    point.setZ(getGroundHeight(map_ptr_, point));
    point = transform.inverse() * point;
  }

  *output_pose_msg_ptr = input_pose_msg;
  output_pose_msg_ptr->pose.pose.position.x = point.getX();
  output_pose_msg_ptr->pose.pose.position.y = point.getY();
  output_pose_msg_ptr->pose.pose.position.z = point.getZ();

  return true;
}

bool PoseInitializer::callAlignService(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr input_pose_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr output_pose_msg_ptr)
{
  auto req =
    std::make_shared<autoware_localization_srvs::srv::PoseWithCovarianceStamped::Request>();
  req->pose_with_cov = *input_pose_msg;

  RCLCPP_INFO(get_logger(), "call NDT Align Server");

  auto result = ndt_client_->async_send_request(req);
  // Wait for the result. `shared_from_this` is fine since this class is only used with `make_shared`.
  if (
    rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(get_logger(), "called NDT Align Server");
    // NOTE temporary cov
    geometry_msgs::msg::PoseWithCovarianceStamped & pose_with_cov = result.get()->pose_with_cov;
    pose_with_cov.pose.covariance[0] = 1.0;
    pose_with_cov.pose.covariance[1 * 6 + 1] = 1.0;
    pose_with_cov.pose.covariance[2 * 6 + 2] = 0.01;
    pose_with_cov.pose.covariance[3 * 6 + 3] = 0.01;
    pose_with_cov.pose.covariance[4 * 6 + 4] = 0.01;
    pose_with_cov.pose.covariance[5 * 6 + 5] = 0.2;
    *output_pose_msg_ptr = pose_with_cov;
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "could not call NDT Align Server");
    return false;
  }
}
