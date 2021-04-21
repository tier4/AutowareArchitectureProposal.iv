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

#include <memory>
#include <string>

#include "bev_optical_flow/lidar_to_image.hpp"

namespace bev_optical_flow
{
LidarToBEVImage::LidarToBEVImage(rclcpp::Node & node)
: logger_(node.get_logger()), clock_(node.get_clock())
{
  grid_size_ = static_cast<float>(get_double_param(node, "grid_size", 0.25));
  point_radius_ = static_cast<float>(get_double_param(node, "point_radius", 50.0));
  z_max_ = static_cast<float>(get_double_param(node, "z_max", 3.0));
  z_min_ = static_cast<float>(get_double_param(node, "z_min", -3.0));
  depth_max_ = node.declare_parameter("depth_max", 255);
  depth_min_ = node.declare_parameter("depth_min", 0);

  image_size_ = 2 * static_cast<int>(point_radius_ / grid_size_);
  utils_ = std::make_shared<Utils>(node);
}

double LidarToBEVImage::get_double_param(
  rclcpp::Node & node, std::string p, const double default_value)
{
  if (node.has_parameter(p)) {
    return node.get_parameter(p).as_double();
  } else {
    return node.declare_parameter(p, default_value);
  }
}

float LidarToBEVImage::pointToPixel(
  const pcl::PointXYZ & point, cv::Point2d & px, float map2base_angle)
{
  // affine transform base_link coords to image coords
  Eigen::Affine2f base2image = Eigen::Translation<float, 2>(point_radius_, point_radius_) *
    Eigen::Rotation2Df(M_PI + map2base_angle).toRotationMatrix();
  Eigen::Vector2f transformed_p = (base2image * Eigen::Vector2f(point.x, point.y)) / grid_size_;
  px.x = transformed_p[1];
  px.y = transformed_p[0];
  float intensity = (point.z + std::abs(z_min_)) / (z_max_ - z_min_);
  return std::round((depth_max_ - depth_min_) * intensity);
}

void LidarToBEVImage::getBEVImage(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg, cv::Mat & bev_image)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  bev_image = cv::Mat::zeros(cv::Size(image_size_, image_size_), CV_8UC1);

  float map2base_angle = utils_->getMap2BaseAngle(cloud_msg->header.stamp);

  size_t size = cloud->points.size();
  for (size_t i = 0; i < size; ++i) {
    auto p = cloud->points.at(i);
    if (
      p.x < -point_radius_ || point_radius_ < p.x || p.y < -point_radius_ || point_radius_ < p.y ||
      p.z < z_min_ || z_max_ < p.z)
    {
      continue;
    }
    cv::Point2d px;
    float depth = pointToPixel(p, px, map2base_angle);
    // bev_image.at<unsigned char>(static_cast<int>(px.y),
    // static_cast<int>(px.x)) = static_cast<int>(depth);
    cv::circle(bev_image, px, 0, static_cast<int>(depth), -1);
  }
}

}  // namespace bev_optical_flow
