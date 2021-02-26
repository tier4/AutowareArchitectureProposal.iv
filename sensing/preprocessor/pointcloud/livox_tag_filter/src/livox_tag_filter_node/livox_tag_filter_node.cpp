/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include "livox_tag_filter/livox_tag_filter_node.hpp"

#include <pcl_conversions/pcl_conversions.h>

struct LivoxPoint
{
  float x;
  float y;
  float z;
  float intensity;
  std::uint8_t tag;
  std::uint8_t line;
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
  LivoxPoint, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                std::uint8_t, tag, tag)(std::uint8_t, line, line))

namespace livox_tag_filter
{
LivoxTagFilterNode::LivoxTagFilterNode() : Node("livox_tag_filter")
{
  // Parameter
  ignore_tags_ = this->declare_parameter("ignore_tags", std::vector<std::int64_t>{});

  // Subscriber
  using std::placeholders::_1;
  sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("input", 1, std::bind(&LivoxTagFilterNode::onPointCloud, this, _1));

  // Publisher
  pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", 1);
}

void LivoxTagFilterNode::onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  pcl::PointCloud<LivoxPoint> points;
  pcl::fromROSMsg(*msg, points);

  const auto isIgnored = [&](const LivoxPoint & p) {
    for (const auto & ignore_tag : ignore_tags_) {
      if (p.tag == ignore_tag) {
        return true;
      }
    }

    return false;
  };

  pcl::PointCloud<LivoxPoint> tag_filtered_points;
  for (const auto & p : points) {
    if (isIgnored(p)) {
      continue;
    }

    tag_filtered_points.push_back(p);
  }

  // Publish ROS message
  sensor_msgs::msg::PointCloud2 tag_filtered_msg;
  pcl::toROSMsg(tag_filtered_points, tag_filtered_msg);
  tag_filtered_msg.header = msg->header;

  pub_pointcloud_->publish(tag_filtered_msg);
}

}  // namespace livox_tag_filter
