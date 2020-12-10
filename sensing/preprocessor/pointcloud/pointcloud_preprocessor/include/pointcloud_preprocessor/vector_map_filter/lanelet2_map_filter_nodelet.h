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
#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_utils/geometry/boost_geometry.h>
#include <dynamic_reconfigure/server.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <nodelet/nodelet.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include "pointcloud_preprocessor/Lanelet2MapFilterConfig.h"

using autoware_utils::LinearRing2d;
using autoware_utils::MultiPoint2d;
using autoware_utils::Point2d;

namespace pointcloud_preprocessor
{
class Lanelet2MapFilterNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  ros::Subscriber map_sub_;
  ros::Subscriber pointcloud_sub_;
  ros::Publisher filtered_pointcloud_pub_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::ConstLanelets road_lanelets_;

  float voxel_size_x_;
  float voxel_size_y_;
  float voxel_size_z_;

  std::shared_ptr<dynamic_reconfigure::Server<pointcloud_preprocessor::Lanelet2MapFilterConfig>>
    srv_;

  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr & msg);

  void mapCallback(const autoware_lanelet2_msgs::MapBin & msg);

  void config_callback(pointcloud_preprocessor::Lanelet2MapFilterConfig & config, uint32_t level);

  bool transformPointCloud(
    const std::string & in_target_frame, const sensor_msgs::PointCloud2::ConstPtr & in_cloud_ptr,
    sensor_msgs::PointCloud2 * out_cloud_ptr);

  LinearRing2d getConvexHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud);

  lanelet::ConstLanelets getIntersectedLanelets(
    const LinearRing2d & convex_hull, const lanelet::ConstLanelets & road_lanelets_);

  pcl::PointCloud<pcl::PointXYZ> getLaneFilteredPointCloud(
    const lanelet::ConstLanelets & joint_lanelets,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  bool pointWithinLanelets(const Point2d & point, const lanelet::ConstLanelets & joint_lanelets);
};

}  // namespace pointcloud_preprocessor
