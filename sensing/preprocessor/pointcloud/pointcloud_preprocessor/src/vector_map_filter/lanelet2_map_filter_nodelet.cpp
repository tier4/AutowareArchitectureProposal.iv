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
#include "pointcloud_preprocessor/vector_map_filter/lanelet2_map_filter_nodelet.h"
#include <lanelet2_core/geometry/Polygon.h>
#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

namespace pointcloud_preprocessor
{
void Lanelet2MapFilterNodelet::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  map_sub_ = pnh_.subscribe("input/vector_map", 1, &Lanelet2MapFilterNodelet::mapCallback, this);
  pointcloud_sub_ =
    pnh_.subscribe("input/pointcloud", 1, &Lanelet2MapFilterNodelet::pointcloudCallback, this);
  filtered_pointcloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("output", 1);

  srv_ = std::make_shared<
    dynamic_reconfigure::Server<pointcloud_preprocessor::Lanelet2MapFilterConfig>>();
  dynamic_reconfigure::Server<pointcloud_preprocessor::Lanelet2MapFilterConfig>::CallbackType f =
    boost::bind(&Lanelet2MapFilterNodelet::config_callback, this, _1, _2);
  srv_->setCallback(f);
}

void Lanelet2MapFilterNodelet::config_callback(
  pointcloud_preprocessor::Lanelet2MapFilterConfig & config, uint32_t level)
{
  if (voxel_size_x_ != config.voxel_size_x) {
    voxel_size_x_ = config.voxel_size_x;
    NODELET_DEBUG(
      "[%s::config_callback] Setting voxel_size_x to: %lf.", getName().c_str(),
      config.voxel_size_x);
  }

  if (voxel_size_y_ != config.voxel_size_y) {
    voxel_size_y_ = config.voxel_size_y;
    NODELET_DEBUG(
      "[%s::config_callback] Setting voxel_size_y to: %lf.", getName().c_str(),
      config.voxel_size_y);
  }

  if (voxel_size_z_ != config.voxel_size_z) {
    voxel_size_z_ = config.voxel_size_z;
    NODELET_DEBUG(
      "[%s::config_callback] Setting voxel_size_z to: %lf.", getName().c_str(),
      config.voxel_size_z);
  }
}

bool Lanelet2MapFilterNodelet::transformPointCloud(
  const std::string & in_target_frame, const sensor_msgs::PointCloud2::ConstPtr & in_cloud_ptr,
  sensor_msgs::PointCloud2 * out_cloud_ptr)
{
  if (in_target_frame == in_cloud_ptr->header.frame_id) {
    *out_cloud_ptr = *in_cloud_ptr;
    return true;
  }

  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      in_target_frame, in_cloud_ptr->header.frame_id, in_cloud_ptr->header.stamp,
      ros::Duration(1.0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
  Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(mat, *in_cloud_ptr, *out_cloud_ptr);
  out_cloud_ptr->header.frame_id = in_target_frame;
  return true;
}

LinearRing2d Lanelet2MapFilterNodelet::getConvexHull(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud)
{
  // downsample pointcloud to reduce convex hull calculation cost
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  downsampled_cloud->points.reserve(input_cloud->points.size());
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input_cloud);
  filter.setLeafSize(0.5, 0.5, 100.0);
  filter.filter(*downsampled_cloud);

  MultiPoint2d candidate_points;
  for (const auto & p : downsampled_cloud->points) {
    candidate_points.emplace_back(p.x, p.y);
  }

  LinearRing2d convex_hull;
  boost::geometry::convex_hull(candidate_points, convex_hull);

  return convex_hull;
}

lanelet::ConstLanelets Lanelet2MapFilterNodelet::getIntersectedLanelets(
  const LinearRing2d & convex_hull, const lanelet::ConstLanelets & road_lanelets)
{
  lanelet::ConstLanelets intersected_lanelets;
  for (const auto & road_lanelet : road_lanelets) {
    if (boost::geometry::intersects(convex_hull, road_lanelet.polygon2d().basicPolygon())) {
      intersected_lanelets.push_back(road_lanelet);
    }
  }
  return intersected_lanelets;
}

bool Lanelet2MapFilterNodelet::pointWithinLanelets(
  const Point2d & point, const lanelet::ConstLanelets & intersected_lanelets)
{
  for (const auto & lanelet : intersected_lanelets) {
    if (boost::geometry::within(point, lanelet.polygon2d().basicPolygon())) {
      return true;
    }
  }
  return false;
}

pcl::PointCloud<pcl::PointXYZ> Lanelet2MapFilterNodelet::getLaneFilteredPointCloud(
  const lanelet::ConstLanelets & intersected_lanelets,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

  filtered_cloud.header = cloud->header;
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setLeafSize(voxel_size_x_, voxel_size_y_, 100000.0);
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setSaveLeafLayout(true);
  voxel_grid.filter(*downsampled_cloud);

  std::unordered_map<size_t, pcl::PointCloud<pcl::PointXYZ>> downsampled2original_map;
  for (const auto & p : cloud->points) {
    if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
      continue;
    }
    const int index = voxel_grid.getCentroidIndexAt(voxel_grid.getGridCoordinates(p.x, p.y, p.z));
    if (index == -1) {
      continue;
    }
    downsampled2original_map[index].points.push_back(p);
  }

  for (const auto & point : downsampled_cloud->points) {
    Point2d boost_point(point.x, point.y);
    if (pointWithinLanelets(boost_point, intersected_lanelets)) {
      const int index =
        voxel_grid.getCentroidIndexAt(voxel_grid.getGridCoordinates(point.x, point.y, point.z));
      if (index == -1) {
        continue;
      }
      for (const auto & original_point : downsampled2original_map[index].points) {
        filtered_cloud.points.push_back(original_point);
      }
    }
  }

  return filtered_cloud;
}

void Lanelet2MapFilterNodelet::pointcloudCallback(
  const sensor_msgs::PointCloud2::ConstPtr & cloud_msg)
{
  if (!lanelet_map_ptr_) {
    return;
  }
  // transform pointcloud to map frame
  sensor_msgs::PointCloud2::Ptr input_transed_cloud_ptr(new sensor_msgs::PointCloud2);
  if (!transformPointCloud("map", cloud_msg, input_transed_cloud_ptr.get())) {
    ROS_ERROR_STREAM_THROTTLE(
      10, "Failed transform from "
            << "map"
            << " to " << cloud_msg->header.frame_id);
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_transed_cloud_ptr, *cloud);
  if (cloud->points.empty()) {
    return;
  }
  // calculate convex hull
  const auto convex_hull = getConvexHull(cloud);
  // get intersected lanelets
  lanelet::ConstLanelets intersected_lanelets = getIntersectedLanelets(convex_hull, road_lanelets_);
  // filter pointcloud by lanelet
  const auto filtered_cloud = getLaneFilteredPointCloud(intersected_lanelets, cloud);
  // transform pointcloud to input frame
  sensor_msgs::PointCloud2::Ptr output_cloud_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(filtered_cloud, *output_cloud_ptr);
  sensor_msgs::PointCloud2::Ptr output_transed_cloud_ptr(new sensor_msgs::PointCloud2);
  if (!transformPointCloud(
        cloud_msg->header.frame_id, output_cloud_ptr, output_transed_cloud_ptr.get())) {
    ROS_ERROR_STREAM_THROTTLE(
      10, "Failed transform from "
            << "map"
            << " to " << output_cloud_ptr->header.frame_id);
    return;
  }
  filtered_pointcloud_pub_.publish(*output_transed_cloud_ptr);
}

void Lanelet2MapFilterNodelet::mapCallback(const autoware_lanelet2_msgs::MapBin & map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(map_msg, lanelet_map_ptr_);
  const lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
}

}  // namespace pointcloud_preprocessor

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pointcloud_preprocessor::Lanelet2MapFilterNodelet, nodelet::Nodelet)
