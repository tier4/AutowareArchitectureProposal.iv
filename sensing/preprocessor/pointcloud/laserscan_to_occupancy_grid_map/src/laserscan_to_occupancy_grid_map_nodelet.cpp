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

#include <laserscan_to_occupancy_grid_map/cost_value.h>
#include <laserscan_to_occupancy_grid_map/laserscan_to_occupancy_grid_map_nodelet.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pluginlib/class_list_macros.h>

namespace occupancy_grid_map
{
OccupancyGridMapNodelet::OccupancyGridMapNodelet() {}

void OccupancyGridMapNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  private_nh_.param<std::string>("map_frame", map_frame_, "map");
  tf2_.reset(new tf2_ros::Buffer());
  tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
  bool input_pointcloud;
  private_nh_.param<bool>("input_pointcloud", input_pointcloud, false);
  if (input_pointcloud)
    pointcloud_sub_ = private_nh_.subscribe(
      "input/pointcloud", 1, &OccupancyGridMapNodelet::onPointCloud2Callback, this);
  else
    laserscan_sub_ =
      private_nh_.subscribe("input/scan", 1, &OccupancyGridMapNodelet::onLaserscanCallback, this);

  occupancy_grid_map_pub_ =
    private_nh_.advertise<nav_msgs::OccupancyGrid>("output/occupancy_grid_map", 1);

  double map_length, map_resolution;
  private_nh_.param<double>("map_length", map_length, 100.0);
  private_nh_.param<double>("map_resolution", map_resolution, 0.5);
  occupancy_grid_map_updater_ptr_ = std::make_shared<costmap_2d::OccupancyGridMapBBFUpdater>(
    map_length / map_resolution, map_length / map_resolution, map_resolution);
}

void OccupancyGridMapNodelet::onLaserscanCallback(
  const sensor_msgs::LaserScan::ConstPtr & input_msg)
{
  // check over max range point
  const float max_range =
    static_cast<float>(occupancy_grid_map_updater_ptr_->getSizeInCellsX()) * 0.5f +
    occupancy_grid_map_updater_ptr_->getResolution();
  constexpr float epsilon = 0.001;
  sensor_msgs::LaserScan laserscan = *input_msg;
  laserscan.range_max = max_range;
  for (size_t i = 0; i < laserscan.ranges.size(); i++) {
    float range = laserscan.ranges[i];
    if (max_range < range || std::isinf(range)) {
      laserscan.ranges[i] = max_range - epsilon;
    }
  }

  // convert to pointcloud
  boost::shared_ptr<sensor_msgs::PointCloud2> pointcloud_ptr =
    boost::make_shared<sensor_msgs::PointCloud2>();
  pointcloud_ptr->header = laserscan.header;
  laserscan2pointcloud_converter_.transformLaserScanToPointCloud(
    laserscan.header.frame_id, laserscan, *pointcloud_ptr, *tf2_);

  // call pointcloud callback
  onPointCloud2Callback(pointcloud_ptr);
}

void OccupancyGridMapNodelet::onPointCloud2Callback(
  const sensor_msgs::PointCloud2::ConstPtr & input_msg)
{
  sensor_msgs::PointCloud2 transformed_pointcloud;
  geometry_msgs::Pose pose;
  try {
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped = tf2_->lookupTransform(
      map_frame_, input_msg->header.frame_id, input_msg->header.stamp, ros::Duration(0.1));
    // transform pointcloud
    Eigen::Matrix4f transform_matrix =
      tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
    pcl_ros::transformPointCloud(transform_matrix, *input_msg, transformed_pointcloud);
    // pose
    pose.position.x = transform_stamped.transform.translation.x;
    pose.position.y = transform_stamped.transform.translation.y;
    pose.position.z = transform_stamped.transform.translation.z;
    pose.orientation = transform_stamped.transform.rotation;
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  costmap_2d::OccupancyGridMap oneshot_occupancy_grid_map(
    occupancy_grid_map_updater_ptr_->getSizeInCellsX(),
    occupancy_grid_map_updater_ptr_->getSizeInCellsY(),
    occupancy_grid_map_updater_ptr_->getResolution());

  oneshot_occupancy_grid_map.raytrace2D(transformed_pointcloud, pose);
  occupancy_grid_map_updater_ptr_->update(oneshot_occupancy_grid_map, pose);

  occupancy_grid_map_pub_.publish(OccupancyGridMaptoMsgPtr(
    map_frame_, input_msg->header.stamp, pose.position.z, *occupancy_grid_map_updater_ptr_));
}

boost::shared_ptr<nav_msgs::OccupancyGrid> OccupancyGridMapNodelet::OccupancyGridMaptoMsgPtr(
  const std::string & frame_id, const ros::Time & time, const float & robot_pose_z,
  const costmap_2d::Costmap2D & occupancy_grid_map)
{
  boost::shared_ptr<nav_msgs::OccupancyGrid> msg_ptr(new nav_msgs::OccupancyGrid);

  msg_ptr->header.frame_id = frame_id;
  msg_ptr->header.stamp = time;
  msg_ptr->info.resolution = occupancy_grid_map.getResolution();

  msg_ptr->info.width = occupancy_grid_map.getSizeInCellsX();
  msg_ptr->info.height = occupancy_grid_map.getSizeInCellsY();

  double wx, wy;
  occupancy_grid_map.mapToWorld(0, 0, wx, wy);
  msg_ptr->info.origin.position.x = wx - msg_ptr->info.resolution / 2;
  msg_ptr->info.origin.position.y = wy - msg_ptr->info.resolution / 2;
  msg_ptr->info.origin.position.z = robot_pose_z;
  msg_ptr->info.origin.orientation.w = 1.0;

  msg_ptr->data.resize(msg_ptr->info.width * msg_ptr->info.height);

  unsigned char * data = occupancy_grid_map.getCharMap();
  for (unsigned int i = 0; i < msg_ptr->data.size(); i++) {
    msg_ptr->data[i] = occupancy_cost_value::cost_translation_table[data[i]];
  }
  return msg_ptr;
}

}  // namespace occupancy_grid_map

PLUGINLIB_EXPORT_CLASS(occupancy_grid_map::OccupancyGridMapNodelet, nodelet::Nodelet)
