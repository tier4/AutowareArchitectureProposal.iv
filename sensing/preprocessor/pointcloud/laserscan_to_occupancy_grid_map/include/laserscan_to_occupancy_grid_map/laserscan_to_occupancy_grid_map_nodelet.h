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

#include <laser_geometry/laser_geometry.h>
#include <laserscan_to_occupancy_grid_map/occupancy_grid_map.h>
#include <laserscan_to_occupancy_grid_map/updater/occupancy_grid_map_binary_bayes_filter_updater.h>
#include <laserscan_to_occupancy_grid_map/updater/occupancy_grid_map_updater_interface.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

namespace occupancy_grid_map
{
class OccupancyGridMapNodelet : public nodelet::Nodelet
{
public:
  OccupancyGridMapNodelet();

private:
  virtual void onInit();

  void onLaserscanCallback(const sensor_msgs::LaserScan::ConstPtr & input_msg);
  void onPointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr & input_msg);
  boost::shared_ptr<nav_msgs::OccupancyGrid> OccupancyGridMaptoMsgPtr(
    const std::string & frame_id, const ros::Time & time, const float & robot_pose_z,
    const costmap_2d::Costmap2D & occupancy_grid_map);
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber laserscan_sub_;
  ros::Subscriber pointcloud_sub_;
  ros::Publisher occupancy_grid_map_pub_;

  boost::shared_ptr<tf2_ros::Buffer> tf2_;
  boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  laser_geometry::LaserProjection laserscan2pointcloud_converter_;

  std::shared_ptr<costmap_2d::OccupancyGridMapUpdaterInterface> occupancy_grid_map_updater_ptr_;

  // ROS Parameters
  std::string map_frame_;
};

}  // namespace occupancy_grid_map
