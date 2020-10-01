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

#include "pointcloud_preprocessor/outlier_filter/radius_search_2d_outlier_filter_nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

namespace pointcloud_preprocessor
{
bool RadiusSearch2DOutlierFilterNodelet::child_init(ros::NodeHandle & nh, bool & has_service)
{
  // Enable the dynamic reconfigure service
  has_service = true;
  srv_ = boost::make_shared<
    dynamic_reconfigure::Server<pointcloud_preprocessor::RadiusSearch2DOutlierFilterConfig> >(nh);
  dynamic_reconfigure::Server<pointcloud_preprocessor::RadiusSearch2DOutlierFilterConfig>::CallbackType
    f = boost::bind(&RadiusSearch2DOutlierFilterNodelet::config_callback, this, _1, _2);
  srv_->setCallback(f);
  kd_tree_ = boost::make_shared<pcl::search::KdTree<pcl::PointXY> >(false);

  return (true);
}

void RadiusSearch2DOutlierFilterNodelet::filter(
  const PointCloud2::ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *xyz_cloud);

  pcl::PointCloud<pcl::PointXY>::Ptr xy_cloud(new pcl::PointCloud<pcl::PointXY>);
  xy_cloud->points.resize(xyz_cloud->points.size());
  for (int i=0; i<xyz_cloud->points.size(); i++) {
    xy_cloud->points[i].x = xyz_cloud->points[i].x;
    xy_cloud->points[i].y = xyz_cloud->points[i].y;
  }

  std::vector<int> k_indices(xy_cloud->points.size());
  std::vector<float> k_dists(xy_cloud->points.size());
  kd_tree_->setInputCloud(xy_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < xy_cloud->points.size(); i++)
  {
    int k = kd_tree_->radiusSearch(i, search_radius_, k_indices, k_dists);
    if (k >= min_neighbors_)
    {
      pcl_output->points.push_back(xyz_cloud->points.at(i));
    }
  }
  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

void RadiusSearch2DOutlierFilterNodelet::subscribe() { Filter::subscribe(); }

void RadiusSearch2DOutlierFilterNodelet::unsubscribe() { Filter::unsubscribe(); }

void RadiusSearch2DOutlierFilterNodelet::config_callback(
  pointcloud_preprocessor::RadiusSearch2DOutlierFilterConfig & config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (min_neighbors_ != config.min_neighbors) {
    min_neighbors_ = config.min_neighbors;
    NODELET_DEBUG(
      "[%s::config_callback] Setting new distance threshold to: %f.", getName().c_str(),
      config.min_neighbors);
  }

  if (search_radius_ != config.search_radius) {
    search_radius_ = config.search_radius;
    NODELET_DEBUG(
      "[%s::config_callback] Setting new distance threshold to: %f.", getName().c_str(),
      config.search_radius);
  }
}

}  // namespace pointcloud_preprocessor

PLUGINLIB_EXPORT_CLASS(pointcloud_preprocessor::RadiusSearch2DOutlierFilterNodelet, nodelet::Nodelet);
