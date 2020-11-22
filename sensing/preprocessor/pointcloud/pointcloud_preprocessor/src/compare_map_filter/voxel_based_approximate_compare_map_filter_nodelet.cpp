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

#include "pointcloud_preprocessor/compare_map_filter/voxel_based_approximate_compare_map_filter_nodelet.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

namespace pointcloud_preprocessor
{
VoxelBasedApproximateCompareMapFilterComponent::VoxelBasedApproximateCompareMapFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("VoxelBasedApproximateCompareMapFilter", options)
{
  distance_threshold_ = static_cast<double>(declare_parameter("distance_threshold", 0.3));

  set_map_in_voxel_grid_ = false;

  using std::placeholders::_1;
  sub_map_ = this->create_subscription<PointCloud2>(
    "map", rclcpp::QoS{1},
    std::bind(&VoxelBasedApproximateCompareMapFilterComponent::input_target_callback, this, _1));

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&VoxelBasedApproximateCompareMapFilterComponent::paramCallback, this, _1));
}

void VoxelBasedApproximateCompareMapFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (voxel_map_ptr_ == NULL) {
    output = *input;
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);
  pcl_output->points.reserve(pcl_input->points.size());
  for (size_t i = 0; i < pcl_input->points.size(); ++i) {
    const int index = voxel_grid_.getCentroidIndexAt(voxel_grid_.getGridCoordinates(
      pcl_input->points.at(i).x, pcl_input->points.at(i).y, pcl_input->points.at(i).z));
    if (index == -1)  // empty voxel
    {
      // map_ptr_->points.at(index)
      pcl_output->points.push_back(pcl_input->points.at(i));
    }
  }

  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

void VoxelBasedApproximateCompareMapFilterComponent::input_target_callback(
  const PointCloud2ConstPtr map)
{
  pcl::PointCloud<pcl::PointXYZ> map_pcl;
  pcl::fromROSMsg<pcl::PointXYZ>(*map, map_pcl);
  const auto map_pcl_ptr = boost::make_shared<const pcl::PointCloud<pcl::PointXYZ>>(map_pcl);

  boost::mutex::scoped_lock lock(mutex_);
  set_map_in_voxel_grid_ = true;
  tf_input_frame_ = map_pcl_ptr->header.frame_id;
  voxel_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid_.setLeafSize(distance_threshold_, distance_threshold_, distance_threshold_);
  voxel_grid_.setInputCloud(map_pcl_ptr);
  voxel_grid_.setSaveLeafLayout(true);
  voxel_grid_.filter(*voxel_map_ptr_);
}

rcl_interfaces::msg::SetParametersResult
VoxelBasedApproximateCompareMapFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (get_param(p, "distance_threshold", distance_threshold_)) {
    voxel_grid_.setLeafSize(distance_threshold_, distance_threshold_, distance_threshold_);
    voxel_grid_.setSaveLeafLayout(true);
    if (set_map_in_voxel_grid_) voxel_grid_.filter(*voxel_map_ptr_);
    RCLCPP_DEBUG(get_logger(), "Setting new distance threshold to: %f.", distance_threshold_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace pointcloud_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  pointcloud_preprocessor::VoxelBasedApproximateCompareMapFilterComponent)
