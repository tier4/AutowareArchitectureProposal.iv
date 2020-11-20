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

#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>
#include "pointcloud_preprocessor/filter.h"

namespace pointcloud_preprocessor
{
class VoxelBasedCompareMapFilterComponent : public pointcloud_preprocessor::Filter
{
protected:
  virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

  // void config_callback(pointcloud_preprocessor::CompareMapFilterConfig & config, uint32_t level);
  void input_target_callback(const PointCloud2ConstPtr map);
  bool is_in_voxel(
    const pcl::PointXYZ & src_point, const pcl::PointXYZ & target_point,
    const double distance_threshold, const PointCloudPtr & map,
    /* Can not add const in PCL specification */ pcl::VoxelGrid<pcl::PointXYZ> & voxel) const;

private:
  // pcl::SegmentDifferences<pcl::PointXYZ> impl_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_map_;
  PointCloudPtr voxel_map_ptr_;
  double distance_threshold_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;
  bool set_map_in_voxel_grid_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VoxelBasedCompareMapFilterComponent(const rclcpp::NodeOptions& options);

};
}  // namespace pointcloud_preprocessor
