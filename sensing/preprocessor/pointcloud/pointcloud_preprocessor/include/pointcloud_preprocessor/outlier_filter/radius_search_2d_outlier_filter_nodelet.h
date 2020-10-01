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
#include "pointcloud_preprocessor/RadiusSearch2DOutlierFilterConfig.h"
#include "pointcloud_preprocessor/filter.h"

#include <pcl/common/impl/common.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>


namespace pointcloud_preprocessor
{
class RadiusSearch2DOutlierFilterNodelet : public pointcloud_preprocessor::Filter
{
protected:
  boost::shared_ptr<
    dynamic_reconfigure::Server<pointcloud_preprocessor::RadiusSearch2DOutlierFilterConfig> >
    srv_;
  virtual void filter(
    const PointCloud2::ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);
  virtual void subscribe();
  virtual void unsubscribe();

  bool child_init(ros::NodeHandle & nh, bool & has_service);
  void config_callback(
    pointcloud_preprocessor::RadiusSearch2DOutlierFilterConfig & config, uint32_t level);

  double search_radius_;
  double min_neighbors_;

  // pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> radius_outlier_removal_;
  pcl::search::Search<pcl::PointXY>::Ptr kd_tree_;
  // pcl::ExtractIndices<pcl::PCLPointCloud2> extract_indices_;

private:

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pointcloud_preprocessor
