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

#include "pointcloud_preprocessor/outlier_filter/ring_outlier_filter_nodelet.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

namespace pointcloud_preprocessor
{
RingOutlierFilterComponent::RingOutlierFilterComponent(const rclcpp::NodeOptions & options)
: Filter("RingOutlierFilter", options)
{
}

void RingOutlierFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);
  pcl::PointCloud<pcl::PointXYZIRADT>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZIRADT>);
  pcl::fromROSMsg(*input, *pcl_input);

  if (pcl_input->points.empty()) {
    return;
  }
  std::vector<pcl::PointCloud<pcl::PointXYZIRADT>> pcl_input_ring_array;
  pcl_input_ring_array.resize(128);  // TODO
  for (const auto & p : pcl_input->points) {
    pcl_input_ring_array.at(p.ring).push_back(p);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_output->points.reserve(pcl_input->points.size());

  pcl::PointCloud<pcl::PointXYZ> pcl_tmp;
  pcl::PointXYZ p;
  for (const auto & ring_pointcloud : pcl_input_ring_array) {
    if (ring_pointcloud.points.size() < 2) {
      continue;
    }

    for (auto iter = std::begin(ring_pointcloud.points);
         iter != std::end(ring_pointcloud.points) - 1; ++iter) {
      p.x = iter->x;
      p.y = iter->y;
      p.z = iter->z;
      pcl_tmp.points.push_back(p);
      // if(std::abs(iter->distance - (iter+1)->distance) <= std::sqrt(iter->distance) * 0.08) {
      const float min_dist = std::min(iter->distance, (iter + 1)->distance);
      const float max_dist = std::max(iter->distance, (iter + 1)->distance);
      float azimuth_diff = (iter + 1)->azimuth - iter->azimuth;
      azimuth_diff = azimuth_diff < 0.f ? azimuth_diff + 36000.f : azimuth_diff;

      if (
        min_dist > 0.f && max_dist > 0.f && azimuth_diff < 100.f &&
        max_dist / min_dist < distance_ratio_) {
        continue;
      } else {
        // same code
        if (
          pcl_tmp.points.size() > num_points_threshold_ ||
          (pcl_tmp.points.front().x - pcl_tmp.points.back().x) *
                (pcl_tmp.points.front().x - pcl_tmp.points.back().x) +
              (pcl_tmp.points.front().y - pcl_tmp.points.back().y) *
                (pcl_tmp.points.front().y - pcl_tmp.points.back().y) +
              (pcl_tmp.points.front().z - pcl_tmp.points.back().z) *
                (pcl_tmp.points.front().z - pcl_tmp.points.back().z) >=
            object_length_threshold_ * object_length_threshold_) {
          for (const auto & tmp_p : pcl_tmp.points) {
            pcl_output->points.push_back(tmp_p);
          }
        }
        pcl_tmp.points.clear();
      }
    }

    // same code
    if (
      pcl_tmp.points.size() > num_points_threshold_ ||
      (pcl_tmp.points.front().x - pcl_tmp.points.back().x) *
            (pcl_tmp.points.front().x - pcl_tmp.points.back().x) +
          (pcl_tmp.points.front().y - pcl_tmp.points.back().y) *
            (pcl_tmp.points.front().y - pcl_tmp.points.back().y) +
          (pcl_tmp.points.front().z - pcl_tmp.points.back().z) *
            (pcl_tmp.points.front().z - pcl_tmp.points.back().z) >=
        object_length_threshold_ * object_length_threshold_) {
      for (const auto & tmp_p : pcl_tmp.points) {
        pcl_output->points.push_back(tmp_p);
      }
    }
    pcl_tmp.points.clear();
  }

  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

// void RingOutlierFilterComponent::config_callback(
//   pointcloud_preprocessor::RingOutlierFilterConfig & config, uint32_t level)
// {
//   boost::mutex::scoped_lock lock(mutex_);

//   if (distance_ratio_ != config.distance_ratio) {
//     distance_ratio_ = config.distance_ratio;
//     NODELET_DEBUG(
//       "[%s::config_callback] Setting new distance ratio to: %f.", getName().c_str(),
//       config.distance_ratio);
//   }
//   if (object_length_threshold_ != config.object_length_threshold) {
//     object_length_threshold_ = config.object_length_threshold;
//     NODELET_DEBUG(
//       "[%s::config_callback] Setting new object length threshold to: %f.", getName().c_str(),
//       config.object_length_threshold);
//   }
//   if (num_points_threshold_ != config.num_points_threshold) {
//     num_points_threshold_ = config.num_points_threshold;
//     NODELET_DEBUG(
//       "[%s::config_callback] Setting new num_points_threshold to: %d.", getName().c_str(),
//       config.num_points_threshold);
//   }
//   // ---[ These really shouldn't be here, and as soon as dynamic_reconfigure improves, we'll remove them and inherit
//   // from Filter
//   if (tf_input_frame_ != config.input_frame) {
//     tf_input_frame_ = config.input_frame;
//     NODELET_DEBUG("[config_callback] Setting the input TF frame to: %s.", tf_input_frame_.c_str());
//   }
//   if (tf_output_frame_ != config.output_frame) {
//     tf_output_frame_ = config.output_frame;
//     NODELET_DEBUG(
//       "[config_callback] Setting the output TF frame to: %s.", tf_output_frame_.c_str());
//   }
//   // ]---
// }

}  // namespace pointcloud_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::RingOutlierFilterComponent)
