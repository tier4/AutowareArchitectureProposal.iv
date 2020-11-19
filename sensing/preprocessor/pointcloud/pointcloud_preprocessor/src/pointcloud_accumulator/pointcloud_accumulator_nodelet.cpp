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

#include "pointcloud_preprocessor/pointcloud_accumulator/pointcloud_accumulator_nodelet.h"

namespace pointcloud_preprocessor
{
PointcloudAccumulatorComponent::PointcloudAccumulatorComponent(const rclcpp::NodeOptions & options)
: Filter("PointcloudAccumulator", options)
{
}

void PointcloudAccumulatorComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);
  pointcloud_buffer_.push_front(input);
  rclcpp::Time last_time = input->header.stamp;
  pcl::PointCloud<pcl::PointXYZ> pcl_input;
  pcl::PointCloud<pcl::PointXYZ> pcl_output;
  for (size_t i = 0; i < pointcloud_buffer_.size(); i++) {
    if (accumulation_time_sec_ < (last_time - pointcloud_buffer_.at(i)->header.stamp).seconds())
      break;
    pcl::fromROSMsg(*pointcloud_buffer_.at(i), pcl_input);
    pcl_output += pcl_input;
  }
  pcl::toROSMsg(pcl_output, output);
  output.header = input->header;
}

// void PointcloudAccumulatorComponent::config_callback(
//   pointcloud_preprocessor::PointcloudAccumulatorConfig & config, uint32_t level)
// {
//   boost::mutex::scoped_lock lock(mutex_);

//   if (accumulation_time_sec_ != config.accumulation_time_sec) {
//     accumulation_time_sec_ = config.accumulation_time_sec;
//     NODELET_DEBUG(
//       "[%s::config_callback] Setting new accumulation time to: %f.", getName().c_str(),
//       config.accumulation_time_sec);
//   }
//   if (pointcloud_buffer_.size() != (size_t)config.pointcloud_buffer_size) {
//     NODELET_DEBUG(
//       "[%s::config_callback] Setting new buffer size to: %d.", getName().c_str(),
//       config.pointcloud_buffer_size);
//   }
//   pointcloud_buffer_.set_capacity((size_t)config.pointcloud_buffer_size);
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
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::PointcloudAccumulatorComponent)