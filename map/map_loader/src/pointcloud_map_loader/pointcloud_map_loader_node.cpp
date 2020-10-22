/*
 * Copyright 2020 TierIV. All rights reserved.
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

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include <map_loader/pointcloud_map_loader_node.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

PointCloudMapLoaderNode::PointCloudMapLoaderNode(const std::vector<std::string> & pcd_paths)
: Node("poincloud_map_loader")
{
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  pub_pointcloud_map_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("output/pointcloud_map", durable_qos);

  const auto pcd = loadPCDFiles(pcd_paths);

  if (pcd.width == 0) {
    RCLCPP_ERROR(get_logger(), "No PCD was loaded: pcd_paths.size() = %zu", pcd_paths.size());
    return;
  }

  pub_pointcloud_map_->publish(pcd);
}

sensor_msgs::msg::PointCloud2 PointCloudMapLoaderNode::loadPCDFiles(
  const std::vector<std::string> & pcd_paths)
{
  sensor_msgs::msg::PointCloud2 whole_pcd{};

  sensor_msgs::msg::PointCloud2 partial_pcd;
  for (const auto & path : pcd_paths) {
    if (pcl::io::loadPCDFile(path, partial_pcd) == -1) {
      RCLCPP_ERROR_STREAM(get_logger(), "PCD load failed: " << path);
    }

    if (whole_pcd.width == 0) {
      whole_pcd = partial_pcd;
    } else {
      whole_pcd.width += partial_pcd.width;
      whole_pcd.row_step += partial_pcd.row_step;
      whole_pcd.data.reserve(whole_pcd.data.size() + partial_pcd.data.size());
      whole_pcd.data.insert(whole_pcd.data.end(), partial_pcd.data.begin(), partial_pcd.data.end());
    }
  }

  whole_pcd.header.frame_id = "map";

  return whole_pcd;
}
