// Copyright 2019-2020 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef NDT_NODES__MAP_PUBLISHER_HPP_
#define NDT_NODES__MAP_PUBLISHER_HPP_

#include <ndt_nodes/visibility_control.hpp>
#include <ndt/ndt_map_publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ndt/ndt_map.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <voxel_grid_nodes/algorithm/voxel_cloud_centroid.hpp>
#include <string>
#include <memory>
#include "common/types.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;

namespace autoware
{
namespace localization
{
namespace ndt_nodes
{

/// Node to read pcd files, transform to ndt maps and publish the resulting maps in PointCloud2
/// format
class NDT_NODES_PUBLIC NDTMapPublisherNode : public rclcpp::Node
{
public:
  using SerializedMap = ndt::StaticNDTMap;
  using MapConfig = perception::filters::voxel_grid::Config;
  using VoxelGrid = perception::filters::voxel_grid_nodes::algorithm::VoxelCloudCentroid;
  /// \brief Parameter constructor
  /// \param node_options Additional options to control creation of the node.
  explicit NDTMapPublisherNode(
    const rclcpp::NodeOptions & node_options
  );

  /// Run the publisher. Following actions are executed in order:
  /// 1. Wait until the map publisher matches the configured number of subscribers. If the
  /// configured timeout occurs, it throws an exception.
  /// 2. Load the PCD file into a PointCloud2 message.
  /// 3. Apply the normal distribution transform loaded PointCloud2 message.
  /// 4. Convert the resulting map representation into a `PointCloud2` message and publish.
  void run();

private:
  /// Initialize and allocate memory for the point clouds that are used as intermediate
  /// representations during  conversions. & setup visualization publisher if required
  /// \param map_frame Frame of the map
  /// \param map_topic Topic name for ndt map
  /// \param viz_map_topic Topic name for map visualization
  void init(
    const std::string & map_frame,
    const std::string & map_topic,
    const std::string & viz_map_topic);

  void publish_earth_to_map_transform(ndt::geocentric_pose_t pose);

  /// Publish the loaded map file. If no new map is loaded, it will publish the
  /// previous map, or an empty map.
  void publish();

  /// Reset the internal point clouds and the ndt map.
  void reset();

  /// Use a Voxel Grid filter to downsample the loaded map prior to publishing.
  void downsample_pc();

  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr m_pub_earth_map;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pub;
  std::unique_ptr<ndt::DynamicNDTMap> m_ndt_map_ptr;
  sensor_msgs::msg::PointCloud2 m_map_pc;
  sensor_msgs::msg::PointCloud2 m_source_pc;
  sensor_msgs::msg::PointCloud2 m_downsampled_pc;
  const std::string m_pcl_file_name;
  const std::string m_yaml_file_name;
  const bool8_t m_viz_map;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_viz_pub;
  std::unique_ptr<MapConfig> m_map_config_ptr;
  std::unique_ptr<MapConfig> m_viz_map_config_ptr;
  std::unique_ptr<VoxelGrid> m_voxelgrid_ptr;
  // Workaround. TODO(yunus.caliskan): Remove in #380
  rclcpp::TimerBase::SharedPtr m_visualization_timer{nullptr};
  rclcpp::TimerBase::SharedPtr m_transform_pub_timer{nullptr};
};

}  // namespace ndt_nodes
}  // namespace localization
}  // namespace autoware

#endif  // NDT_NODES__MAP_PUBLISHER_HPP_
