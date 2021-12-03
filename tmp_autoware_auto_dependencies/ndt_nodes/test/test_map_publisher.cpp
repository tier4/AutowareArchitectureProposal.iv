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

#include <gtest/gtest.h>
#include <ndt_nodes/map_publisher.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "test_map_publisher.hpp"

using autoware::localization::ndt::read_from_pcd;
using autoware::localization::ndt::geodetic_pose_t;
using autoware::localization::ndt_nodes::NDTMapPublisherNode;
using autoware::localization::ndt::DynamicNDTMap;
using autoware::localization::ndt::StaticNDTMap;
using autoware::localization::ndt::Real;
using autoware::perception::filters::voxel_grid::Config;

std::string build_yaml_string()
{
  YAML::Emitter yaml_out;
  yaml_out << YAML::Comment("test_map.yaml");
  yaml_out << YAML::BeginMap;
  yaml_out << YAML::Key << "map_config";
  yaml_out << YAML::Value << YAML::BeginMap;
  yaml_out << YAML::Key << "latitude" << YAML::Value << 0.0;
  yaml_out << YAML::Key << "longitude" << YAML::Value << 0.0;
  yaml_out << YAML::Key << "elevation" << YAML::Value << 0.0;
  yaml_out << YAML::EndMap;
  yaml_out << YAML::EndMap;
  std::string yaml_string(yaml_out.c_str());
  return yaml_string;
}

TEST(PCDLoadTest, Basics) {
  constexpr auto num_points = 5U;
  pcl::PointCloud<pcl::PointXYZI> dummy_cloud{};
  sensor_msgs::msg::PointCloud2 msg;
  using autoware::common::types::PointXYZI;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> msg_modifier{msg, "base_link"};
  const std::string test_fname = "PCDLoadTest_test_pcd_file.pcd";
  const std::string non_existing_fname = "NON_EXISTING_FILE_PCDLoadTest.XYZ";

  for (auto i = 0U; i < num_points; i++) {
    pcl::PointXYZI pt;
    pt.x = static_cast<float32_t>(i);
    pt.y = static_cast<float32_t>(i);
    pt.z = static_cast<float32_t>(i);
    pt.intensity = static_cast<float32_t>(i);
    dummy_cloud.push_back(pt);
  }
  pcl::io::savePCDFile(test_fname, dummy_cloud);

  ASSERT_TRUE(std::ifstream{test_fname}.good());
  ASSERT_FALSE(std::ifstream{non_existing_fname}.good());
  EXPECT_THROW(read_from_pcd(non_existing_fname, &msg), std::runtime_error);
  EXPECT_NO_THROW(read_from_pcd(test_fname, &msg));

  point_cloud_msg_wrapper::PointCloud2View<PointXYZI> msg_view{msg};
  auto counter = 0.0F;

  for (const auto & pt : msg_view) {
    EXPECT_FLOAT_EQ(pt.x, counter);
    EXPECT_FLOAT_EQ(pt.y, counter);
    EXPECT_FLOAT_EQ(pt.z, counter);
    EXPECT_FLOAT_EQ(pt.intensity, counter);
    counter += 1.0F;
  }
  EXPECT_FLOAT_EQ(counter, num_points);
  remove(test_fname.c_str());
}

TEST(YamlLoadTest, Basics) {
  const std::string test_fname = "YamlLoadTest_test_yaml_file.yaml";
  const std::string non_existing_fname = "NON_EXISTING_FILE_yamlLoadTest.XYZ";
  const std::string test_pcd_fname = "test.pcd";

  std::string yaml_string = build_yaml_string();

  std::ofstream yaml_fout(test_fname);
  yaml_fout << yaml_string << std::endl;
  yaml_fout.close();

  geodetic_pose_t gp{0, 0, 0, 0, 0, 0};
  ASSERT_TRUE(std::ifstream{test_fname}.good());
  ASSERT_FALSE(std::ifstream{non_existing_fname}.good());
  EXPECT_THROW(
    read_from_yaml(
      non_existing_fname, &gp),
    std::runtime_error);
  EXPECT_NO_THROW(
    read_from_yaml(
      test_fname, &gp));

  remove(test_fname.c_str());
}

TEST_F(MapPublisherTest, CoreFunctionality)
{
  using Cloud = sensor_msgs::msg::PointCloud2;
  const auto grid_config = Config(m_min_point, m_max_point, m_voxel_size, m_capacity);

  std::string yaml_file_name = "MapPublisherTest_test.yaml";
  const auto pcl_file_name = "MapPublisherTest_test.pcd";
  std::string yaml_string = build_yaml_string();

  std::ofstream yaml_fout(yaml_file_name);
  yaml_fout << yaml_string << std::endl;
  yaml_fout.close();

  ASSERT_TRUE(std::ifstream{yaml_file_name}.good());

  // have a validation map to transform the source cloud. The publisher's output
  // should match the cells in this map.
  DynamicNDTMap dynamic_validation_map(grid_config);
  StaticNDTMap static_received_map{};
  const auto map_topic = "ndt_map";
  const auto map_frame = "map";
  auto callback_counter = 0U;
  Cloud received_cloud_map;
  const auto listener_node = rclcpp::Node::make_shared("MapPublisherTest_listener_node");
  const auto sub =
    listener_node->create_subscription<Cloud>(
    map_topic,
    rclcpp::QoS(rclcpp::KeepLast(5U)).transient_local(),
    [&received_cloud_map, &callback_counter](Cloud::ConstSharedPtr msg) {
      ++callback_counter;
      received_cloud_map = *msg;
    });

  // Create map publisher. It is given 5 seconds to discover the test subscription.

  std::vector<rclcpp::Parameter> params;

  params.emplace_back("map_pcd_file", pcl_file_name);
  params.emplace_back("map_yaml_file", yaml_file_name);
  params.emplace_back("map_frame", map_frame);

  params.emplace_back(
    "map_config.min_point.x",
    static_cast<float32_t>(grid_config.get_min_point().x));
  params.emplace_back(
    "map_config.min_point.y",
    static_cast<float32_t>(grid_config.get_min_point().y));
  params.emplace_back(
    "map_config.min_point.z",
    static_cast<float32_t>(grid_config.get_min_point().z));
  params.emplace_back(
    "map_config.max_point.x",
    static_cast<float32_t>(grid_config.get_max_point().x));
  params.emplace_back(
    "map_config.max_point.y",
    static_cast<float32_t>(grid_config.get_max_point().y));
  params.emplace_back(
    "map_config.max_point.z",
    static_cast<float32_t>(grid_config.get_max_point().z));
  params.emplace_back(
    "map_config.voxel_size.x",
    static_cast<float32_t>(grid_config.get_voxel_size().x));
  params.emplace_back(
    "map_config.voxel_size.y",
    static_cast<float32_t>(grid_config.get_voxel_size().y));
  params.emplace_back(
    "map_config.voxel_size.z",
    static_cast<float32_t>(grid_config.get_voxel_size().z));
  params.emplace_back("map_config.capacity", static_cast<int32_t>(grid_config.get_capacity()));

  // Build a dense PC that can be transformed into 125 cells. See function for details.
  build_pc(grid_config);

  // Set up the validation grid.
  EXPECT_EQ(m_voxel_centers.size(), 125U);
  dynamic_validation_map.insert(m_pc);
  EXPECT_EQ(dynamic_validation_map.size(), m_voxel_centers.size());

  // Save the dense map to be read.
  const auto pcl_source = from_pointcloud2(m_pc);
  pcl::io::savePCDFile(pcl_file_name, pcl_source);
  ASSERT_TRUE(std::ifstream{pcl_file_name}.good());

  // Read pcd, pass the cloud to the internal dynamic map.
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  std::shared_ptr<NDTMapPublisherNode> map_publisher;
  EXPECT_NO_THROW(map_publisher = std::make_shared<NDTMapPublisherNode>(node_options));

  while (callback_counter < 1U) {
    rclcpp::spin_some(listener_node);
  }

  EXPECT_EQ(callback_counter, 1U);
  // Check that received pointcloud is a valid ndt map in terms of meta information.
  // Insert to static map for easier iteration and access.
  static_received_map.set(received_cloud_map);
  EXPECT_EQ(static_received_map.size(), dynamic_validation_map.size());

  // m_voxel_centers contain the centroid locations in the dynamic map.
  // Iterate and lookup these centroids in the reference and the received map.
  for (const auto & expected_centroid_it : m_voxel_centers) {
    const auto expected_centroid = expected_centroid_it.second;
    const auto & received_cell = static_received_map.cell(expected_centroid)[0U];
    const auto & reference_cell = dynamic_validation_map.cell(expected_centroid)[0U];
    EXPECT_TRUE(
      received_cell.centroid().isApprox(
        reference_cell.centroid(),
        std::numeric_limits<Real>::epsilon()));
    EXPECT_TRUE(
      received_cell.inverse_covariance().isApprox(
        reference_cell.inverse_covariance(),
        std::numeric_limits<Real>::epsilon() * 1e2));
  }
  remove(pcl_file_name);
  remove(yaml_file_name.c_str());
}

TEST_F(MapPublisherTest, VizFunctionality)
{
  using Cloud = sensor_msgs::msg::PointCloud2;
  const auto grid_config = Config(m_min_point, m_max_point, m_voxel_size, m_capacity);

  std::string yaml_file_name = "MapPublisherTest_test.yaml";
  const auto pcl_file_name = "MapPublisherTest_test.pcd";
  // have a validation source cloud. The viz cloud publisher's output
  // should match the cells in this map.
  const auto map_topic = "map_topic";
  const auto viz_map_topic = "viz_ndt_map";
  const auto map_frame = "map";
  auto callback_counter = 0U;
  auto viz_callback_counter = 0U;
  Cloud received_cloud_map;
  Cloud received_viz_cloud_map;
  const auto listener_node = rclcpp::Node::make_shared("MapPublisherTest_listener_node");
  const auto sub =
    listener_node->create_subscription<Cloud>(
    map_topic,
    rclcpp::QoS(rclcpp::KeepLast(5U)).transient_local(),
    [&received_cloud_map, &callback_counter](Cloud::ConstSharedPtr msg) {
      ++callback_counter;
      received_cloud_map = *msg;
    });
  const auto viz_listener_node = rclcpp::Node::make_shared("MapPublisherTest_viz_listener_node");
  const auto viz_sub =
    viz_listener_node->create_subscription<Cloud>(
    viz_map_topic,
    rclcpp::QoS(rclcpp::KeepLast(5U)).transient_local(),
    [&received_viz_cloud_map, &viz_callback_counter](Cloud::ConstSharedPtr msg) {
      ++viz_callback_counter;
      received_viz_cloud_map = *msg;
    });

  std::string yaml_string = build_yaml_string();
  std::ofstream yaml_fout(yaml_file_name);
  yaml_fout << yaml_string << std::endl;
  yaml_fout.close();

  ASSERT_TRUE(std::ifstream{yaml_file_name}.good());

  // Create map publisher. It is given 5 seconds to discover the test subscription.
  std::vector<rclcpp::Parameter> params;

  params.emplace_back("map_pcd_file", pcl_file_name);
  params.emplace_back("map_yaml_file", yaml_file_name);
  params.emplace_back("map_frame", map_frame);
  params.emplace_back("viz_map", true);

  params.emplace_back(
    "map_config.min_point.x",
    static_cast<float32_t>(grid_config.get_min_point().x));
  params.emplace_back(
    "map_config.min_point.y",
    static_cast<float32_t>(grid_config.get_min_point().y));
  params.emplace_back(
    "map_config.min_point.z",
    static_cast<float32_t>(grid_config.get_min_point().z));
  params.emplace_back(
    "map_config.max_point.x",
    static_cast<float32_t>(grid_config.get_max_point().x));
  params.emplace_back(
    "map_config.max_point.y",
    static_cast<float32_t>(grid_config.get_max_point().y));
  params.emplace_back(
    "map_config.max_point.z",
    static_cast<float32_t>(grid_config.get_max_point().z));
  params.emplace_back(
    "map_config.voxel_size.x",
    static_cast<float32_t>(grid_config.get_voxel_size().x));
  params.emplace_back(
    "map_config.voxel_size.y",
    static_cast<float32_t>(grid_config.get_voxel_size().y));
  params.emplace_back(
    "map_config.voxel_size.z",
    static_cast<float32_t>(grid_config.get_voxel_size().z));
  params.emplace_back("map_config.capacity", static_cast<int32_t>(grid_config.get_capacity()));

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  std::shared_ptr<NDTMapPublisherNode> map_publisher_ptr;

  // Build a dense PC that can be transformed into 125 cells. See function for details.
  build_pc(grid_config);

  // Save the dense map to be read.
  const auto pcl_source = from_pointcloud2(m_pc);
  pcl::io::savePCDFile(pcl_file_name, pcl_source);
  ASSERT_TRUE(std::ifstream{pcl_file_name}.good());

  // Read pcd, pass the cloud to the internal dynamic map.
//  EXPECT_NO_THROW(map_publisher_ptr = std::make_shared<NDTMapPublisherNode>(node_options));
  map_publisher_ptr = std::make_shared<NDTMapPublisherNode>(node_options);
  while (viz_callback_counter < 1U) {
    rclcpp::spin_some(map_publisher_ptr);  // TODO(yunus.caliskan): Remove spinning in #380
    rclcpp::spin_some(listener_node);
    rclcpp::spin_some(viz_listener_node);
  }

  EXPECT_EQ(viz_callback_counter, 1U);
  // Check that received viz pointcloud is a valid ndt map in terms of meta information.
  EXPECT_EQ(received_viz_cloud_map.width, 650U);

  remove(pcl_file_name);
  remove(yaml_file_name.c_str());
}
