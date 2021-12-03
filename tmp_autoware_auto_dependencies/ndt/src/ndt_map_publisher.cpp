// Copyright 2020 the Autoware Foundation
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

#include <GeographicLib/Geocentric.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include <ndt/ndt_map_publisher.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <yaml-cpp/yaml.h>

#include <string>
#include <utility>

namespace autoware
{
namespace localization
{
namespace ndt
{

void read_from_yaml(
  const std::string & yaml_file_name,
  geodetic_pose_t * geo_pose)
{
  try {
    YAML::Node map_info = YAML::LoadFile(yaml_file_name);
    if (map_info["map_config"]) {
      if (map_info["map_config"]["latitude"] &&
        map_info["map_config"]["longitude"] &&
        map_info["map_config"]["elevation"])
      {
        geo_pose->latitude = map_info["map_config"]["latitude"].as<double>();
        geo_pose->longitude = map_info["map_config"]["longitude"].as<double>();
        geo_pose->elevation = map_info["map_config"]["elevation"].as<double>();
      } else {
        throw std::runtime_error("Yaml file: map origin not found\n");
      }
      if (map_info["map_config"]["roll"]) {
        geo_pose->roll = map_info["map_config"]["roll"].as<double>();
      }
      if (map_info["map_config"]["pitch"]) {
        geo_pose->pitch = map_info["map_config"]["pitch"].as<double>();
      }
      if (map_info["map_config"]["yaw"]) {
        geo_pose->yaw = map_info["map_config"]["yaw"].as<double>();
      }
    } else {
      throw std::runtime_error("Yaml file: map config not found\n");
    }
  } catch (const YAML::BadFile & ex) {
    throw std::runtime_error("Yaml file not found\n");
  } catch (const YAML::ParserException & ex) {
    throw std::runtime_error("Yaml syntax error\n");
  }
}

void read_from_pcd(const std::string & file_name, sensor_msgs::msg::PointCloud2 * msg)
{
  pcl::PCLPointCloud2 pcl_cloud;
  if (pcl::io::loadPCDFile(file_name, pcl_cloud) == -1) {  // load the file
    throw std::runtime_error(std::string("PCD file ") + file_name + " could not be loaded.");
  }
  if (pcl_cloud.data.size() == 0) {
    throw std::runtime_error("PCD cloud empty\n");
  }

  // Convert to sensor_msgs in order to check the available fields
  sensor_msgs::msg::PointCloud2 cloud;
  pcl_conversions::moveFromPCL(pcl_cloud, cloud);

  using autoware::common::types::float32_t;
  using autoware::common::types::PointXYZI;
  using point_cloud_msg_wrapper::PointCloud2View;
  using point_cloud_msg_wrapper::PointCloud2Modifier;
  if (PointCloud2View<PointXYZI>::can_be_created_from(cloud)) {
    // The cloud already has correct fields.
    cloud.header.frame_id = msg->header.frame_id;
    *msg = std::move(cloud);
    return;
  }

  struct PointWithoutIntensity
  {
    float32_t x;
    float32_t y;
    float32_t z;
  };

  if (PointCloud2View<PointWithoutIntensity>::can_be_created_from(cloud)) {
    sensor_msgs::msg::PointCloud2 adjusted_cloud;
    point_cloud_msg_wrapper::PointCloud2View<PointWithoutIntensity> old_cloud_view{cloud};
    point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> adjusted_cloud_modifier{
      adjusted_cloud, msg->header.frame_id};
    adjusted_cloud_modifier.reserve(old_cloud_view.size());

    for (const auto & old_point : old_cloud_view) {
      const PointXYZI point{
        old_point.x,
        old_point.y,
        old_point.z,
        0.0f};
      adjusted_cloud_modifier.push_back(point);
    }

    *msg = std::move(adjusted_cloud);
    return;
  }

  // We must pack this structure as this is how data is stored coming from the PCL conversion.
  #pragma pack(push, 1)
  struct PointWithUintIntensity
  {
    float32_t x;
    float32_t y;
    float32_t z;
    std::uint8_t intensity;
  };
  #pragma pack(pop)

  if (PointCloud2View<PointWithUintIntensity>::can_be_created_from(cloud)) {
    // We need to convert the intensity field.
    sensor_msgs::msg::PointCloud2 adjusted_cloud;
    point_cloud_msg_wrapper::PointCloud2View<PointWithUintIntensity> old_cloud_view{cloud};
    point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> adjusted_cloud_modifier{
      adjusted_cloud, msg->header.frame_id};
    adjusted_cloud_modifier.reserve(old_cloud_view.size());

    for (const auto & old_point : old_cloud_view) {
      const PointXYZI point{
        old_point.x,
        old_point.y,
        old_point.z,
        static_cast<float>(old_point.intensity)};
      adjusted_cloud_modifier.push_back(point);
    }

    *msg = std::move(adjusted_cloud);
    return;
  }

  throw std::runtime_error("intensity datatype is not float or uint8_t");
}

geocentric_pose_t load_map(
  const std::string & yaml_file_name,
  const std::string & pcl_file_name,
  sensor_msgs::msg::PointCloud2 & pc_out)
{
  using autoware::common::types::PointXYZI;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI>{pc_out}.clear();
  geodetic_pose_t geodetic_pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  if (!yaml_file_name.empty()) {
    read_from_yaml(yaml_file_name, &geodetic_pose);
  } else {
    throw std::runtime_error("YAML file name empty\n");
  }

  if (!pcl_file_name.empty()) {
    read_from_pcd(pcl_file_name, &pc_out);
  } else {
    throw std::runtime_error("PCD file name empty\n");
  }

  float64_t x(0.0), y(0.0), z(0.0);

  GeographicLib::Geocentric earth(
    GeographicLib::Constants::WGS84_a(),
    GeographicLib::Constants::WGS84_f());

  earth.Forward(
    geodetic_pose.latitude,
    geodetic_pose.longitude,
    geodetic_pose.elevation,
    x, y, z);

  return {x, y, z, geodetic_pose.roll, geodetic_pose.pitch, geodetic_pose.yaw};
}
}  // namespace ndt
}  // namespace localization
}  // namespace autoware
