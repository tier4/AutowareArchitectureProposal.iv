// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "pointcloud_preprocessor/filter.hpp"
#include "pointcloud_preprocessor/vector_map_filter/lanelet2_map_filter_nodelet.hpp"

#include "boost/geometry/algorithms/convex_hull.hpp"
#include "boost/geometry/algorithms/intersects.hpp"

#include "lanelet2_core/geometry/Polygon.h"
#include "pcl_ros/transforms.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace pointcloud_preprocessor
{
Lanelet2MapFilterComponent::Lanelet2MapFilterComponent(const rclcpp::NodeOptions & node_options)
: Node("LaneletMapFilter", node_options)
{
  using std::placeholders::_1;

  // Set parameters
  {
    voxel_size_x_ = declare_parameter("voxel_size_x", 0.04);
    voxel_size_y_ = declare_parameter("voxel_size_y", 0.04);
    voxel_size_z_ = declare_parameter("voxel_size_z", 0.04);
  }

  // Set publisher
  {
    filtered_pointcloud_pub_ = this->create_publisher<PointCloud2>("output", 1);
  }

  // Set subscriber
  {
    map_sub_ = this->create_subscription<autoware_lanelet2_msgs::msg::MapBin>(
      "input/vector_map", rclcpp::QoS{1}.transient_local(),
      std::bind(&Lanelet2MapFilterComponent::mapCallback, this, _1));
    pointcloud_sub_ = this->create_subscription<PointCloud2>(
      "input/pointcloud", 1, std::bind(&Lanelet2MapFilterComponent::pointcloudCallback, this, _1));
  }

  // Set parameter reconfigure
  {
    set_param_res_ = this->add_on_set_parameters_callback(
      std::bind(&Lanelet2MapFilterComponent::paramCallback, this, _1));
  }

  // Set tf
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
}

rcl_interfaces::msg::SetParametersResult Lanelet2MapFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  if (get_param(p, "voxel_size_x", voxel_size_x_)) {
    RCLCPP_DEBUG(get_logger(), "Setting voxel_size_x to: %f.", voxel_size_x_);
  }

  if (get_param(p, "voxel_size_y", voxel_size_y_)) {
    RCLCPP_DEBUG(get_logger(), "Setting voxel_size_y to: %f.", voxel_size_y_);
  }

  if (get_param(p, "voxel_size_x", voxel_size_z_)) {
    RCLCPP_DEBUG(get_logger(), "Setting voxel_size_x to: %f.", voxel_size_z_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

bool Lanelet2MapFilterComponent::transformPointCloud(
  const std::string & in_target_frame, const PointCloud2ConstPtr & in_cloud_ptr,
  PointCloud2 * out_cloud_ptr)
{
  if (in_target_frame == in_cloud_ptr->header.frame_id) {
    *out_cloud_ptr = *in_cloud_ptr;
    return true;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      in_target_frame, in_cloud_ptr->header.frame_id, in_cloud_ptr->header.stamp,
      rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return false;
  }
  // tf2::doTransform(*in_cloud_ptr, *out_cloud_ptr, transform_stamped);
  Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(mat, *in_cloud_ptr, *out_cloud_ptr);
  out_cloud_ptr->header.frame_id = in_target_frame;
  return true;
}

LinearRing2d Lanelet2MapFilterComponent::getConvexHull(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud)
{
  // downsample pointcloud to reduce convex hull calculation cost
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  downsampled_cloud->points.reserve(input_cloud->points.size());
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input_cloud);
  filter.setLeafSize(0.5, 0.5, 100.0);
  filter.filter(*downsampled_cloud);

  MultiPoint2d candidate_points;
  for (const auto & p : downsampled_cloud->points) {
    candidate_points.emplace_back(p.x, p.y);
  }

  LinearRing2d convex_hull;
  boost::geometry::convex_hull(candidate_points, convex_hull);

  return convex_hull;
}

lanelet::ConstLanelets Lanelet2MapFilterComponent::getIntersectedLanelets(
  const LinearRing2d & convex_hull, const lanelet::ConstLanelets & road_lanelets)
{
  lanelet::ConstLanelets intersected_lanelets;
  for (const auto & road_lanelet : road_lanelets) {
    if (boost::geometry::intersects(convex_hull, road_lanelet.polygon2d().basicPolygon())) {
      intersected_lanelets.push_back(road_lanelet);
    }
  }
  return intersected_lanelets;
}

bool Lanelet2MapFilterComponent::pointWithinLanelets(
  const Point2d & point, const lanelet::ConstLanelets & intersected_lanelets)
{
  for (const auto & lanelet : intersected_lanelets) {
    if (boost::geometry::within(point, lanelet.polygon2d().basicPolygon())) {
      return true;
    }
  }
  return false;
}

pcl::PointCloud<pcl::PointXYZ> Lanelet2MapFilterComponent::getLaneFilteredPointCloud(
  const lanelet::ConstLanelets & intersected_lanelets,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

  filtered_cloud.header = cloud->header;
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setLeafSize(voxel_size_x_, voxel_size_y_, 100000.0);
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setSaveLeafLayout(true);
  voxel_grid.filter(*downsampled_cloud);

  std::unordered_map<size_t, pcl::PointCloud<pcl::PointXYZ>> downsampled2original_map;
  for (const auto & p : cloud->points) {
    if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
      continue;
    }
    const int index = voxel_grid.getCentroidIndexAt(voxel_grid.getGridCoordinates(p.x, p.y, p.z));
    if (index == -1) {
      continue;
    }
    downsampled2original_map[index].points.push_back(p);
  }

  for (const auto & point : downsampled_cloud->points) {
    Point2d boost_point(point.x, point.y);
    if (pointWithinLanelets(boost_point, intersected_lanelets)) {
      const int index =
        voxel_grid.getCentroidIndexAt(voxel_grid.getGridCoordinates(point.x, point.y, point.z));
      if (index == -1) {
        continue;
      }
      for (const auto & original_point : downsampled2original_map[index].points) {
        filtered_cloud.points.push_back(original_point);
      }
    }
  }

  return filtered_cloud;
}

void Lanelet2MapFilterComponent::pointcloudCallback(const PointCloud2ConstPtr cloud_msg)
{
  if (!lanelet_map_ptr_) {
    return;
  }
  // transform pointcloud to map frame
  PointCloud2Ptr input_transed_cloud_ptr(new sensor_msgs::msg::PointCloud2);
  if (!transformPointCloud("map", cloud_msg, input_transed_cloud_ptr.get())) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(10000).count(),
      "Failed transform from " <<
        "map" <<
        " to " << cloud_msg->header.frame_id);
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_transed_cloud_ptr, *cloud);
  if (cloud->points.empty()) {
    return;
  }
  // calculate convex hull
  const auto convex_hull = getConvexHull(cloud);
  // get intersected lanelets
  lanelet::ConstLanelets intersected_lanelets = getIntersectedLanelets(convex_hull, road_lanelets_);
  // filter pointcloud by lanelet
  const auto filtered_cloud = getLaneFilteredPointCloud(intersected_lanelets, cloud);
  // transform pointcloud to input frame
  PointCloud2Ptr output_cloud_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(filtered_cloud, *output_cloud_ptr);
  PointCloud2Ptr output_transed_cloud_ptr(new sensor_msgs::msg::PointCloud2);
  if (!transformPointCloud(
      cloud_msg->header.frame_id, output_cloud_ptr, output_transed_cloud_ptr.get()))
  {
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(10000).count(),
      "Failed transform from " << cloud_msg->header.frame_id << " to " <<
        output_cloud_ptr->header.frame_id);
    return;
  }
  filtered_pointcloud_pub_->publish(*output_transed_cloud_ptr);
}

void Lanelet2MapFilterComponent::mapCallback(
  const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);
  const lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
}

}  // namespace pointcloud_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::Lanelet2MapFilterComponent)
