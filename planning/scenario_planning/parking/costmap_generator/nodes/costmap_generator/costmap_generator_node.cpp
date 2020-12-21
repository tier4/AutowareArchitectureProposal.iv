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

/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, private_node
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    private_node list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    private_node software without specific prior written permission.
 *
 *  private_node SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF private_node SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************/

#include "costmap_generator/costmap_generator.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_extension/utility/utilities.hpp"
#include "lanelet2_extension/visualization/visualization.hpp"

#include "costmap_generator/object_map_utils.hpp"

namespace
{
bool isActive(const autoware_planning_msgs::msg::Scenario::ConstSharedPtr scenario)
{
  if (!scenario) {
    return false;
  }

  const auto & s = scenario->activating_scenarios;
  if (
    std::find(std::begin(s), std::end(s), autoware_planning_msgs::msg::Scenario::PARKING) !=
    std::end(s))
  {
    return true;
  }

  return false;
}

// Convert from Point32 to Point
std::vector<geometry_msgs::msg::Point> poly2vector(const geometry_msgs::msg::Polygon & poly)
{
  std::vector<geometry_msgs::msg::Point> ps;
  for (const auto & p32 : poly.points) {
    geometry_msgs::msg::Point p;
    p.x = p32.x;
    p.y = p32.y;
    p.z = p32.z;
    ps.push_back(p);
  }
  return ps;
}

}  // namespace

CostmapGenerator::CostmapGenerator()
: Node("costmap_generator"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Parameters
  costmap_frame_ = this->declare_parameter<std::string>("costmap_frame", "map");
  vehicle_frame_ = this->declare_parameter<std::string>("vehicle_frame", "base_link");
  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  update_rate_ = this->declare_parameter<double>("update_rate", 10.0);
  grid_min_value_ = this->declare_parameter<double>("grid_min_value", 0.0);
  grid_max_value_ = this->declare_parameter<double>("grid_max_value", 1.0);
  grid_resolution_ = this->declare_parameter<double>("grid_resolution", 0.2);
  grid_length_x_ = this->declare_parameter<double>("grid_length_x", 50);
  grid_length_y_ = this->declare_parameter<double>("grid_length_y", 30);
  grid_position_x_ = this->declare_parameter<double>("grid_position_x", 20);
  grid_position_y_ = this->declare_parameter<double>("grid_position_y", 0);
  maximum_lidar_height_thres_ = this->declare_parameter<double>("maximum_lidar_height_thres", 0.3);
  minimum_lidar_height_thres_ = this->declare_parameter<double>("minimum_lidar_height_thres", -2.2);
  use_objects_ = this->declare_parameter<bool>("use_objects", true);
  use_points_ = this->declare_parameter<bool>("use_points", true);
  use_wayarea_ = this->declare_parameter<bool>("use_wayarea", true);
  expand_polygon_size_ = this->declare_parameter<double>("expand_polygon_size", 1.0);
  size_of_expansion_kernel_ = this->declare_parameter<int>("size_of_expansion_kernel", 9);

  // Wait for first tf
  // We want to do this before creating subscriptions
  while (rclcpp::ok()) {
    try {
      tf_buffer_.lookupTransform(map_frame_, vehicle_frame_, rclcpp::Time(0));
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "waiting for initial pose...");
    }
    rclcpp::sleep_for(std::chrono::milliseconds(5000));
  }


  // Subscribers
  using std::placeholders::_1;
  sub_objects_ = this->create_subscription<autoware_perception_msgs::msg::DynamicObjectArray>(
    "input/objects", 1, std::bind(&CostmapGenerator::onObjects, this, _1));
  sub_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input/points_no_ground", 1, std::bind(&CostmapGenerator::onPoints, this, _1));
  sub_lanelet_bin_map_ = this->create_subscription<autoware_lanelet2_msgs::msg::MapBin>(
    "input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&CostmapGenerator::onLaneletMapBin, this, _1));
  sub_scenario_ = this->create_subscription<autoware_planning_msgs::msg::Scenario>(
    "input/scenario", 1, std::bind(&CostmapGenerator::onScenario, this, _1));

  // Publishers
  pub_costmap_ = this->create_publisher<grid_map_msgs::msg::GridMap>("output/grid_map", 1);
  pub_occupancy_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "output/occupancy_grid", 1);

  // Timer
  auto timer_callback = std::bind(&CostmapGenerator::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(update_rate_));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);


  // Initialize
  initGridmap();
}

void CostmapGenerator::loadRoadAreasFromLaneletMap(
  const lanelet::LaneletMapPtr lanelet_map,
  std::vector<std::vector<geometry_msgs::msg::Point>> * area_points)
{
  // use all lanelets in map of subtype road to give way area
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map);
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);

  // convert lanelets to polygons and put into area_points array
  for (const auto & ll : road_lanelets) {
    geometry_msgs::msg::Polygon poly;
    lanelet::visualization::lanelet2Polygon(ll, &poly);
    area_points->push_back(poly2vector(poly));
  }
}

void CostmapGenerator::loadParkingAreasFromLaneletMap(
  const lanelet::LaneletMapPtr lanelet_map,
  std::vector<std::vector<geometry_msgs::msg::Point>> * area_points)
{
  // Parking lots
  lanelet::ConstPolygons3d all_parking_lots = lanelet::utils::query::getAllParkingLots(lanelet_map);
  for (const auto & ll_poly : all_parking_lots) {
    geometry_msgs::msg::Polygon poly;
    lanelet::utils::conversion::toGeomMsgPoly(ll_poly, &poly);
    area_points->push_back(poly2vector(poly));
  }

  // Parking spaces
  lanelet::ConstLineStrings3d all_parking_spaces =
    lanelet::utils::query::getAllParkingSpaces(lanelet_map);
  for (const auto & parking_space : all_parking_spaces) {
    lanelet::ConstPolygon3d ll_poly;
    lanelet::utils::lineStringWithWidthToPolygon(parking_space, &ll_poly);

    geometry_msgs::msg::Polygon poly;
    lanelet::utils::conversion::toGeomMsgPoly(ll_poly, &poly);
    area_points->push_back(poly2vector(poly));
  }
}

void CostmapGenerator::onLaneletMapBin(
  const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr msg)
{
  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_);

  if (use_wayarea_) {
    loadRoadAreasFromLaneletMap(lanelet_map_, &area_points_);
    loadParkingAreasFromLaneletMap(lanelet_map_, &area_points_);
  }
}

void CostmapGenerator::onObjects(
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr msg)
{
  objects_ = msg;
}

void CostmapGenerator::onPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  points_ = msg;
}

void CostmapGenerator::onScenario(const autoware_planning_msgs::msg::Scenario::ConstSharedPtr msg)
{
  scenario_ = msg;
}

void CostmapGenerator::onTimer()
{
  if (!isActive(scenario_)) {
    return;
  }

  // Get current pose
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf =
      tf_buffer_.lookupTransform(
      costmap_frame_, vehicle_frame_, rclcpp::Time(
        0), rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("Exception: "), "%s", ex.what());
    return;
  }

  // Set grid center
  grid_map::Position p;
  p.x() = tf.transform.translation.x;
  p.y() = tf.transform.translation.y;
  costmap_.setPosition(p);

  if (use_wayarea_ && lanelet_map_) {
    costmap_[LayerName::wayarea] = generateWayAreaCostmap();
  }

  if (use_objects_ && objects_) {
    costmap_[LayerName::objects] = generateObjectsCostmap(objects_);
  }

  if (use_points_ && points_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*points_, *points);
    costmap_[LayerName::points] = generatePointsCostmap(points);
  }

  costmap_[LayerName::combined] = generateCombinedCostmap();

  publishCostmap(costmap_);
}

void CostmapGenerator::initGridmap()
{
  costmap_.setFrameId(costmap_frame_);
  costmap_.setGeometry(
    grid_map::Length(grid_length_x_, grid_length_y_), grid_resolution_,
    grid_map::Position(grid_position_x_, grid_position_y_));

  costmap_.add(LayerName::points, grid_min_value_);
  costmap_.add(LayerName::objects, grid_min_value_);
  costmap_.add(LayerName::wayarea, grid_min_value_);
  costmap_.add(LayerName::combined, grid_min_value_);
}

grid_map::Matrix CostmapGenerator::generatePointsCostmap(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & in_points)
{
  grid_map::Matrix points_costmap = points2costmap_.makeCostmapFromPoints(
    maximum_lidar_height_thres_, minimum_lidar_height_thres_, grid_min_value_, grid_max_value_,
    costmap_, LayerName::points, in_points);
  return points_costmap;
}

autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr transformObjects(
  const tf2_ros::Buffer & tf_buffer,
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr in_objects,
  const std::string & target_frame_id, const std::string & src_frame_id)
{
  auto objects = new autoware_perception_msgs::msg::DynamicObjectArray();
  *objects = *in_objects;
  objects->header.frame_id = target_frame_id;

  geometry_msgs::msg::TransformStamped objects2costmap;
  try {
    objects2costmap =
      tf_buffer.lookupTransform(
      target_frame_id, src_frame_id, rclcpp::Time(
        0), rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("Exception: "), "%s", ex.what());
  }

  for (auto & object : objects->objects) {
    geometry_msgs::msg::PoseStamped output_stamped, input_stamped;
    input_stamped.pose = object.state.pose_covariance.pose;
    tf2::doTransform(input_stamped, output_stamped, objects2costmap);
    object.state.pose_covariance.pose = output_stamped.pose;
  }

  return autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr(objects);
}

grid_map::Matrix CostmapGenerator::generateObjectsCostmap(
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr in_objects)
{
  const auto object_frame = in_objects->header.frame_id;
  const auto transformed_objects =
    transformObjects(tf_buffer_, in_objects, costmap_frame_, object_frame);

  grid_map::Matrix objects_costmap = objects2costmap_.makeCostmapFromObjects(
    costmap_, expand_polygon_size_, size_of_expansion_kernel_, transformed_objects);

  return objects_costmap;
}

grid_map::Matrix CostmapGenerator::generateWayAreaCostmap()
{
  grid_map::GridMap lanelet2_costmap = costmap_;
  if (!area_points_.empty()) {
    object_map::FillPolygonAreas(
      lanelet2_costmap, area_points_, LayerName::wayarea, grid_max_value_, grid_min_value_,
      grid_min_value_, grid_max_value_, costmap_frame_, map_frame_, tf_buffer_);
  }
  return lanelet2_costmap[LayerName::wayarea];
}

grid_map::Matrix CostmapGenerator::generateCombinedCostmap()
{
  // assuming combined_costmap is calculated by element wise max operation
  grid_map::GridMap combined_costmap = costmap_;

  combined_costmap[LayerName::combined].setConstant(grid_min_value_);

  combined_costmap[LayerName::combined] =
    combined_costmap[LayerName::combined].cwiseMax(combined_costmap[LayerName::points]);

  combined_costmap[LayerName::combined] =
    combined_costmap[LayerName::combined].cwiseMax(combined_costmap[LayerName::wayarea]);

  combined_costmap[LayerName::combined] =
    combined_costmap[LayerName::combined].cwiseMax(combined_costmap[LayerName::objects]);

  return combined_costmap[LayerName::combined];
}

void CostmapGenerator::publishCostmap(const grid_map::GridMap & costmap)
{
  // Set header
  std_msgs::msg::Header header;
  header.frame_id = costmap_frame_;
  header.stamp = this->now();

  // Publish OccupancyGrid
  nav_msgs::msg::OccupancyGrid out_occupancy_grid;
  grid_map::GridMapRosConverter::toOccupancyGrid(
    costmap, LayerName::combined, grid_min_value_, grid_max_value_, out_occupancy_grid);
  out_occupancy_grid.header = header;
  pub_occupancy_grid_->publish(out_occupancy_grid);

  // Publish GridMap
  auto out_gridmap_msg = grid_map::GridMapRosConverter::toMessage(costmap);
  out_gridmap_msg->header = header;
  pub_costmap_->publish(*out_gridmap_msg);
}
