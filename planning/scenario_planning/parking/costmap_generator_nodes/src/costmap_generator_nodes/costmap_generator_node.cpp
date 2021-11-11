// Copyright 2021 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

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

#include "costmap_generator_nodes/costmap_generator_node.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "had_map_utils/had_map_conversion.hpp"
#include "had_map_utils/had_map_query.hpp"
#include "had_map_utils/had_map_visualization.hpp"
#include "tf2/utils.h"

// taken from mapping/had_map/lanelet2_map_provider/src/lanelet2_map_visualizer.cpp
void insertMarkerArray(
  visualization_msgs::msg::MarkerArray & a1, const visualization_msgs::msg::MarkerArray & a2)
{
  if (!a2.markers.empty()) {
    a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
  }
}

// taken from mapping/had_map/lanelet2_map_provider/src/lanelet2_map_visualizer.cpp
visualization_msgs::msg::MarkerArray createLaneletVisualization(
  std::shared_ptr<lanelet::LaneletMap> & map)
{
  auto lls = autoware::common::had_map_utils::getConstLaneletLayer(map);

  std_msgs::msg::ColorRGBA color_lane_bounds, color_parking_bounds, color_parking_access_bounds,
    color_geom_bounds, color_lanelets, color_parking, color_parking_access, color_pickup_dropoff;
  autoware::common::had_map_utils::setColor(&color_lane_bounds, 1.0f, 1.0f, 1.0f, 1.0f);
  autoware::common::had_map_utils::setColor(&color_parking_bounds, 1.0f, 1.0f, 1.0f, 1.0f);
  autoware::common::had_map_utils::setColor(&color_parking_access_bounds, 1.0f, 1.0f, 1.0f, 1.0f);
  autoware::common::had_map_utils::setColor(&color_geom_bounds, 0.0f, 0.0f, 1.0f, 1.0f);
  autoware::common::had_map_utils::setColor(&color_lanelets, 0.2f, 0.5f, 0.6f, 0.6f);
  autoware::common::had_map_utils::setColor(&color_parking, 0.3f, 0.3f, 0.7f, 0.5f);
  autoware::common::had_map_utils::setColor(&color_parking_access, 0.3f, 0.7f, 0.3f, 0.5f);
  autoware::common::had_map_utils::setColor(&color_pickup_dropoff, 0.9f, 0.2f, 0.1f, 0.7f);

  visualization_msgs::msg::MarkerArray map_marker_array;

  rclcpp::Time marker_t = rclcpp::Time(0);
  insertMarkerArray(
    map_marker_array, autoware::common::had_map_utils::laneletsBoundaryAsMarkerArray(
      marker_t, lls, color_lane_bounds, true));
  insertMarkerArray(
    map_marker_array, autoware::common::had_map_utils::laneletsAsTriangleMarkerArray(
      marker_t, "lanelet_triangles", lls, color_lanelets));

  // for parking spots defined as areas (LaneletOSM definition)
  auto ll_areas = autoware::common::had_map_utils::getAreaLayer(map);
  auto ll_parking_areas = autoware::common::had_map_utils::subtypeAreas(ll_areas, "parking_spot");
  auto ll_parking_access_areas =
    autoware::common::had_map_utils::subtypeAreas(ll_areas, "parking_access");

  insertMarkerArray(
    map_marker_array, autoware::common::had_map_utils::areasBoundaryAsMarkerArray(
      marker_t, "parking_area_bounds", ll_parking_areas, color_parking_bounds));
  insertMarkerArray(
    map_marker_array,
    autoware::common::had_map_utils::areasBoundaryAsMarkerArray(
      marker_t, "parking_access_area_bounds", ll_parking_access_areas, color_parking_bounds));
  insertMarkerArray(
    map_marker_array, autoware::common::had_map_utils::areasAsTriangleMarkerArray(
      marker_t, "parking_area_triangles", ll_parking_areas, color_parking));
  insertMarkerArray(
    map_marker_array,
    autoware::common::had_map_utils::areasAsTriangleMarkerArray(
      marker_t, "parking_access_area_triangles", ll_parking_access_areas, color_parking_access));

  return map_marker_array;
}

nav_msgs::msg::OccupancyGrid createOccupancyGrid(
  const grid_map::GridMap & costmap, const std::string & layer_name,
  autoware::planning::costmap_generator::CostmapGeneratorParams & costmap_params,
  rclcpp::Clock::SharedPtr clock)
{
  nav_msgs::msg::OccupancyGrid occupancy_grid;

  // Convert to OccupancyGrid
  grid_map::GridMapRosConverter::toOccupancyGrid(
    costmap, layer_name, static_cast<float>(costmap_params.grid_min_value),
    static_cast<float>(costmap_params.grid_max_value), occupancy_grid);

  // Set header
  std_msgs::msg::Header header;
  header.frame_id = costmap_params.costmap_frame;
  header.stamp = clock->now();
  occupancy_grid.header = header;

  return occupancy_grid;
}

namespace autoware
{
namespace planning
{
namespace costmap_generator
{
CostmapGeneratorNode::CostmapGeneratorNode(const rclcpp::NodeOptions & node_options)
: Node("costmap_generator_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Node Parameters
  vehicle_frame_ = this->declare_parameter<std::string>("vehicle_frame", "base_link");
  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  route_box_padding_ = this->declare_parameter<double>("route_box_padding_m", 10.0);

  // Costmap Parameters
  costmap_params_.grid_min_value = this->declare_parameter<double>("grid_min_value", 0.0);
  costmap_params_.grid_max_value = this->declare_parameter<double>("grid_max_value", 1.0);
  costmap_params_.grid_resolution = this->declare_parameter<double>("grid_resolution", 0.2);
  costmap_params_.grid_length_x = this->declare_parameter<double>("grid_length_x", 50);
  costmap_params_.grid_length_y = this->declare_parameter<double>("grid_length_y", 50);
  costmap_params_.grid_position_x = this->declare_parameter<double>("grid_position_x", 0);
  costmap_params_.grid_position_y = this->declare_parameter<double>("grid_position_y", 0);
  costmap_params_.use_wayarea = this->declare_parameter<bool>("use_wayarea", true);
  costmap_params_.bound_costmap = this->declare_parameter<bool>("bound_costmap", true);
  costmap_params_.costmap_frame = this->declare_parameter<std::string>("costmap_frame", "map");

  // Initialize costmap generator
  costmap_generator_ = std::make_unique<CostmapGenerator>(costmap_params_);

  // Wait for first tf
  // We want to do this before creating subscriptions
  while (rclcpp::ok()) {
    try {
      tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      break;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Waiting for initial pose...");
    }
    rclcpp::sleep_for(std::chrono::seconds(5));
  }

  // Setup Map Service
  map_client_ =
    this->create_client<autoware_auto_msgs::srv::HADMapService>("~/client/HAD_Map_Service");

  while (!map_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for HAD map server.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for HAD map service...");
  }

  // Publishers
  debug_occupancy_grid_publisher_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/occupancy_grid", 1);
  debug_local_had_map_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/local_had_map", 1);

  // Action server
  costmap_action_server_ = rclcpp_action::create_server<autoware_auto_msgs::action::PlannerCostmap>(
    this->get_node_base_interface(), this->get_node_clock_interface(),
    this->get_node_logging_interface(), this->get_node_waitables_interface(), "generate_costmap",
    [this](auto uuid, auto goal) {return this->handleGoal(uuid, goal);},
    [this](auto goal_handle) {return this->handleCancel(goal_handle);},
    [this](auto goal_handle) {return this->handleAccepted(goal_handle);});
}

rclcpp_action::GoalResponse CostmapGeneratorNode::handleGoal(
  const rclcpp_action::GoalUUID &,
  const std::shared_ptr<const autoware_auto_msgs::action::PlannerCostmap::Goal>)
{
  if (!isIdle()) {
    RCLCPP_WARN(get_logger(), "Costmap generator is not in idle. Rejecting new request.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Received new costmap request.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CostmapGeneratorNode::handleCancel(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<autoware_auto_msgs::action::PlannerCostmap>>)
{
  RCLCPP_WARN(get_logger(), "Received action cancellation, rejecting.");
  return rclcpp_action::CancelResponse::REJECT;
}

void CostmapGeneratorNode::handleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<autoware_auto_msgs::action::PlannerCostmap>>
  goal_handle)
{
  setGeneratingState();

  goal_handle_ = goal_handle;

  auto map_request = std::make_shared<autoware_auto_msgs::srv::HADMapService::Request>(
    createMapRequest(goal_handle_->get_goal()->route));

  // TODO(kielczykowski-rai): replace it with synchronized implementation
  auto result = map_client_->async_send_request(
    map_request, std::bind(&CostmapGeneratorNode::mapResponse, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Requested HAD map.");
}

autoware_auto_msgs::srv::HADMapService::Request CostmapGeneratorNode::createMapRequest(
  const autoware_auto_msgs::msg::HADMapRoute & route) const
{
  auto request = autoware_auto_msgs::srv::HADMapService::Request();

  request.requested_primitives.push_back(
    autoware_auto_msgs::srv::HADMapService_Request::DRIVEABLE_GEOMETRY);

  // x
  request.geom_upper_bound.push_back(
    std::fmax(route.start_point.position.x, route.goal_point.position.x) + route_box_padding_);
  // y
  request.geom_upper_bound.push_back(
    std::fmax(route.start_point.position.y, route.goal_point.position.y) + route_box_padding_);
  // z (ignored)
  request.geom_upper_bound.push_back(0.0);

  // x
  request.geom_lower_bound.push_back(
    std::fmin(route.start_point.position.x, route.goal_point.position.x) - route_box_padding_);
  // y
  request.geom_lower_bound.push_back(
    std::fmin(route.start_point.position.y, route.goal_point.position.y) - route_box_padding_);
  // z (ignored)
  request.geom_lower_bound.push_back(0.0);

  return request;
}

grid_map::Position CostmapGeneratorNode::getCostmapToVehicleTranslation()
{
  // Get current pose
  geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(
    costmap_params_.costmap_frame, vehicle_frame_, rclcpp::Time(0),
    rclcpp::Duration::from_seconds(1.0));

  return grid_map::Position(tf.transform.translation.x, tf.transform.translation.y);
}

void CostmapGeneratorNode::mapResponse(
  rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedFuture future)
{
  RCLCPP_INFO(get_logger(), "Received HAD map.");

  // Create Lanelet2 map
  auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  autoware::common::had_map_utils::fromBinaryMsg(future.get()->map, lanelet_map_ptr);

  // Find translation from grid map to robots center position
  grid_map::Position vehicle_to_grid_position;
  try {
    vehicle_to_grid_position = getCostmapToVehicleTranslation();
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Setting costmap position failure: %s", ex.what());
    return;
  }

  // Get costmap to map transform
  geometry_msgs::msg::TransformStamped costmap_to_map_transform;
  try {
    costmap_to_map_transform = tf_buffer_.lookupTransform(
      costmap_params_.costmap_frame, map_frame_, rclcpp::Time(0),
      rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Map to costmmap transform lookup failure: %s", ex.what());
    return;
  }

  // Generate costmap
  auto costmap = costmap_generator_->generateCostmap(
    lanelet_map_ptr, vehicle_to_grid_position, costmap_to_map_transform);

  // Create result
  auto result = std::make_shared<autoware_auto_msgs::action::PlannerCostmap::Result>();
  auto out_occupancy_grid =
    createOccupancyGrid(costmap, LayerName::COMBINED, costmap_params_, get_clock());
  result->costmap = out_occupancy_grid;

  // Publish visualizations
  auto map_marker_array = createLaneletVisualization(lanelet_map_ptr);
  debug_local_had_map_publisher_->publish(map_marker_array);
  debug_occupancy_grid_publisher_->publish(out_occupancy_grid);

  goal_handle_->succeed(result);

  RCLCPP_INFO(get_logger(), "Costmap generation succeeded.");

  setIdleState();
}

}  // namespace costmap_generator
}  // namespace planning
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::planning::costmap_generator::CostmapGeneratorNode)
