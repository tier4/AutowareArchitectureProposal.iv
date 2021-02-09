/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 *
 */

#include "rclcpp/rclcpp.hpp"

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_projection/UTM.h"
#include "autoware_lanelet2_msgs/msg/map_bin.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp"
#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_extension/visualization/visualization.hpp"

#include <vector>

static bool g_viz_lanelets_centerline = true;
static rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr g_map_pub;
static std::shared_ptr<rclcpp::Logger> g_logger;

void insertMarkerArray(
  visualization_msgs::msg::MarkerArray * a1, const visualization_msgs::msg::MarkerArray & a2)
{
  a1->markers.insert(a1->markers.end(), a2.markers.begin(), a2.markers.end());
}

void setColor(std_msgs::msg::ColorRGBA * cl, double r, double g, double b, double a)
{
  cl->r = r;
  cl->g = g;
  cl->b = b;
  cl->a = a;
}

void binMapCallback(const autoware_lanelet2_msgs::msg::MapBin::SharedPtr msg)
{
  lanelet::LaneletMapPtr viz_lanelet_map(new lanelet::LaneletMap);

  lanelet::utils::conversion::fromBinMsg(*msg, viz_lanelet_map);
  RCLCPP_INFO((*g_logger), "Map is loaded\n");

  // get lanelets etc to visualize
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(viz_lanelet_map);
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
  lanelet::ConstLanelets crosswalk_lanelets =
    lanelet::utils::query::crosswalkLanelets(all_lanelets);
  lanelet::ConstLineStrings3d pedestrian_markings =
    lanelet::utils::query::getAllPedestrianMarkings(viz_lanelet_map);
  lanelet::ConstLanelets walkway_lanelets = lanelet::utils::query::walkwayLanelets(all_lanelets);
  std::vector<lanelet::ConstLineString3d> stop_lines =
    lanelet::utils::query::stopLinesLanelets(road_lanelets);
  std::vector<lanelet::TrafficLightConstPtr> tl_reg_elems =
    lanelet::utils::query::trafficLights(all_lanelets);
  std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  std::vector<lanelet::DetectionAreaConstPtr> da_reg_elems =
    lanelet::utils::query::detectionAreas(all_lanelets);
  lanelet::ConstLineStrings3d parking_spaces =
    lanelet::utils::query::getAllParkingSpaces(viz_lanelet_map);
  lanelet::ConstPolygons3d parking_lots = lanelet::utils::query::getAllParkingLots(viz_lanelet_map);
  lanelet::ConstPolygons3d obstacle_polygons =
    lanelet::utils::query::getAllObstaclePolygons(viz_lanelet_map);

  std_msgs::msg::ColorRGBA cl_road, cl_cross, cl_pedestrian_markings, cl_ll_borders, cl_stoplines,
    cl_trafficlights, cl_detection_areas, cl_parking_lots, cl_parking_spaces, cl_lanelet_id,
    cl_obstacle_polygons;
  setColor(&cl_road, 0.27, 0.27, 0.27, 0.999);
  setColor(&cl_cross, 0.27, 0.3, 0.27, 0.5);
  setColor(&cl_pedestrian_markings, 0.5, 0.5, 0.5, 0.999);
  setColor(&cl_ll_borders, 0.5, 0.5, 0.5, 0.999);
  setColor(&cl_stoplines, 0.5, 0.5, 0.5, 0.999);
  setColor(&cl_trafficlights, 0.5, 0.5, 0.5, 0.8);
  setColor(&cl_detection_areas, 0.27, 0.27, 0.37, 0.5);
  setColor(&cl_obstacle_polygons, 0.4, 0.27, 0.27, 0.5);
  setColor(&cl_parking_lots, 0.5, 0.5, 0.0, 0.3);
  setColor(&cl_parking_spaces, 1.0, 0.647, 0.0, 0.6);
  setColor(&cl_lanelet_id, 0.5, 0.5, 0.5, 0.999);

  visualization_msgs::msg::MarkerArray map_marker_array;

  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::lineStringsAsMarkerArray(stop_lines, "stop_lines", cl_stoplines, 0.5));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::laneletDirectionAsMarkerArray(road_lanelets));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
      "crosswalk_lanelets", crosswalk_lanelets, cl_cross));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::pedestrianMarkingsAsMarkerArray(
                         pedestrian_markings, cl_pedestrian_markings));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
      "walkway_lanelets", walkway_lanelets, cl_cross));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::obstaclePolygonsAsMarkerArray(obstacle_polygons, cl_obstacle_polygons));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::detectionAreasAsMarkerArray(da_reg_elems, cl_detection_areas));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::parkingLotsAsMarkerArray(parking_lots, cl_parking_lots));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::parkingSpacesAsMarkerArray(parking_spaces, cl_parking_spaces));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::laneletsBoundaryAsMarkerArray(
                         road_lanelets, cl_ll_borders, g_viz_lanelets_centerline));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::autowareTrafficLightsAsMarkerArray(aw_tl_reg_elems, cl_trafficlights));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::generateLaneletIdMarker(road_lanelets, cl_lanelet_id));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("road_lanelets", road_lanelets, cl_road));

  g_map_pub->publish(map_marker_array);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("lanelet2_map_visualizer");
  auto bin_map_sub = node->create_subscription<autoware_lanelet2_msgs::msg::MapBin>(
    "input/lanelet2_map", rclcpp::QoS{1}.transient_local(), std::bind(&binMapCallback, std::placeholders::_1));

  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  g_map_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "output/lanelet2_map_marker", durable_qos);

  g_logger = std::make_shared<rclcpp::Logger>(node->get_logger());
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
