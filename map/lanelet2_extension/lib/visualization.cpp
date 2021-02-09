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
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 */

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "Eigen/Eigen"

#include <algorithm>
#include <string>
#include <unordered_set>
#include <vector>

#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_extension/utility/utilities.hpp"
#include "lanelet2_extension/visualization/visualization.hpp"

namespace
{
template<typename T>
bool exists(const std::unordered_set<T> & set, const T & element)
{
  return std::find(set.begin(), set.end(), element) != set.end();
}

void adjacentPoints(
  const int i, const int N, const geometry_msgs::msg::Polygon poly,
  geometry_msgs::msg::Point32 * p0,
  geometry_msgs::msg::Point32 * p1, geometry_msgs::msg::Point32 * p2)
{
  if (p0 == nullptr || p1 == nullptr || p2 == nullptr) {
    std::cerr << __FUNCTION__ << ": either p0, p1, or p2 is null pointer!" << std::endl;
    return;
  }

  *p1 = poly.points[i];
  if (i == 0) {
    *p0 = poly.points[N - 1];
  } else {
    *p0 = poly.points[i - 1];
  }

  if (i < N - 1) {
    *p2 = poly.points[i + 1];
  } else {
    *p2 = poly.points[0];
  }
}

double hypot(const geometry_msgs::msg::Point32 & p0, const geometry_msgs::msg::Point32 & p1)
{
  return sqrt(pow((p1.x - p0.x), 2.0) + pow((p1.y - p0.y), 2.0));
}

bool isAttributeValue(
  const lanelet::ConstPoint3d p, const std::string attr_str, const std::string value_str)
{
  lanelet::Attribute attr = p.attribute(attr_str);
  if (attr.value().compare(value_str) == 0) {return true;}
  return false;
}

bool isLaneletAttributeValue(
  const lanelet::ConstLanelet ll, const std::string attr_str, const std::string value_str)
{
  lanelet::Attribute attr = ll.attribute(attr_str);
  if (attr.value().compare(value_str) == 0) {return true;}
  return false;
}

void lightAsMarker(
  lanelet::ConstPoint3d p, visualization_msgs::msg::Marker * marker, const std::string ns)
{
  if (marker == nullptr) {
    std::cerr << __FUNCTION__ << ": marker is null pointer!" << std::endl;
    return;
  }

  marker->header.frame_id = "map";
  marker->header.stamp = rclcpp::Time();
  marker->frame_locked - true;
  marker->ns = ns;
  marker->id = p.id();
  marker->lifetime = rclcpp::Duration(0, 0);
  marker->type = visualization_msgs::msg::Marker::SPHERE;
  marker->pose.position.x = p.x();
  marker->pose.position.y = p.y();
  marker->pose.position.z = p.z();
  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = 0.0;
  marker->pose.orientation.w = 1.0;

  float s = 0.3;

  marker->scale.x = s;
  marker->scale.y = s;
  marker->scale.z = s;

  marker->color.r = 0.0f;
  marker->color.g = 0.0f;
  marker->color.b = 0.0f;
  marker->color.a = 0.999f;

  if (isAttributeValue(p, "color", "red")) {
    marker->color.r = 1.0f;
  } else if (isAttributeValue(p, "color", "green")) {
    marker->color.g = 1.0f;
  } else if (isAttributeValue(p, "color", "yellow")) {
    marker->color.r = 1.0f;
    marker->color.g = 1.0f;
  } else {
    marker->color.r = 1.0f;
    marker->color.g = 1.0f;
    marker->color.b = 1.0f;
  }
}

void laneletDirectionAsMarker(
  const lanelet::ConstLanelet ll, visualization_msgs::msg::Marker * marker, const int id,
  const std::string ns)
{
  if (marker == nullptr) {
    std::cerr << __FUNCTION__ << ": marker is null pointer!" << std::endl;
    return;
  }

  marker->header.frame_id = "map";
  marker->header.stamp = rclcpp::Time();
  marker->frame_locked = true;
  marker->ns = ns;
  marker->id = id;
  marker->type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker->lifetime = rclcpp::Duration(0, 0);

  lanelet::BasicPoint3d pt[3];
  pt[0].x() = 0.0;
  pt[0].y() = -0.3;
  pt[0].z() = 0.0;
  pt[1].x() = 0.0;
  pt[1].y() = 0.3;
  pt[1].z() = 0;
  pt[2].x() = 1.0;
  pt[2].y() = 0.0;
  pt[2].z() = 0;

  lanelet::BasicPoint3d pc, pc2;

  lanelet::ConstLineString3d center_ls = ll.centerline();
  float s = 1.0;

  marker->pose.position.x = 0.0;  // p.x();
  marker->pose.position.y = 0.0;  // p.y();
  marker->pose.position.z = 0.0;  // p.z();
  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = 0.0;
  marker->pose.orientation.w = 1.0;
  marker->scale.x = s;
  marker->scale.y = s;
  marker->scale.z = s;
  marker->color.r = 1.0f;
  marker->color.g = 1.0f;
  marker->color.b = 1.0f;
  marker->color.a = 0.999;

  lanelet::Attribute attr = ll.attribute("turn_direction");
  double turn_dir = 0;

  std_msgs::msg::ColorRGBA c;
  c.r = 0.5;
  c.g = 0.5;
  c.b = 0.5;
  c.a = 0.5;

  if (isLaneletAttributeValue(ll, "turn_direction", "right")) {
    turn_dir = -M_PI / 2.0;
    c.r = 0.5;
    c.g = 0.5;
    c.b = 0.6;
  } else if (isLaneletAttributeValue(ll, "turn_direction", "left")) {
    turn_dir = M_PI / 2.0;
    c.r = 0.5;
    c.g = 0.6;
    c.b = 0.6;
  }

  for (int ci = 0; ci < center_ls.size() - 1; ) {
    pc = center_ls[ci];
    if (center_ls.size() > 1) {
      pc2 = center_ls[ci + 1];
    } else {
      return;
    }

    double heading = atan2(pc2.y() - pc.y(), pc2.x() - pc.x());

    lanelet::BasicPoint3d pt_tf[3];

    Eigen::Vector3d axis(0, 0, 1);
    Eigen::Transform<double, 3, Eigen::Affine> t(Eigen::AngleAxis<double>(heading, axis));

    for (int i = 0; i < 3; i++) {
      pt_tf[i] = t * pt[i] + pc;
    }

    geometry_msgs::msg::Point gp[3];

    for (int i = 0; i < 3; i++) {
      std_msgs::msg::ColorRGBA cn = c;

      gp[i].x = pt_tf[i].x();
      gp[i].y = pt_tf[i].y();
      gp[i].z = pt_tf[i].z();
      marker->points.push_back(gp[i]);
      marker->colors.push_back(cn);
    }
    ci = ci + 1;
  }
}

bool isClockWise(const geometry_msgs::msg::Polygon & polygon)
{
  const int N = polygon.points.size();
  const double x_offset = polygon.points[0].x;
  const double y_offset = polygon.points[0].y;
  double sum = 0.0;
  for (int i = 0; i < polygon.points.size(); ++i) {
    sum += (polygon.points[i].x - x_offset) * (polygon.points[(i + 1) % N].y - y_offset) -
           (polygon.points[i].y - y_offset) * (polygon.points[(i + 1) % N].x - x_offset);
  }

  return sum < 0.0;
}

// Is angle AOB less than 180?
// https://qiita.com/fujii-kotaro/items/a411f2a45627ed2f156e
bool isAcuteAngle(
  const geometry_msgs::msg::Point32 & a, const geometry_msgs::msg::Point32 & o,
  const geometry_msgs::msg::Point32 & b)
{
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x) >= 0;
}

// https://qiita.com/fujii-kotaro/items/a411f2a45627ed2f156e
bool isWithinTriangle(
  const geometry_msgs::msg::Point32 & a, const geometry_msgs::msg::Point32 & b,
  const geometry_msgs::msg::Point32 & c, const geometry_msgs::msg::Point32 & p)
{
  double c1 = (b.x - a.x) * (p.y - b.y) - (b.y - a.y) * (p.x - b.x);
  double c2 = (c.x - b.x) * (p.y - c.y) - (c.y - b.y) * (p.x - c.x);
  double c3 = (a.x - c.x) * (p.y - a.y) - (a.y - c.y) * (p.x - a.x);

  return c1 > 0.0 && c2 > 0.0 && c3 > 0.0 || c1 < 0.0 && c2 < 0.0 && c3 < 0.0;
}

visualization_msgs::msg::Marker polygonAsMarker(
  const lanelet::ConstPolygon3d & polygon, const std::string & name_space,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker marker;
  if (polygon.size() < 3) {
    return marker;
  }

  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Time();
  marker.frame_locked = true;
  marker.id = polygon.id();
  marker.ns = name_space;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.lifetime = rclcpp::Duration(0, 0);
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color = color;

  geometry_msgs::msg::Polygon geom_poly;
  lanelet::utils::conversion::toGeomMsgPoly(polygon, &geom_poly);

  std::vector<geometry_msgs::msg::Polygon> triangles;
  lanelet::visualization::polygon2Triangle(geom_poly, &triangles);

  marker.points.reserve(polygon.size() - 2);
  marker.colors.reserve(polygon.size() - 2);
  for (const auto & tri : triangles) {
    geometry_msgs::msg::Point geom_pts[3];
    for (int i = 0; i < 3; i++) {
      lanelet::utils::conversion::toGeomMsgPt(tri.points[i], &geom_pts[i]);
      marker.points.push_back(geom_pts[i]);
      marker.colors.push_back(color);
    }
  }
  return marker;
}

}  // anonymous namespace

namespace lanelet
{
void visualization::lanelet2Triangle(
  const lanelet::ConstLanelet & ll, std::vector<geometry_msgs::msg::Polygon> * triangles)
{
  if (triangles == nullptr) {
    std::cerr << __FUNCTION__ << ": triangles is null pointer!" << std::endl;
    return;
  }

  triangles->clear();
  geometry_msgs::msg::Polygon ll_poly;
  lanelet2Polygon(ll, &ll_poly);
  polygon2Triangle(ll_poly, triangles);
}

void visualization::polygon2Triangle(
  const geometry_msgs::msg::Polygon & polygon, std::vector<geometry_msgs::msg::Polygon> * triangles)
{
  geometry_msgs::msg::Polygon poly = polygon;
  if (!isClockWise(poly)) std::reverse(poly.points.begin(), poly.points.end());

  // ear clipping: find smallest internal angle in polygon
  int N = poly.points.size();

  // array of angles for each vertex
  std::vector<bool> is_acute_angle;
  is_acute_angle.assign(N, false);
  for (int i = 0; i < N; i++) {
    geometry_msgs::msg::Point32 p0, p1, p2;

    adjacentPoints(i, N, poly, &p0, &p1, &p2);
    is_acute_angle.at(i) = isAcuteAngle(p0, p1, p2);
  }

  // start ear clipping
  while (N >= 3) {
    int clipped_vertex = -1;

    for (int i = 0; i < N; i++) {
      bool theta = is_acute_angle.at(i);
      if (theta == true) {
        geometry_msgs::msg::Point32 p0, p1, p2;
        adjacentPoints(i, N, poly, &p0, &p1, &p2);

        int j_begin = (i + 2) % N;
        int j_end = (i - 1 + N) % N;
        bool is_ear = true;
        for (int j = j_begin; j != j_end; j = (j + 1) % N) {
          if (isWithinTriangle(p0, p1, p2, poly.points.at(j))) {
            is_ear = false;
            break;
          }
        }

        if (is_ear) {
          clipped_vertex = i;
          break;
        }
      }
    }
    if (clipped_vertex < 0 || clipped_vertex >= N) {
      // print in yellow to indicate warning
      std::cerr <<
        "\033[1;33mCould not find valid vertex for ear clipping triangulation. Triangulation result might be "
        "invalid\033[0m" << std::endl;
      clipped_vertex = 0;
    }

    // create triangle
    geometry_msgs::msg::Point32 p0, p1, p2;
    adjacentPoints(clipped_vertex, N, poly, &p0, &p1, &p2);
    geometry_msgs::msg::Polygon triangle;
    triangle.points.push_back(p0);
    triangle.points.push_back(p1);
    triangle.points.push_back(p2);
    triangles->push_back(triangle);

    // remove vertex of center of angle
    auto it = poly.points.begin();
    std::advance(it, clipped_vertex);
    poly.points.erase(it);

    // remove from angle list
    auto it_angle = is_acute_angle.begin();
    std::advance(it_angle, clipped_vertex);
    is_acute_angle.erase(it_angle);

    // update angle list
    N = poly.points.size();
    if (clipped_vertex == N) {
      clipped_vertex = 0;
    }
    adjacentPoints(clipped_vertex, N, poly, &p0, &p1, &p2);
    is_acute_angle.at(clipped_vertex) = isAcuteAngle(p0, p1, p2);

    int i_prev = (clipped_vertex == 0) ? N - 1 : clipped_vertex - 1;
    adjacentPoints(i_prev, N, poly, &p0, &p1, &p2);
    is_acute_angle.at(i_prev) = isAcuteAngle(p0, p1, p2);
  }
}

void visualization::lanelet2Polygon(
  const lanelet::ConstLanelet & ll, geometry_msgs::msg::Polygon * polygon)
{
  if (polygon == nullptr) {
    std::cerr << __FUNCTION__ << ": polygon is null pointer!" << std::endl;
    return;
  }

  lanelet::CompoundPolygon3d ll_poly = ll.polygon3d();

  polygon->points.clear();
  polygon->points.reserve(ll_poly.size());

  for (const auto & pt : ll_poly) {
    geometry_msgs::msg::Point32 pt32;
    utils::conversion::toGeomMsgPt32(pt.basicPoint(), &pt32);
    polygon->points.push_back(pt32);
  }
}

visualization_msgs::msg::MarkerArray visualization::laneletDirectionAsMarkerArray(
  const lanelet::ConstLanelets lanelets)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int ll_dir_count = 0;
  for (auto lli = lanelets.begin(); lli != lanelets.end(); lli++) {
    lanelet::ConstLanelet ll = *lli;

    //      bool road_found = false;
    if (ll.hasAttribute(std::string("turn_direction"))) {
      lanelet::Attribute attr = ll.attribute("turn_direction");
      visualization_msgs::msg::Marker marker;

      laneletDirectionAsMarker(ll, &marker, ll_dir_count, "lanelet direction");

      marker_array.markers.push_back(marker);
      ll_dir_count++;
    }
  }

  return marker_array;
}

visualization_msgs::msg::MarkerArray visualization::autowareTrafficLightsAsMarkerArray(
  const std::vector<lanelet::AutowareTrafficLightConstPtr> tl_reg_elems,
  const std_msgs::msg::ColorRGBA c, const rclcpp::Duration duration, const double scale)
{
  visualization_msgs::msg::MarkerArray tl_marker_array;

  int tl_count = 0;
  for (auto tli = tl_reg_elems.begin(); tli != tl_reg_elems.end(); tli++) {
    lanelet::ConstLineStrings3d light_bulbs;
    lanelet::AutowareTrafficLightConstPtr tl = *tli;

    const auto lights = tl->trafficLights();
    for (const auto & lsp : lights) {
      if (lsp.isLineString())  // traffic lights can either polygons or
      {                        // linestrings
        lanelet::ConstLineString3d ls = static_cast<lanelet::ConstLineString3d>(lsp);

        visualization_msgs::msg::Marker marker;
        visualization::trafficLight2TriangleMarker(
          ls, &marker, "traffic_light_triangle", c, duration, scale);
        tl_marker_array.markers.push_back(marker);
      }
    }

    light_bulbs = tl->lightBulbs();
    for (auto ls : light_bulbs) {
      lanelet::ConstLineString3d l = static_cast<lanelet::ConstLineString3d>(ls);

      for (auto pt : l) {
        if (pt.hasAttribute("color")) {
          visualization_msgs::msg::Marker marker;
          lightAsMarker(pt, &marker, "traffic_light");
          tl_marker_array.markers.push_back(marker);

          tl_count++;
        }
      }
    }
  }

  return tl_marker_array;
}

visualization_msgs::msg::MarkerArray visualization::detectionAreasAsMarkerArray(
  const std::vector<lanelet::DetectionAreaConstPtr> & da_reg_elems,
  const std_msgs::msg::ColorRGBA c,
  const rclcpp::Duration duration)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;

  if (da_reg_elems.empty()) {
    return marker_array;
  }

  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Time();
  marker.frame_locked = true;
  marker.ns = "detection_area";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.lifetime = duration;
  marker.pose.position.x = 0.0;  // p.x();
  marker.pose.position.y = 0.0;  // p.y();
  marker.pose.position.z = 0.0;  // p.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 0.999;

  int da_count = 0;
  for (const auto & da_reg_elem : da_reg_elems) {
    marker.points.clear();
    marker.colors.clear();
    marker.id = da_reg_elem->id();

    // area visualization
    const auto detection_areas = da_reg_elem->detectionAreas();
    for (const auto & detection_area : detection_areas) {
      geometry_msgs::msg::Polygon geom_poly;
      utils::conversion::toGeomMsgPoly(detection_area, &geom_poly);

      std::vector<geometry_msgs::msg::Polygon> triangles;
      polygon2Triangle(geom_poly, &triangles);

      for (auto tri : triangles) {
        geometry_msgs::msg::Point tri0[3];

        for (int i = 0; i < 3; i++) {
          utils::conversion::toGeomMsgPt(tri.points[i], &tri0[i]);
          marker.points.push_back(tri0[i]);
          marker.colors.push_back(c);
        }
      }  // for triangles0
    }    // for detection areas
    marker_array.markers.push_back(marker);

    // stop line visualization
    visualization_msgs::msg::Marker ls_marker;
    std_msgs::msg::ColorRGBA ls_c;
    ls_c.r = 0.5;
    ls_c.g = 0.5;
    ls_c.b = 0.5;
    ls_c.a = 0.999;
    visualization::lineString2Marker(
      da_reg_elem->stopLine(), &ls_marker, "map", "detection_area", ls_c, 0.5);
    marker_array.markers.push_back(ls_marker);
  }  // for regulatory elements

  return marker_array;
}

visualization_msgs::msg::MarkerArray visualization::pedestrianMarkingsAsMarkerArray(
  const lanelet::ConstLineStrings3d & pedestrian_markings, const std_msgs::msg::ColorRGBA & c)
{
  visualization_msgs::msg::MarkerArray marker_array;

  if (pedestrian_markings.empty()) {
    return marker_array;
  }

  for (const auto & linestring : pedestrian_markings) {
    lanelet::ConstPolygon3d polygon;
    if (utils::lineStringToPolygon(linestring, &polygon)) {
      visualization_msgs::msg::Marker marker = polygonAsMarker(polygon, "pedestrian_marking", c);
      marker.id = linestring.id();
      if (!marker.points.empty()) {
        marker_array.markers.push_back(marker);
      }
    } else {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("lanelet2_extension.visualization"), "pedestrian marking " << linestring.id() << " failed conversion.");
    }
  }
  return marker_array;
}

visualization_msgs::msg::MarkerArray visualization::parkingLotsAsMarkerArray(
  const lanelet::ConstPolygons3d & parking_lots, const std_msgs::msg::ColorRGBA & c)
{
  visualization_msgs::msg::MarkerArray marker_array;

  if (parking_lots.empty()) {
    return marker_array;
  }

  for (const auto & polygon : parking_lots) {
    const visualization_msgs::msg::Marker marker = polygonAsMarker(polygon, "parking_lots", c);
    if (!marker.points.empty()) {
      marker_array.markers.push_back(marker);
    }
  }
  return marker_array;
}
visualization_msgs::msg::MarkerArray visualization::parkingSpacesAsMarkerArray(
  const lanelet::ConstLineStrings3d & parking_spaces, const std_msgs::msg::ColorRGBA & c)
{
  visualization_msgs::msg::MarkerArray marker_array;

  if (parking_spaces.empty()) {
    return marker_array;
  }

  for (const auto & linestring : parking_spaces) {
    lanelet::ConstPolygon3d polygon;
    if (utils::lineStringWithWidthToPolygon(linestring, &polygon)) {
      visualization_msgs::msg::Marker marker = polygonAsMarker(polygon, "parking_space", c);
      marker.id = linestring.id();
      if (!marker.points.empty()) {
        marker_array.markers.push_back(marker);
      }
    } else {
      std::cerr << "parking space " << linestring.id() << " failed conversion." << std::endl;
    }
  }
  return marker_array;
}

visualization_msgs::msg::MarkerArray visualization::generateLaneletIdMarker(
  const lanelet::ConstLanelets road_lanelets, const std_msgs::msg::ColorRGBA c, const double scale)
{
  visualization_msgs::msg::MarkerArray markers;
  for (const auto & ll : road_lanelets) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "lanelet_id";
    marker.id = ll.id();
    marker.type = marker.TEXT_VIEW_FACING;
    marker.action = marker.ADD;
    const auto centerline = ll.centerline();
    const size_t target_position_index = centerline.size() / 2;
    const auto target_position = centerline[target_position_index];
    marker.pose.position.x = target_position.x();
    marker.pose.position.y = target_position.y();
    marker.pose.position.z = target_position.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color = c;
    marker.scale.z = scale;
    marker.frame_locked = true;
    marker.text = std::to_string(ll.id());
    markers.markers.push_back(marker);
  }
  return markers;
}

visualization_msgs::msg::MarkerArray visualization::obstaclePolygonsAsMarkerArray(
  const lanelet::ConstPolygons3d & obstacle_polygons, const std_msgs::msg::ColorRGBA & c)
{
  visualization_msgs::msg::MarkerArray marker_array;

  if (obstacle_polygons.empty()) {
    return marker_array;
  }

  for (const auto & polygon : obstacle_polygons) {
    visualization_msgs::msg::Marker marker = polygonAsMarker(polygon, "obstacles", c);
    marker.id = polygon.id();
    if (!marker.points.empty()) {
      marker_array.markers.push_back(marker);
    }
  }
  return marker_array;
}

visualization_msgs::msg::MarkerArray visualization::lineStringsAsMarkerArray(
  const std::vector<lanelet::ConstLineString3d> line_strings, const std::string name_space,
  const std_msgs::msg::ColorRGBA c, const double lss)
{
  std::unordered_set<lanelet::Id> added;
  visualization_msgs::msg::MarkerArray ls_marker_array;
  for (auto i = line_strings.begin(); i != line_strings.end(); i++) {
    const lanelet::ConstLineString3d & ls = *i;
    if (!exists(added, ls.id())) {
      visualization_msgs::msg::Marker ls_marker;
      visualization::lineString2Marker(ls, &ls_marker, "map", name_space, c, lss);
      ls_marker_array.markers.push_back(ls_marker);
      added.insert(ls.id());
    }
  }

  return ls_marker_array;
}

visualization_msgs::msg::MarkerArray visualization::laneletsBoundaryAsMarkerArray(
  const lanelet::ConstLanelets & lanelets, const std_msgs::msg::ColorRGBA c,
  const bool viz_centerline)
{
  double lss = 0.1;  // line string size
  std::unordered_set<lanelet::Id> added;
  visualization_msgs::msg::MarkerArray marker_array;
  for (auto li = lanelets.begin(); li != lanelets.end(); li++) {
    lanelet::ConstLanelet lll = *li;

    lanelet::ConstLineString3d left_ls = lll.leftBound();
    lanelet::ConstLineString3d right_ls = lll.rightBound();
    lanelet::ConstLineString3d center_ls = lll.centerline();

    visualization_msgs::msg::Marker left_line_strip, right_line_strip, center_line_strip;
    if (!exists(added, left_ls.id())) {
      visualization::lineString2Marker(left_ls, &left_line_strip, "map", "left_lane_bound", c, lss);
      marker_array.markers.push_back(left_line_strip);
      added.insert(left_ls.id());
    }
    if (!exists(added, right_ls.id())) {
      visualization::lineString2Marker(
        right_ls, &right_line_strip, "map", "right_lane_bound", c, lss);
      marker_array.markers.push_back(right_line_strip);
      added.insert(right_ls.id());
    }
    if (viz_centerline && !exists(added, center_ls.id())) {
      visualization::lineString2Marker(
        center_ls, &center_line_strip, "map", "center_lane_line", c, std::max(lss * 0.1, 0.02));
      marker_array.markers.push_back(center_line_strip);
      added.insert(center_ls.id());
    }
  }
  return marker_array;
}

visualization_msgs::msg::MarkerArray visualization::trafficLightsAsTriangleMarkerArray(
  const std::vector<lanelet::TrafficLightConstPtr> tl_reg_elems, const std_msgs::msg::ColorRGBA c,
  const rclcpp::Duration duration, const double scale)
{
  // convert to to an array of linestrings and publish as marker array using
  // existing function

  int tl_count = 0;
  std::vector<lanelet::ConstLineString3d> line_strings;
  visualization_msgs::msg::MarkerArray marker_array;

  for (auto tli = tl_reg_elems.begin(); tli != tl_reg_elems.end(); tli++) {
    lanelet::TrafficLightConstPtr tl = *tli;
    lanelet::LineString3d ls;

    auto lights = tl->trafficLights();
    for (auto lsp : lights) {
      if (lsp.isLineString())  // traffic lights can either polygons or
      {                        // linestrings
        lanelet::ConstLineString3d ls = static_cast<lanelet::ConstLineString3d>(lsp);

        visualization_msgs::msg::Marker marker;
        visualization::trafficLight2TriangleMarker(
          ls, &marker, "traffic_light_triangle", c, duration, scale);
        marker_array.markers.push_back(marker);
        tl_count++;
      }
    }
  }

  return marker_array;
}

visualization_msgs::msg::MarkerArray visualization::laneletsAsTriangleMarkerArray(
  const std::string ns, const lanelet::ConstLanelets & lanelets, const std_msgs::msg::ColorRGBA c)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;

  if (lanelets.empty()) {
    return marker_array;
  }

  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Time();
  marker.frame_locked = true;
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.lifetime = rclcpp::Duration(0, 0);
  marker.pose.position.x = 0.0;  // p.x();
  marker.pose.position.y = 0.0;  // p.y();
  marker.pose.position.z = 0.0;  // p.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 0.999;

  for (auto ll : lanelets) {
    std::vector<geometry_msgs::msg::Polygon> triangles;
    lanelet2Triangle(ll, &triangles);

    for (auto tri : triangles) {
      geometry_msgs::msg::Point tri0[3];

      for (int i = 0; i < 3; i++) {
        utils::conversion::toGeomMsgPt(tri.points[i], &tri0[i]);

        marker.points.push_back(tri0[i]);
        marker.colors.push_back(c);
      }
    }
  }
  if (!marker.points.empty()) {
    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

void visualization::trafficLight2TriangleMarker(
  const lanelet::ConstLineString3d ls, visualization_msgs::msg::Marker * marker,
  const std::string ns,
  const std_msgs::msg::ColorRGBA cl, const rclcpp::Duration duration, const double scale)
{
  if (marker == nullptr) {
    std::cerr << __FUNCTION__ << ": marker is null pointer!" << std::endl;
    return;
  }
  marker->header.frame_id = "map";
  marker->header.stamp = rclcpp::Time();
  marker->frame_locked = true;
  marker->ns = ns;
  marker->id = ls.id();
  marker->type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker->lifetime = duration;

  marker->pose.position.x = 0.0;  // p.x();
  marker->pose.position.y = 0.0;  // p.y();
  marker->pose.position.z = 0.0;  // p.z();
  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = 0.0;
  marker->pose.orientation.w = 1.0;
  marker->scale.x = 1.0;
  marker->scale.y = 1.0;
  marker->scale.z = 1.0;
  marker->color.r = 1.0f;
  marker->color.g = 1.0f;
  marker->color.b = 1.0f;
  marker->color.a = 0.999;

  double h = 0.7;
  if (ls.hasAttribute("height")) {
    lanelet::Attribute attr = ls.attribute("height");
    h = std::stod(attr.value());
  }

  // construct triangles and add to marker

  // define polygon of traffic light border
  Eigen::Vector3d v[4];
  v[0] << ls.front().x(), ls.front().y(), ls.front().z();
  v[1] << ls.back().x(), ls.back().y(), ls.back().z();
  v[2] << ls.back().x(), ls.back().y(), ls.back().z() + h;
  v[3] << ls.front().x(), ls.front().y(), ls.front().z() + h;

  Eigen::Vector3d c = (v[0] + v[1] + v[2] + v[3]) / 4;

  if (scale > 0.0 && scale != 1.0) {
    for (int i = 0; i < 4; i++) {
      v[i] = (v[i] - c) * scale + c;
    }
  }
  geometry_msgs::msg::Point tri0[3];
  utils::conversion::toGeomMsgPt(v[0], &tri0[0]);
  utils::conversion::toGeomMsgPt(v[1], &tri0[1]);
  utils::conversion::toGeomMsgPt(v[2], &tri0[2]);
  geometry_msgs::msg::Point tri1[3];
  utils::conversion::toGeomMsgPt(v[0], &tri1[0]);
  utils::conversion::toGeomMsgPt(v[2], &tri1[1]);
  utils::conversion::toGeomMsgPt(v[3], &tri1[2]);

  for (int i = 0; i < 3; i++) {
    marker->points.push_back(tri0[i]);
    marker->colors.push_back(cl);
  }
  for (int i = 0; i < 3; i++) {
    marker->points.push_back(tri1[i]);
    marker->colors.push_back(cl);
  }
}

void visualization::lineString2Marker(
  const lanelet::ConstLineString3d ls, visualization_msgs::msg::Marker * marker,
  const std::string frame_id, const std::string ns, const std_msgs::msg::ColorRGBA c, const float lss)
{
  if (marker == nullptr) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("lanelet2_extension.visualization"), __FUNCTION__ << ": marker is null pointer!");
    return;
  }

  marker->header.frame_id = frame_id;
  marker->header.stamp = rclcpp::Time();
  marker->frame_locked = true;
  marker->ns = ns;
  marker->action = visualization_msgs::msg::Marker::ADD;
  marker->type = visualization_msgs::msg::Marker::TRIANGLE_LIST;

  marker->id = ls.id();
  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = 0.0;
  marker->pose.orientation.w = 1.0;
  marker->scale.x = 1.0;
  marker->scale.y = 1.0;
  marker->scale.z = 1.0;
  marker->color = c;

  // fill out lane line
  if (ls.size() < 2) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("lanelet2_extension.visualization"), __FUNCTION__ << ": marker line size is 1 or 0!");
    return;
  }
  for (auto i = ls.begin(); i + 1 != ls.end(); i++) {
    geometry_msgs::msg::Point p;
    const float heading = std::atan2((*(i + 1)).y() - (*i).y(), (*(i + 1)).x() - (*i).x());
    const float x_offset = lss * 0.5 * std::sin(heading);
    const float y_offset = lss * 0.5 * std::cos(heading);

    p.x = (*i).x() + x_offset;
    p.y = (*i).y() - y_offset;
    p.z = (*i).z();
    marker->points.push_back(p);
    p.x = (*i).x() - x_offset;
    p.y = (*i).y() + y_offset;
    p.z = (*i).z();
    marker->points.push_back(p);
    p.x = (*(i + 1)).x() + x_offset;
    p.y = (*(i + 1)).y() - y_offset;
    p.z = (*(i + 1)).z();
    marker->points.push_back(p);
    marker->colors.push_back(c);
    p.x = (*(i + 1)).x() + x_offset;
    p.y = (*(i + 1)).y() - y_offset;
    p.z = (*(i + 1)).z();
    marker->points.push_back(p);
    p.x = (*(i + 1)).x() - x_offset;
    p.y = (*(i + 1)).y() + y_offset;
    p.z = (*(i + 1)).z();
    marker->points.push_back(p);
    p.x = (*i).x() - x_offset;
    p.y = (*i).y() + y_offset;
    p.z = (*i).z();
    marker->points.push_back(p);
    marker->colors.push_back(c);
  }
}

}  // namespace lanelet
