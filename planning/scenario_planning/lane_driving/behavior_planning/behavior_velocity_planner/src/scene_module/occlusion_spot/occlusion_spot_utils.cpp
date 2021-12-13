// Copyright 2021 Tier IV, Inc.
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

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <interpolation/spline_interpolation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>
#include <utilization/interpolate.hpp>
#include <utilization/path_utilization.hpp>
#include <utilization/util.hpp>

#include <deque>
#include <functional>
#include <limits>
#include <numeric>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace occlusion_spot_utils
{
bool splineInterpolate(
  const PathWithLaneId & input, const double interval, PathWithLaneId * output,
  const rclcpp::Logger logger)
{
  *output = input;

  if (input.points.size() <= 1) {
    RCLCPP_WARN(logger, "Do not interpolate because path size is 1.");
    return false;
  }

  static constexpr double ep = 1.0e-8;

  // calc arclength for path
  std::vector<double> base_x;
  std::vector<double> base_y;
  std::vector<double> base_z;
  std::vector<double> base_v;
  for (const auto & p : input.points) {
    base_x.push_back(p.point.pose.position.x);
    base_y.push_back(p.point.pose.position.y);
    base_z.push_back(p.point.pose.position.z);
    base_v.push_back(p.point.longitudinal_velocity_mps);
  }
  std::vector<double> base_s = interpolation::calcEuclidDist(base_x, base_y);

  // remove duplicating sample points
  {
    size_t Ns = base_s.size();
    size_t i = 1;
    while (i < Ns) {
      if (std::fabs(base_s[i - 1] - base_s[i]) < ep) {
        base_s.erase(base_s.begin() + i);
        base_x.erase(base_x.begin() + i);
        base_y.erase(base_y.begin() + i);
        base_z.erase(base_z.begin() + i);
        base_v.erase(base_v.begin() + i);
        Ns -= 1;
        i -= 1;
      }
      ++i;
    }
  }

  std::vector<double> resampled_s;
  for (double d = 0.0; d < base_s.back() - ep; d += interval) {
    resampled_s.push_back(d);
  }

  // do spline for xy
  const std::vector<double> resampled_x = ::interpolation::slerp(base_s, base_x, resampled_s);
  const std::vector<double> resampled_y = ::interpolation::slerp(base_s, base_y, resampled_s);
  const std::vector<double> resampled_z = ::interpolation::slerp(base_s, base_z, resampled_s);
  const std::vector<double> resampled_v = ::interpolation::slerp(base_s, base_v, resampled_s);

  // set xy
  output->points.clear();
  for (size_t i = 0; i < resampled_s.size(); i++) {
    PathPointWithLaneId p;
    p.point.pose.position.x = resampled_x.at(i);
    p.point.pose.position.y = resampled_y.at(i);
    p.point.pose.position.z = resampled_z.at(i);
    p.point.longitudinal_velocity_mps = resampled_v.at(i);
    output->points.push_back(p);
  }

  // set yaw
  for (int i = 1; i < static_cast<int>(resampled_s.size()) - 1; i++) {
    auto p = output->points.at(i - 1).point.pose.position;
    auto n = output->points.at(i + 1).point.pose.position;
    double yaw = std::atan2(n.y - p.y, n.x - p.x);
    output->points.at(i).point.pose.orientation = planning_utils::getQuaternionFromYaw(yaw);
  }
  if (output->points.size() > 1) {
    size_t l = resampled_s.size();
    output->points.front().point.pose.orientation = output->points.at(1).point.pose.orientation;
    output->points.back().point.pose.orientation = output->points.at(l - 1).point.pose.orientation;
  }
  return true;
}

ROAD_TYPE getCurrentRoadType(
  const lanelet::ConstLanelet & current_lanelet,
  [[maybe_unused]] const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  const auto logger{rclcpp::get_logger("behavior_velocity_planner").get_child("occlusion_spot")};
  rclcpp::Clock clock{RCL_ROS_TIME};
  occlusion_spot_utils::ROAD_TYPE road_type;
  std::string location;
  if (
    current_lanelet.hasAttribute(lanelet::AttributeNamesString::Subtype) &&
    current_lanelet.attribute(lanelet::AttributeNamesString::Subtype) ==
      lanelet::AttributeValueString::Highway) {
    location = "highway";
  } else {
    location = current_lanelet.attributeOr("location", "else");
  }
  RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 3000, "location: " << location);
  if (location == "urban" || location == "public") {
    road_type = occlusion_spot_utils::ROAD_TYPE::PUBLIC;
    RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 3000, "public road: " << location);
  } else if (location == "private") {
    road_type = occlusion_spot_utils::ROAD_TYPE::PRIVATE;
    RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 3000, "private road");
  } else if (location == "highway") {
    road_type = occlusion_spot_utils::ROAD_TYPE::HIGHWAY;
    RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 3000, "highway road");
  } else {
    road_type = occlusion_spot_utils::ROAD_TYPE::UNKNOWN;
    RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 3000, "unknown road");
  }
  return road_type;
}

Point setPoint(const double x, const double y, const double z)
{
  Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}
void calcSlowDownPointsForPossibleCollision(
  const int closest_idx, const PathWithLaneId & path, const double offset,
  std::vector<PossibleCollisionInfo> & possible_collisions)
{
  if (possible_collisions.empty()) {
    return;
  }
  // get interpolated value between (s0,v0) - (s1,value) - (s2,v2)
  auto getInterpolatedValue = [](double s0, double v0, double s1, double s2, double v2) {
    if (s2 - s0 < std::numeric_limits<float>::min()) {
      return v0;
    }
    return v0 + (v2 - v0) * (s1 - s0) / (s2 - s0);
  };
  // insert path point orientation to possible collision
  size_t collision_index = 0;
  double dist_along_path_point = offset;
  double dist_along_next_path_point = dist_along_path_point;
  for (size_t idx = closest_idx; idx < path.points.size() - 1; idx++) {
    auto p_prev = path.points[idx].point;
    auto p_next = path.points[idx + 1].point;
    const double dist_to_col =
      possible_collisions.at(collision_index).arc_lane_dist_at_collision.length;
    dist_along_next_path_point +=
      autoware_utils::calcDistance2d(p_prev.pose.position, p_next.pose.position);
    // process if nearest possible collision is between current and next path point
    if (dist_along_path_point < dist_to_col) {
      for (; collision_index < possible_collisions.size(); collision_index++) {
        const double d0 = dist_along_path_point;  // distance at arc coordinate
        const double d1 = dist_along_next_path_point;
        const auto p0 = p_prev.pose.position;
        const auto p1 = p_next.pose.position;
        const double v0 = p_prev.longitudinal_velocity_mps;
        const double v1 = p_next.longitudinal_velocity_mps;
        const double v = getInterpolatedValue(d0, v0, dist_to_col, d1, v1);
        const double z = getInterpolatedValue(d0, p0.z, dist_to_col, d1, p1.z);
        // height is used to visualize marker correctly
        auto & col = possible_collisions[collision_index];
        col.collision_path_point.longitudinal_velocity_mps = v;
        col.collision_path_point.pose.position.z = z;
        col.intersection_pose.position.z = z;
        col.obstacle_info.position.z = z;
        const double current_dist2col = col.arc_lane_dist_at_collision.length + offset;
        // break searching if dist to collision is farther than next path point
        if (dist_along_next_path_point < current_dist2col) {
          break;
        }
      }
      if (collision_index == possible_collisions.size()) {
        break;
      }
    }
    dist_along_path_point = dist_along_next_path_point;
  }
}

// !generate center line from right/left lanelets
void generateCenterLaneLine(
  const PathWithLaneId & path, const lanelet::routing::RoutingGraphPtr & routing_graph_ptr,
  const lanelet::LaneletMapPtr & lanelet_map_ptr,
  std::vector<lanelet::BasicLineString2d> & attension_line)
{
  std::set<int> ids;
  for (const auto & p : path.points) {
    for (const int id : p.lane_ids) {
      ids.insert(id);
    }
  }
  for (const auto id : ids) {
    const auto ll = lanelet_map_ptr->laneletLayer.get(id);
    const auto polygon2d = ll.polygon2d().basicPolygon();
    attension_line.emplace_back(ll.centerline2d().basicLineString());
    const auto right_ll = (!!routing_graph_ptr->right(ll)) ? routing_graph_ptr->right(ll)
                                                           : routing_graph_ptr->adjacentRight(ll);
    const auto left_ll = (!!routing_graph_ptr->left(ll)) ? routing_graph_ptr->left(ll)
                                                         : routing_graph_ptr->adjacentLeft(ll);
    if (right_ll) {
      attension_line.emplace_back(right_ll.get().centerline2d().basicLineString());
      for (const auto l : routing_graph_ptr->following(right_ll.get())) {
        attension_line.emplace_back(l.centerline2d().basicLineString());
      }
    }
    if (left_ll) {
      attension_line.emplace_back(left_ll.get().centerline2d().basicLineString());
      for (const auto l : routing_graph_ptr->following(left_ll.get())) {
        attension_line.emplace_back(l.centerline2d().basicLineString());
      }
    }
  }
}

std::vector<PredictedObject> getParkedVehicles(
  const PredictedObjects & dyn_objects, const std::vector<BasicLineString2d> & attension_line,
  const PlannerParam & param, std::vector<Point> & debug_point)
{
  std::vector<PredictedObject> parked_vehicles;
  std::vector<Point> points;
  for (const auto & obj : dyn_objects.objects) {
    bool is_parked_vehicle = true;
    if (!occlusion_spot_utils::isStuckVehicle(obj, param.stuck_vehicle_vel)) {
      continue;
    }
    const geometry_msgs::msg::Point & p = obj.kinematics.initial_pose_with_covariance.pose.position;
    BasicPoint2d obj_point(p.x, p.y);
    for (const auto & line : attension_line) {
      if (boost::geometry::distance(obj_point, line) < param.lateral_deviation_thr) {
        is_parked_vehicle = false;
        break;
      }
    }
    if (is_parked_vehicle) {
      parked_vehicles.emplace_back(obj);
      points.emplace_back(p);
    }
  }
  debug_point = points;
  return parked_vehicles;
}

ArcCoordinates getOcclusionPoint(const PredictedObject & obj, const ConstLineString2d & ll_string)
{
  Polygon2d poly = planning_utils::toFootprintPolygon(obj);
  std::deque<lanelet::ArcCoordinates> arcs;
  for (const auto & p : poly.outer()) {
    lanelet::BasicPoint2d obj_p = {p.x(), p.y()};
    arcs.emplace_back(lanelet::geometry::toArcCoordinates(ll_string, obj_p));
  }
  /** remove
   *  x--------*
   *  |        |
   *  x--------* <- return
   * Ego===============> path
   **/
  // sort by arc length
  std::sort(arcs.begin(), arcs.end(), [](ArcCoordinates arc1, ArcCoordinates arc2) {
    return arc1.length < arc2.length;
  });
  // remove closests 2 polygon point
  arcs.pop_front();
  arcs.pop_front();
  // sort by arc distance
  std::sort(arcs.begin(), arcs.end(), [](ArcCoordinates arc1, ArcCoordinates arc2) {
    return std::abs(arc1.distance) < std::abs(arc2.distance);
  });
  // return closest to path point which is choosen by farthest 2 points.
  return arcs.at(0);
}

double calcSignedArcDistance(const double distance, const double offset)
{
  // if distance is lower than offset return 0;
  if (std::abs(distance) < offset) {
    return 0;
  } else if (distance < 0) {
    return distance + offset;
  } else {
    return distance - offset;
  }
  // error case
  return -1.0;
}

PossibleCollisionInfo calculateCollisionPathPointFromOcclusionSpot(
  const ArcCoordinates & arc_coord_occlusion,
  const ArcCoordinates & arc_coord_occlusion_with_offset,
  const lanelet::ConstLanelet & path_lanelet, const PlannerParam & param)
{
  /**
   * @brief calculate obstacle collsion intersection from arc coordinate info.
   *                                      ^
   *                                      |
   * Ego ---------collision----------intersection-------> path
   *                                      |
   *             ------------------       |
   *            |     Vehicle      |   obstacle
   *             ------------------
   */
  PossibleCollisionInfo pc;
  const auto ll = path_lanelet.centerline2d();
  BasicPoint2d collision_point =
    fromArcCoordinates(ll, {arc_coord_occlusion_with_offset.length, 0.0});
  BasicPoint2d obstacle_point = fromArcCoordinates(ll, arc_coord_occlusion);
  BasicPoint2d intersection_point = fromArcCoordinates(ll, {arc_coord_occlusion.length, 0.0});
  Point search_point = setPoint(collision_point[0], collision_point[1], 0);
  geometry_msgs::msg::Quaternion q = autoware_utils::createQuaternionFromYaw(
    lanelet::utils::getLaneletAngle(path_lanelet, search_point));
  // arc length to col point
  pc.arc_lane_dist_at_collision = arc_coord_occlusion_with_offset;
  // obstacle info
  pc.obstacle_info.position = setPoint(obstacle_point[0], obstacle_point[1], 0);
  pc.obstacle_info.max_velocity = param.pedestrian_vel;
  pc.obstacle_info.min_deceleration = param.pedestrian_decel;
  // collision_point this value is going to be recalculated later
  pc.collision_path_point.pose.position = search_point;
  pc.collision_path_point.pose.orientation = q;
  // intersection point
  pc.intersection_pose.position = setPoint(intersection_point[0], intersection_point[1], 0);
  pc.intersection_pose.orientation = q;
  return pc;
}

std::vector<PossibleCollisionInfo> generatePossibleCollisionBehindParkedVehicle(
  const lanelet::ConstLanelet & path_lanelet, const PlannerParam & param,
  [[maybe_unused]] const double offset_from_start_to_ego,
  const std::vector<PredictedObject> & dyn_objects)
{
  std::vector<PossibleCollisionInfo> possible_collisions;
  const double half_vehicle_width = 0.5 * param.vehicle_info.vehicle_width;
  const double baselink_to_front = param.vehicle_info.baselink_to_front;
  auto ll = path_lanelet.centerline2d();
  for (const auto & dyn : dyn_objects) {
    ArcCoordinates arc_coord_occlusion = getOcclusionPoint(dyn, ll);
    ArcCoordinates arc_coord_occlusion_with_offset = {
      arc_coord_occlusion.length - baselink_to_front - param.safety_margin,
      calcSignedArcDistance(arc_coord_occlusion.distance, half_vehicle_width)};
    // ignore if collision is not avoidable by velocity control.
    if (
      arc_coord_occlusion_with_offset.length < offset_from_start_to_ego ||
      arc_coord_occlusion_with_offset.length > param.detection_area_length ||
      arc_coord_occlusion_with_offset.length > lanelet::geometry::length2d(path_lanelet) ||
      std::abs(arc_coord_occlusion_with_offset.distance) <= 1e-3 ||
      std::abs(arc_coord_occlusion_with_offset.distance) > param.lateral_distance_thr) {
      continue;
    }
    PossibleCollisionInfo pc = calculateCollisionPathPointFromOcclusionSpot(
      arc_coord_occlusion, arc_coord_occlusion_with_offset, path_lanelet, param);
    possible_collisions.emplace_back(pc);
  }
  // sort by arc length
  std::sort(
    possible_collisions.begin(), possible_collisions.end(),
    [](PossibleCollisionInfo pc1, PossibleCollisionInfo pc2) {
      return pc1.arc_lane_dist_at_collision.length < pc2.arc_lane_dist_at_collision.length;
    });
  return possible_collisions;
}

// -------------------------------------------------------------------------------------------

std::pair<double, double> extractTargetRoadArcLength(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const double max_range, const PathWithLaneId & path,
  const ROAD_TYPE & target_road_type)
{
  bool found_target = false;
  double start_dist = 0;
  double dist_sum = 0;
  // search lanelet that includes target_road_type only
  for (size_t i = 0; i < path.points.size() - 1; i++) {
    ROAD_TYPE search_road_type = occlusion_spot_utils::getCurrentRoadType(
      lanelet_map_ptr->laneletLayer.get(path.points[i].lane_ids[0]), lanelet_map_ptr);
    if (found_target && search_road_type != target_road_type) {
      break;
    }
    // ignore path farther than max range
    if (dist_sum > max_range) {
      break;
    }
    if (!found_target && search_road_type == target_road_type) {
      start_dist = dist_sum;
      found_target = true;
    }
    const auto & curr_p = path.points[i].point.pose.position;
    const auto & next_p = path.points[i + 1].point.pose.position;
    dist_sum += autoware_utils::calcDistance2d(curr_p, next_p);
  }
  return std::pair<int, int>(start_dist, dist_sum);
}

void filterPossibleCollisionByRoadType(
  const LaneletMapPtr & lanelet_map_ptr, const double offset, const PathWithLaneId & path,
  std::vector<PossibleCollisionInfo> & possible_collisions, const ROAD_TYPE road_type,
  const PlannerParam & param)
{
  //! Note : consider offset_from_start_to_ego here
  for (auto & pc : possible_collisions) {
    pc.arc_lane_dist_at_collision.length -= offset;
    pc.arc_lane_dist_at_collision.length -= param.safety_margin;
  }
  if (param.consider_road_type) {
    std::pair<double, double> focus_length =
      extractTargetRoadArcLength(lanelet_map_ptr, param.detection_area_length, path, road_type);
    int idx = 0;
    for (const auto pc : possible_collisions) {
      const auto pc_len = pc.arc_lane_dist_at_collision.length;
      if (focus_length.first < pc_len && pc_len < focus_length.second) {
        continue;
      }
      // -----erase-----|start------target-------end|----erase---
      possible_collisions.erase(possible_collisions.begin() + idx);
    }
    idx++;
  }
}

void generatePossibleCollisions(
  std::vector<PossibleCollisionInfo> & possible_collisions, const PathWithLaneId & path,
  const grid_map::GridMap & grid, const PlannerParam & param,
  std::vector<lanelet::BasicPolygon2d> & debug)
{
  lanelet::ConstLanelet path_lanelet = toPathLanelet(path);
  if (path_lanelet.centerline2d().empty()) {
    return;
  }
  // generate sidewalk possible collision
  generateSidewalkPossibleCollisions(possible_collisions, grid, path_lanelet, param, debug);
  possible_collisions.insert(
    possible_collisions.end(), possible_collisions.begin(), possible_collisions.end());
}

void generateSidewalkPossibleCollisions(
  std::vector<PossibleCollisionInfo> & possible_collisions, const grid_map::GridMap & grid,
  const lanelet::ConstLanelet & path_lanelet, const PlannerParam & param,
  std::vector<lanelet::BasicPolygon2d> & debug)
{
  //! Max length
  const double half_vehicle_width = param.vehicle_info.vehicle_width * 0.5;
  geometry::SliceRange slice_range = {
    0, half_vehicle_width, param.detection_area_length, param.sidewalk.focus_range};
  std::vector<geometry::Slice> sidewalk_slices =
    geometry::buildSidewalkSlices(path_lanelet, slice_range, param.sidewalk.slice_size);
  double length_lower_bound = std::numeric_limits<double>::max();
  double distance_lower_bound = std::numeric_limits<double>::max();
  std::sort(
    sidewalk_slices.begin(), sidewalk_slices.end(),
    [](const geometry::Slice & s1, const geometry::Slice & s2) {
      return s1.range.min_length < s2.range.min_length;
    });
  for (const geometry::Slice & sidewalk_slice : sidewalk_slices) {
    debug.push_back(sidewalk_slice.polygon);
    if ((sidewalk_slice.range.min_length < length_lower_bound ||
         std::abs(sidewalk_slice.range.min_distance) < distance_lower_bound)) {
      std::vector<grid_map::Position> occlusion_spot_positions;
      grid_utils::findOcclusionSpots(
        occlusion_spot_positions, grid, sidewalk_slice.polygon,
        param.sidewalk.min_occlusion_spot_size);
      generateSidewalkPossibleCollisionFromOcclusionSpot(
        possible_collisions, grid, occlusion_spot_positions, path_lanelet, param);
      if (!possible_collisions.empty()) {
        length_lower_bound = sidewalk_slice.range.min_length;
        distance_lower_bound = std::abs(sidewalk_slice.range.min_distance);
        possible_collisions.insert(
          possible_collisions.end(), possible_collisions.begin(), possible_collisions.end());
        debug.push_back(sidewalk_slice.polygon);
      }
    }
  }
}

void generateSidewalkPossibleCollisionFromOcclusionSpot(
  std::vector<PossibleCollisionInfo> & possible_collisions, const grid_map::GridMap & grid,
  const std::vector<grid_map::Position> & occlusion_spot_positions,
  const lanelet::ConstLanelet & path_lanelet, const PlannerParam & param)
{
  const auto & center_line = path_lanelet.centerline2d();
  const double baselink_to_front = param.vehicle_info.baselink_to_front;
  const double half_vehicle_width = param.vehicle_info.vehicle_width * 0.5;
  for (grid_map::Position occlusion_spot_position : occlusion_spot_positions) {
    // arc intersection
    BasicPoint2d obstacle_point(occlusion_spot_position[0], occlusion_spot_position[1]);
    ArcCoordinates arc_coord_occlusion_point = toArcCoordinates(center_line, obstacle_point);
    const double length_to_col =
      arc_coord_occlusion_point.length - baselink_to_front - param.safety_margin;
    ArcCoordinates arc_coord_collision_point = {
      length_to_col, calcSignedArcDistance(arc_coord_occlusion_point.distance, half_vehicle_width)};
    if (length_to_col < 0) {
      continue;
    }
    PossibleCollisionInfo pc = calculateCollisionPathPointFromOcclusionSpot(
      arc_coord_occlusion_point, arc_coord_collision_point, path_lanelet, param);
    const auto & ip = pc.intersection_pose.position;
    const auto & cp = pc.collision_path_point.pose.position;
    bool collision_free_at_intersection =
      grid_utils::isCollisionFree(grid, occlusion_spot_position, grid_map::Position(ip.x, ip.y));
    bool collision_free_at_collision_point =
      grid_utils::isCollisionFree(grid, occlusion_spot_position, grid_map::Position(cp.x, cp.y));
    if (collision_free_at_intersection && collision_free_at_collision_point) {
      possible_collisions.emplace_back(pc);
    }
  }
}

}  // namespace occlusion_spot_utils
}  // namespace behavior_velocity_planner
