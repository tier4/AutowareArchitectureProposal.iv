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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <utility>
#include <vector>

#include "autoware_planning_msgs/msg/path_point_with_lane_id.hpp"
#include "autoware_planning_msgs/msg/path_with_lane_id.hpp"
#include "grid_map_core/GridMap.hpp"
#include "lanelet2_core/geometry/Lanelet.h"
#include "lanelet2_core/primitives/Lanelet.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_traffic_rules/TrafficRulesFactory.h"
#include "scene_module/occlusion_spot/occlusion_spot_utils.hpp"

namespace test
{
// Default grid parameter such that UNKNOWN cells have a value of 50
const behavior_velocity_planner::grid_utils::GridParam grid_param = {10, 51};

/* Horizontal lanelet
        0 1 2 3 4 5 6
      0 x x x x x x x
      1
      2 x x x x x x x
      3
      4
      5
      */
inline lanelet::ConstLanelet horizontalLanelet(
  std::pair<double, double> origin = {0, 0}, double width = 2.0, double length = 6.0,
  int nb_points = 2)
{
  lanelet::Lanelet l;
  lanelet::Points3d line;
  for (double x = origin.first; x <= origin.first + length; x += length / (nb_points - 1)) {
    line.emplace_back(lanelet::ConstPoint2d(0, x, origin.second));
  }
  l.setLeftBound(lanelet::LineString3d(0, line));
  line.clear();
  for (double x = origin.second; x <= origin.first + length; x += length / (nb_points - 1)) {
    line.emplace_back(lanelet::ConstPoint2d(0, x, origin.second + width));
  }
  l.setRightBound(lanelet::LineString3d(1, line));
  return lanelet::ConstLanelet(l);
}
/* Vertical lanelet
        0 1 2 3 4 5 6
      0 x   x
      1 x   x
      2 x   x
      3 x   x
      4 x   x
      5 x   x
      */
inline lanelet::ConstLanelet verticalLanelet(
  std::pair<double, double> origin = {0, 0}, double width = 2.0, double length = 5.0,
  int nb_points = 2)
{
  lanelet::Lanelet l;
  lanelet::Points3d line;
  for (double y = origin.second; y <= origin.second + length; y += length / (nb_points - 1)) {
    line.emplace_back(lanelet::ConstPoint2d(0, origin.first, y));
  }
  l.setLeftBound(lanelet::LineString3d(0, line));
  line.clear();
  for (double y = origin.second; y <= origin.second + length; y += length / (nb_points - 1)) {
    line.emplace_back(lanelet::ConstPoint2d(0, origin.first + width, y));
  }
  l.setRightBound(lanelet::LineString3d(1, line));
  return lanelet::ConstLanelet(l);
}

// /!\ columns and rows in the GridMap are "inverted" (x -> rows, y -> columns)
inline grid_map::GridMap generateGrid(int w, int h, double res)
{
  grid_map::GridMap grid{};
  grid_map::Length length(w * res, h * res);
  grid.setGeometry(length, res, grid_map::Position(length.x() / 2.0, length.y() / 2.0));
  grid.add("layer", behavior_velocity_planner::grid_utils::occlusion_cost_value::FREE_SPACE);
  return grid;
}

inline autoware_planning_msgs::msg::PathWithLaneId generatePath(
  double x0, double y0, double x, double y, int nb_points)
{
  autoware_planning_msgs::msg::PathWithLaneId path{};
  double x_step = (x - x0) / (nb_points - 1);
  double y_step = (y - y0) / (nb_points - 1);
  for (int i = 0; i < nb_points; ++i) {
    autoware_planning_msgs::msg::PathPointWithLaneId point{};
    point.point.pose.position.x = x0 + x_step * i;
    point.point.pose.position.y = y0 + y_step * i;
    point.point.pose.position.z = 0.0;
    path.points.push_back(point);
  }
  return path;
}

using behavior_velocity_planner::occlusion_spot_utils::PossibleCollisionInfo;
inline void generatePossibleCollisions(
  std::vector<PossibleCollisionInfo> & possible_collisions, double x0, double y0, double x,
  double y, int nb_cols)
{
  using behavior_velocity_planner::occlusion_spot_utils::ObstacleInfo;
  using behavior_velocity_planner::occlusion_spot_utils::PossibleCollisionInfo;
  const double lon = 0.0;  // assume col_x = intersection_x
  const double lat = -1.0;
  const double velocity = 1.0;
  /**
   * @brief representation of a possible collision between ego and some obstacle
   *                                      ^
   *                                      |
   * Ego ---------collision----------intersection-------> path
   *                                      |
   *             ------------------       |
   *            |     Vehicle      |   obstacle
   *             ------------------
   */
  double x_step = (x - x0) / (nb_cols - 1);
  double y_step = (y - y0) / (nb_cols - 1);
  for (int i = 0; i < nb_cols; ++i) {
    // collision
    ObstacleInfo obstacle_info;
    obstacle_info.position.x = x0 + x_step * i;
    obstacle_info.position.y = y0 + y_step * i + lat;
    obstacle_info.max_velocity = velocity;

    // intersection
    geometry_msgs::msg::Pose intersection_pose{};
    intersection_pose.position.x = x0 + x_step * i + lon;
    intersection_pose.position.x = y0 + y_step * i;

    // collision path point
    autoware_planning_msgs::msg::PathPoint collision_path_point{};
    collision_path_point.pose.position.x = x0 + x_step * i + lon;
    collision_path_point.pose.position.y = y0 + y_step * i;

    lanelet::ArcCoordinates arc;
    arc.length = obstacle_info.position.x;
    arc.distance = obstacle_info.position.y;

    PossibleCollisionInfo col(obstacle_info, collision_path_point, intersection_pose, arc);
    possible_collisions.emplace_back(col);
  }
}
inline void addConstantVelocity(
  autoware_planning_msgs::msg::PathWithLaneId & trajectory, double velocity)
{
  for (auto & p : trajectory.points) {
    p.point.twist.linear.x = velocity;
  }
}
inline autoware_perception_msgs::msg::DynamicObject generateDynamicObject(double x)
{
  autoware_perception_msgs::msg::DynamicObject obj;
  obj.shape.dimensions.x = 5.0;
  obj.shape.dimensions.y = 2.0;
  tf2::Quaternion q;
  obj.state.pose_covariance.pose.orientation = tf2::toMsg(q);
  obj.state.orientation_reliable = true;
  obj.state.twist_reliable = true;
  obj.state.twist_covariance.twist.linear.x = 0;
  obj.state.pose_covariance.pose.position.x = x;
  obj.state.pose_covariance.pose.position.y = 0;
  obj.semantic.type = autoware_perception_msgs::msg::Semantic::CAR;
  return obj;
}
inline geometry_msgs::msg::Pose generatePose(double x)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = 0.0;
  p.position.z = 1.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  p.orientation = tf2::toMsg(q);
  return p;
}

}  // namespace test

#endif  // UTILS_HPP_
