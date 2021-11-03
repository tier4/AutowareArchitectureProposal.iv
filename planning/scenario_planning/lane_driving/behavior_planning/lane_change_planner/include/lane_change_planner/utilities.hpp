// Copyright 2019 Autoware Foundation. All rights reserved.
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

#ifndef LANE_CHANGE_PLANNER__UTILITIES_HPP_
#define LANE_CHANGE_PLANNER__UTILITIES_HPP_

#include "lane_change_planner/route_handler.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/dynamic_object_array.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <limits>
#include <string>
#include <vector>

namespace lane_change_planner
{
namespace util
{
using Point = boost::geometry::model::d2::point_xy<double>;
using Polygon = boost::geometry::model::polygon<Point>;
using LineString = boost::geometry::model::linestring<Point>;
struct FrenetCoordinate3d
{
  double length;
  double distance;
  FrenetCoordinate3d() : length(0), distance(0) {}
};

double normalizeRadian(const double radian);
double l2Norm(const geometry_msgs::msg::Vector3 vector);

Eigen::Vector3d convertToEigenPt(const geometry_msgs::msg::Point geom_pt);
std::vector<geometry_msgs::msg::Point> convertToGeometryPointArray(
  const autoware_planning_msgs::msg::PathWithLaneId & path);
geometry_msgs::msg::PoseArray convertToGeometryPoseArray(
  const autoware_planning_msgs::msg::PathWithLaneId & path);

autoware_perception_msgs::msg::PredictedPath convertToPredictedPath(
  const autoware_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Twist & vehicle_twist, const geometry_msgs::msg::Pose & vehicle_pose,
  const double duration, const double resolution, const double acceleration,
  const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr & clock);
autoware_perception_msgs::msg::PredictedPath resamplePredictedPath(
  const autoware_perception_msgs::msg::PredictedPath & input_path, const double resolution,
  const double duration, const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr & clock);

bool convertToFrenetCoordinate3d(
  const std::vector<geometry_msgs::msg::Point> & linestring,
  const geometry_msgs::msg::Point search_point_geom, FrenetCoordinate3d * frenet_coordinate);

geometry_msgs::msg::Pose lerpByPose(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2, const double t);

geometry_msgs::msg::Point lerpByLength(
  const std::vector<geometry_msgs::msg::Point> & array, const double length);
bool lerpByTimeStamp(
  const autoware_perception_msgs::msg::PredictedPath & path, const rclcpp::Time & t,
  geometry_msgs::msg::Pose * lerped_pt, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr & clock);

double getDistance3d(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);
double getDistanceBetweenPredictedPaths(
  const autoware_perception_msgs::msg::PredictedPath & path1,
  const autoware_perception_msgs::msg::PredictedPath & path2, const double start_time,
  const double end_time, const double resolution, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr & clock);

double getDistanceBetweenPredictedPathAndObject(
  const autoware_perception_msgs::msg::DynamicObject & object,
  const autoware_perception_msgs::msg::PredictedPath & path, const double start_time,
  const double end_time, const double resolution, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr & clock);

std::vector<size_t> filterObjectsByLanelets(
  const autoware_perception_msgs::msg::DynamicObjectArray & objects,
  const lanelet::ConstLanelets & lanelets, const double start_arc_length,
  const double end_arc_length, const rclcpp::Logger & logger);

std::vector<size_t> filterObjectsByLanelets(
  const autoware_perception_msgs::msg::DynamicObjectArray & objects,
  const lanelet::ConstLanelets & target_lanelets, const rclcpp::Logger & logger);

bool calcObjectPolygon(
  const autoware_perception_msgs::msg::DynamicObject & object, Polygon * object_polygon,
  const rclcpp::Logger & logger);

std::vector<size_t> filterObjectsByPath(
  const autoware_perception_msgs::msg::DynamicObjectArray & objects,
  const std::vector<size_t> & object_indices,
  const autoware_planning_msgs::msg::PathWithLaneId & ego_path, const double vehicle_width,
  const rclcpp::Logger & logger);

const geometry_msgs::msg::Pose refineGoal(
  const geometry_msgs::msg::Pose & goal, const lanelet::ConstLanelet & goal_lanelet);

autoware_planning_msgs::msg::PathWithLaneId refinePath(
  const double search_radius_range, const double search_rad_range,
  const autoware_planning_msgs::msg::PathWithLaneId & input, const geometry_msgs::msg::Pose & goal,
  const int64_t goal_lane_id, const rclcpp::Logger & logger);
autoware_planning_msgs::msg::PathWithLaneId removeOverlappingPoints(
  const autoware_planning_msgs::msg::PathWithLaneId & input_path);

bool containsGoal(const lanelet::ConstLanelets & lanes, const lanelet::Id & goal_id);

nav_msgs::msg::OccupancyGrid generateDrivableArea(
  const lanelet::ConstLanelets & lanes, const geometry_msgs::msg::PoseStamped & current_pose,
  const double width, const double height, const double resolution, const double vehicle_length,
  const RouteHandler & route_handler);

double getDistanceToEndOfLane(
  const geometry_msgs::msg::Pose & current_pose, const lanelet::ConstLanelets & lanelets);

double getDistanceToNextIntersection(
  const geometry_msgs::msg::Pose & current_pose, const lanelet::ConstLanelets & lanelets);
double getDistanceToCrosswalk(
  const geometry_msgs::msg::Pose & current_pose, const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphContainer & overall_graphs);
double getSignedDistance(
  const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Pose & goal_pose,
  const lanelet::ConstLanelets & lanelets);

std::vector<uint64_t> getIds(const lanelet::ConstLanelets & lanelets);

autoware_planning_msgs::msg::Path convertToPathFromPathWithLaneId(
  const autoware_planning_msgs::msg::PathWithLaneId & path_with_lane_id);

lanelet::Polygon3d getVehiclePolygon(
  const geometry_msgs::msg::Pose & vehicle_pose, const double vehicle_width,
  const double base_link2front);

autoware_planning_msgs::msg::PathPointWithLaneId insertStopPoint(
  double length, autoware_planning_msgs::msg::PathWithLaneId * path);

double getArcLengthToTargetLanelet(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelet & target_lane,
  const geometry_msgs::msg::Pose & pose);

std::vector<Polygon> getTargetLaneletPolygons(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & pose,
  const double check_length, const std::string & target_type);

std::vector<Polygon> filterObstaclePolygons(
  const std::vector<Polygon> & obstacle_polygons,
  const autoware_perception_msgs::msg::DynamicObjectArray & objects,
  const double static_obstacle_velocity_thresh);

double getDistanceToNearestObstaclePolygon(
  const std::vector<Polygon> & obstacle_polygons, const geometry_msgs::msg::Pose & pose);

class SplineInterpolate
{
  bool initialized_ = false;
  std::vector<double> a_;
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;
  std::vector<double> h_;
  rclcpp::Logger logger_;

public:
  explicit SplineInterpolate(const rclcpp::Logger & logger);
  bool interpolate(
    const std::vector<double> & base_index, const std::vector<double> & base_value,
    const std::vector<double> & return_index, std::vector<double> & return_value);

private:
  double getValue(const double & query, const std::vector<double> & base_index) const;
  void generateSpline(
    const std::vector<double> & base_index, const std::vector<double> & base_value);
  bool isIncrease(const std::vector<double> & x) const;
  bool isValidInput(
    const std::vector<double> & base_index, const std::vector<double> & base_value,
    const std::vector<double> & return_index, std::vector<double> & return_value) const;
  std::vector<double> solveLinearSystem(const double omega, const size_t max_iter) const;
  bool isConvergeL1(
    const std::vector<double> & r1, const std::vector<double> & r2,
    const double converge_range) const;
};

}  // namespace util
}  // namespace lane_change_planner

#endif  // LANE_CHANGE_PLANNER__UTILITIES_HPP_
