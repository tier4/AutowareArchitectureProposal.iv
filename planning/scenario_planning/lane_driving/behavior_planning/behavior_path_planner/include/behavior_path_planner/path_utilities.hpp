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

#ifndef BEHAVIOR_PATH_PLANNER__PATH_UTILITIES_HPP_
#define BEHAVIOR_PATH_PLANNER__PATH_UTILITIES_HPP_

#include <opencv2/opencv.hpp>

#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <limits>
#include <vector>

namespace behavior_path_planner
{
namespace util
{
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;

std::vector<double> calcPathArcLengthArray(
  const PathWithLaneId & path, size_t start = 0, size_t end = std::numeric_limits<size_t>::max(),
  double offset = 0.0);

double calcPathArcLength(
  const PathWithLaneId & path, size_t start = 0, size_t end = std::numeric_limits<size_t>::max());

PathWithLaneId resamplePathWithSpline(const PathWithLaneId & path, double interval);

Path toPath(const PathWithLaneId & input);

size_t getIdxByArclength(
  const PathWithLaneId & path, const Point & origin, const double signed_arc);

void clipPathLength(
  PathWithLaneId & path, const Point base_pos, const double forward, const double backward);

}  // namespace util
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__PATH_UTILITIES_HPP_
