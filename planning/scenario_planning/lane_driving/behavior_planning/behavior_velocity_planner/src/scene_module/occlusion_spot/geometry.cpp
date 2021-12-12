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

#include <interpolation/spline_interpolation.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scene_module/occlusion_spot/geometry.hpp>
#include <utilization/interpolate.hpp>

#include <algorithm>
#include <vector>

namespace behavior_velocity_planner
{
namespace geometry
{
using lanelet::BasicLineString2d;
using lanelet::BasicPoint2d;
namespace bg = boost::geometry;
namespace lg = lanelet::geometry;

//! this interpolation must not consider original point in order not to make duplicate point
bool splineInterpolate(
  const lanelet::BasicLineString2d & input, const double interval,
  lanelet::BasicLineString2d * output)
{
  if (input.size() <= 1) {
    return false;
  }

  static constexpr double ep = 1.0e-8;

  // calc arclength for path
  std::vector<double> base_x;
  std::vector<double> base_y;
  for (const auto & p : input) {
    base_x.emplace_back(p.x());
    base_y.emplace_back(p.y());
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
        Ns -= 1;
        i -= 1;
      }
      ++i;
    }
  }

  std::vector<double> resampled_s;
  for (double d = 0.0; d < base_s.back() - ep; d += interval) {
    resampled_s.emplace_back(d);
  }

  // do spline for xy
  const std::vector<double> resampled_x = ::interpolation::slerp(base_s, base_x, resampled_s);
  const std::vector<double> resampled_y = ::interpolation::slerp(base_s, base_y, resampled_s);

  // set xy
  for (size_t i = 0; i < resampled_s.size(); i++) {
    output->emplace_back(resampled_x.at(i), resampled_y.at(i));
  }
  return true;
}
void buildSlices(
  std::vector<Slice> & slices, const lanelet::ConstLanelet & path_lanelet, const SliceRange & range,
  const double slice_length, const double slice_width)
{
  lanelet::BasicLineString2d path_boundary = path_lanelet.centerline2d().basicLineString();
  if (path_boundary.size() < 2) {
    return;
  }
  const int num_lateral_slice =
    static_cast<int>(std::abs(range.max_distance - range.min_distance) / slice_width);
  // ignore the last point
  const int num_longitudinal_slice =
    static_cast<int>(std::abs(range.max_length - range.min_length) / slice_length);
  slices.reserve(num_lateral_slice * num_longitudinal_slice);
  // Note: offsetNoThrow is necessary
  // as the sharp turn at the end of the trajectory can "reverse" the linestring
  /**
   * @brief
   * build slice from occupancy grid : the first slice to create is ssss
   *
   *         | 0 | 1 | 2 | 3 | 4 |
   *       0 | --|---|---|---|---|--  outer bound
   *       1 |   | ? | ? |   |   |
   *       2 |   | ? | ? |   |   |
   *       3 |   | ? | ? |   |   |                  ^
   *       4 | s | s | ? |   |   |                  | dist_ratio
   *       5 |-s-|-s-|---|---|---|--  inner bound     --> length ratio
   *       Ego--------------------->     total_slice_length
   */
  lanelet::BasicLineString2d inner_bounds = path_boundary;
  lanelet::BasicLineString2d outer_bounds;
  outer_bounds = lanelet::geometry::offsetNoThrow(path_boundary, range.max_distance);
  // correct self crossing with best effort
  boost::geometry::correct(outer_bounds);
  rclcpp::Clock clock{RCL_ROS_TIME};
  try {
    // if correct couldn't solve self crossing skip this area
    lanelet::geometry::internal::checkForInversion(
      path_boundary, outer_bounds, std::abs(range.max_distance));
  } catch (...) {
    RCLCPP_DEBUG_STREAM_THROTTLE(
      rclcpp::get_logger("behavior_velocity_planner").get_child("occlusion_spot"), clock, 5000,
      "self crossing with offset " << range.max_distance);
  }
  // offset last point is especially messy
  inner_bounds.pop_back();
  outer_bounds.pop_back();
  // resize to the same size as slice
  inner_bounds.resize(std::min(inner_bounds.size(), outer_bounds.size()));
  outer_bounds.resize(std::min(inner_bounds.size(), outer_bounds.size()));
  if (inner_bounds.size() < 2 || outer_bounds.size() < 2) {
    return;
  }
  const double ratio_dist_start = std::abs(range.min_distance / range.max_distance);
  const double ratio_dist_increment = std::min(1.0, slice_width / std::abs(range.max_distance));
  for (int s = 0; s < num_longitudinal_slice; s++) {
    const double length = range.min_length + s * slice_length;
    const double next_length = range.min_length + (s + 1.0) * slice_length;
    const double min_length =
      std::min(lanelet::geometry::length(outer_bounds), lanelet::geometry::length(inner_bounds));
    if (next_length > min_length) {
      break;
    }
    for (int d = 0; d < num_lateral_slice; d++) {
      const double ratio_dist = ratio_dist_start + d * ratio_dist_increment;
      const double next_ratio_dist = ratio_dist_start + (d + 1.0) * ratio_dist_increment;
      Slice slice;
      buildInterpolatedPolygon(
        slice.polygon, inner_bounds, outer_bounds, length, next_length, ratio_dist,
        next_ratio_dist);
      slice.range.min_length = length;
      slice.range.max_length = next_length;
      slice.range.min_distance = ratio_dist * range.max_distance;
      slice.range.max_distance = next_ratio_dist * range.max_distance;
      slices.emplace_back(slice);
    }
  }
}

void buildInterpolatedPolygon(
  lanelet::BasicPolygon2d & polygons, const lanelet::BasicLineString2d & inner_bounds,
  const lanelet::BasicLineString2d & outer_bounds, const double current_length,
  const double next_length, const double from_ratio_dist, const double to_ratio_dist)
{
  if (current_length >= next_length) {
    RCLCPP_WARN(
      rclcpp::get_logger("behavior_velocity_planner").get_child("occlusion_spot"),
      "buildInterpolatedPolygon: starting length must be lower than target length");
  }
  lanelet::BasicLineString2d inner_polygons;
  lanelet::BasicLineString2d outer_polygons;
  inner_polygons.reserve(inner_bounds.size());
  outer_polygons.reserve(outer_bounds.size());
  // Find starting point. Interpolate it if necessary
  double length = 0.0;
  size_t point_index = 0;
  lanelet::BasicPoint2d inner_polygon_from;
  lanelet::BasicPoint2d inner_polygon_to;
  lanelet::BasicPoint2d outer_polygon_from;
  lanelet::BasicPoint2d outer_polygon_to;
  // Search first points of polygon
  for (; length < current_length && point_index < inner_bounds.size() - 1; ++point_index) {
    length +=
      lanelet::geometry::distance2d(inner_bounds[point_index], inner_bounds[point_index + 1]);
  }
  // if find current bound point index
  if (length > current_length) {
    const double length_between_points =
      lanelet::geometry::distance2d(inner_bounds[point_index - 1], inner_bounds[point_index]);
    const double length_ratio =
      (length_between_points - (length - current_length)) / length_between_points;
    inner_polygon_from =
      lerp(inner_bounds[point_index - 1], inner_bounds[point_index], length_ratio);
    outer_polygon_from =
      lerp(outer_bounds[point_index - 1], outer_bounds[point_index], length_ratio);
  } else {
    inner_polygon_from = inner_bounds[point_index];
    outer_polygon_from = outer_bounds[point_index];
    if (length < next_length && point_index < inner_bounds.size() - 1) {
      length +=
        lanelet::geometry::distance2d(inner_bounds[point_index], inner_bounds[point_index + 1]);
      ++point_index;
    }
  }
  inner_polygons.emplace_back(lerp(inner_polygon_from, outer_polygon_from, from_ratio_dist));
  outer_polygons.emplace_back(lerp(inner_polygon_from, outer_polygon_from, to_ratio_dist));

  // Intermediate points
  for (; length < next_length && point_index < inner_bounds.size() - 1; ++point_index) {
    inner_polygons.emplace_back(
      lerp(inner_bounds[point_index], outer_bounds[point_index], from_ratio_dist));
    outer_polygons.emplace_back(
      lerp(inner_bounds[point_index], outer_bounds[point_index], to_ratio_dist));
    length +=
      lanelet::geometry::distance2d(inner_bounds[point_index], inner_bounds[point_index + 1]);
  }
  // Last points
  if (length > next_length) {
    const double length_between_points =
      lanelet::geometry::distance2d(inner_bounds[point_index - 1], inner_bounds[point_index]);
    const double length_ratio =
      (length_between_points - (length - next_length)) / length_between_points;
    inner_polygon_to = lerp(inner_bounds[point_index - 1], inner_bounds[point_index], length_ratio);
    outer_polygon_to = lerp(outer_bounds[point_index - 1], outer_bounds[point_index], length_ratio);
  } else {
    inner_polygon_to = inner_bounds[point_index];
    outer_polygon_to = outer_bounds[point_index];
  }
  inner_polygons.emplace_back(lerp(inner_polygon_to, outer_polygon_to, from_ratio_dist));
  outer_polygons.emplace_back(lerp(inner_polygon_to, outer_polygon_to, to_ratio_dist));
  // Build polygon
  inner_polygons.insert(inner_polygons.end(), outer_polygons.rbegin(), outer_polygons.rend());
  polygons = lanelet::BasicPolygon2d(inner_polygons);
}

void buildSlicePolygons(
  std::vector<Slice> & slices, const lanelet::ConstLanelet & path_lanelet, const SliceRange & range,
  const double slice_length, const double slice_width)
{
  if (path_lanelet.centerline2d().basicLineString().size() < 2) {
    return;
  }
  BasicLineString2d inner_bounds = path_lanelet.centerline2d().basicLineString();
  BasicLineString2d outer_bounds = lg::offsetNoThrow(inner_bounds, range.max_distance);
  bg::correct(inner_bounds);
  bg::correct(outer_bounds);
  rclcpp::Clock clock{RCL_ROS_TIME};
  try {
    // if correct couldn't solve self crossing skip this area
    lg::internal::checkForInversion(inner_bounds, outer_bounds, std::abs(range.max_distance));
  } catch (...) {
    RCLCPP_DEBUG_STREAM_THROTTLE(
      rclcpp::get_logger("behavior_velocity_planner").get_child("occlusion_spot"), clock, 5000,
      "self crossing with offset " << range.max_distance);
  }
  const double longitudinal_max_dist_inner = lg::length(inner_bounds);
  const double longitudinal_max_dist_outer = lg::length(outer_bounds);
  std::cout << "len" << longitudinal_max_dist_outer << std::endl;
  const int num_lateral_slice =
    static_cast<int>(std::abs(range.max_distance - range.min_distance) / slice_width);
  int num_longitudinal_slice = 0;
  double interval_inner = slice_length;
  double interval_outer = slice_length;
  //! correspond interval that is longer than other side
  if (longitudinal_max_dist_outer >= longitudinal_max_dist_inner) {
    num_longitudinal_slice = static_cast<int>(longitudinal_max_dist_outer / slice_length);
    interval_inner = longitudinal_max_dist_inner / num_longitudinal_slice;
  } else {
    num_longitudinal_slice = static_cast<int>(longitudinal_max_dist_inner / slice_length);
    interval_outer = longitudinal_max_dist_outer / num_longitudinal_slice;
  }
  BasicLineString2d inner_bounds_interp;
  BasicLineString2d outer_bounds_interp;
  splineInterpolate(outer_bounds, interval_outer, &outer_bounds_interp);
  splineInterpolate(inner_bounds, interval_inner, &inner_bounds_interp);
  const double ratio_dist_start = std::abs(range.min_distance / range.max_distance);
  const double ratio_dist_increment = std::min(1.0, slice_width / std::abs(range.max_distance));
  lanelet::BasicPolygon2d poly;
  for (int s = 0; s < num_longitudinal_slice - 1; s++) {
    const double length = range.min_length + s * slice_length;
    const double next_length = range.min_length + (s + 1.0) * slice_length;
    for (int d = 0; d < num_lateral_slice; d++) {
      const double ratio_dist = ratio_dist_start + d * ratio_dist_increment;
      const double next_ratio_dist = ratio_dist_start + (d + 1.0) * ratio_dist_increment;
      Slice slice;
      BasicLineString2d inner_polygons;
      BasicLineString2d outer_polygons;
      inner_polygons.emplace_back(lerp(inner_bounds[s], outer_bounds[s], ratio_dist));
      inner_polygons.emplace_back(lerp(inner_bounds[s + 1], outer_bounds[s + 1], ratio_dist));
      outer_polygons.emplace_back(lerp(inner_bounds[s], outer_bounds[s], next_ratio_dist));
      outer_polygons.emplace_back(lerp(inner_bounds[s + 1], outer_bounds[s + 1], next_ratio_dist));
      // Build polygon
      inner_polygons.insert(inner_polygons.end(), outer_polygons.rbegin(), outer_polygons.rend());
      slice.polygon = lanelet::BasicPolygon2d(inner_polygons);
      slice.range.min_length = length;
      slice.range.max_length = next_length;
      slice.range.min_distance = ratio_dist * range.max_distance;
      slice.range.max_distance = next_ratio_dist * range.max_distance;
      slices.emplace_back(slice);
    }
  }
}

std::vector<geometry::Slice> buildSidewalkSlices(
  const lanelet::ConstLanelet & path_lanelet, const double longitudinal_offset,
  const double lateral_offset, const double slice_size, const double lateral_max_dist)
{
  std::vector<geometry::Slice> slices;
  std::vector<geometry::Slice> left_slices;
  std::vector<geometry::Slice> right_slices;
  geometry::SliceRange left_slice_range = {
    longitudinal_offset, 0.0, lateral_offset, lateral_offset + lateral_max_dist};
  geometry::buildSlicePolygons(left_slices, path_lanelet, left_slice_range, slice_size, slice_size);
  geometry::SliceRange right_slice_range = {
    longitudinal_offset, 0.0, -lateral_offset, -lateral_offset - lateral_max_dist};
  geometry::buildSlicePolygons(
    right_slices, path_lanelet, right_slice_range, slice_size, slice_size);
  // Properly order lanelets from closest to furthest
  for (size_t i = 0; i < right_slices.size(); ++i) {
    slices.emplace_back(right_slices[i]);
  }
  for (size_t i = 0; i < left_slices.size(); ++i) {
    slices.emplace_back(left_slices[i]);
  }

  return slices;
}
}  // namespace geometry
}  // namespace behavior_velocity_planner
