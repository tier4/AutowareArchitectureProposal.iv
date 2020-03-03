// Copyright 2020 Embotech AG, Zurich, Switzerland
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

#ifndef GEOMETRY__INTERSECTION_HPP_
#define GEOMETRY__INTERSECTION_HPP_


#include <motion_common/config.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/bounding_box.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <geometry/convex_hull.hpp>
#include <geometry/common_2d.hpp>

#include <limits>
#include <vector>
#include <iostream>
#include <list>
#include <utility>
#include <type_traits>
#include <algorithm>

namespace autoware
{
namespace common
{
namespace geometry
{
using motion::motion_common::VehicleConfig;
using autoware_auto_msgs::msg::TrajectoryPoint;
using autoware_auto_msgs::msg::BoundingBox;
using motion::motion_common::to_angle;
using autoware::common::geometry::convex_hull;
using autoware::common::geometry::get_normal;
using autoware::common::geometry::dot_2d;
using autoware::common::geometry::minus_2d;
using autoware::common::geometry::times_2d;
using autoware::common::geometry::norm_2d;
using autoware::common::geometry::closest_line_point_2d;

using Point = geometry_msgs::msg::Point32;

namespace details
{

using Line = std::pair<Point, Point>;

/// \tparam Iter1 Iterator over point-types that must have point adapters
//      defined or have float members x and y
/// \brief Compute a sorted list of faces of a polyhedron given a list of points
/// \param[in] start Start iterator of the list of points
/// \param[in] end End iterator of the list of points
/// \return The list of faces
template<typename Iter>
std::vector<Line> get_sorted_face_list(const Iter start, const Iter end)
{
  // First get a sorted list of points - convex_hull does that by modifying its argument
  auto corner_list = std::list<Point>(start, end);
  const auto first_interior_point = convex_hull(corner_list);

  std::vector<Line> face_list{};
  auto itLast = corner_list.begin();
  auto itNext = std::next(itLast, 1);
  do {
    face_list.emplace_back(Line{*itLast, *itNext});
    itLast = itNext;
    itNext = std::next(itNext, 1);
  } while ((itNext != first_interior_point) && (itNext != corner_list.end()));

  face_list.emplace_back(Line{*itLast, corner_list.front()});

  return face_list;
}


}  // namespace details

// TODO(s.me) implement GJK(+EPA) algorithm as well as per Chris Ho's suggestion
/// \tparam Iter Iterator over point-types that must have point adapters
//      defined or have float members x and y
/// \brief Check if polyhedra defined by two given sets of points intersect
//    This uses SAT for doing the actual checking
//    (https://en.wikipedia.org/wiki/Hyperplane_separation_theorem#Use_in_collision_detection)
/// \param[in] begin1 Start iterator to first list of point types
/// \param[in] end1   End iterator to first list of point types
/// \param[in] begin2 Start iterator to first list of point types
/// \param[in] end2   End iterator to first list of point types
/// \return true if the boxes collide, false otherwise.
template<typename Iter>
bool intersect(const Iter begin1, const Iter end1, const Iter begin2, const Iter end2)
{
  // Obtain sorted lists of faces of both boxes, merge them into one big list of faces
  auto faces = details::get_sorted_face_list(begin1, end1);
  const auto faces_2 = details::get_sorted_face_list(begin2, end2);
  faces.reserve(faces.size() + faces_2.size());
  faces.insert(faces.end(), faces_2.begin(), faces_2.end() );

  // Also look at last line
  for (const auto & face : faces) {
    // Compute normal vector to the face and define a closure to get progress along it
    const auto normal = get_normal(minus_2d(face.second, face.first));
    auto get_position_along_line = [&normal](auto point)
      {
        return dot_2d(normal, minus_2d(point, Point{}) );
      };

    // Define a function to get the minimum and maximum projected position of the corners
    // of a given bounding box along the normal line of the face
    auto get_projected_min_max = [&get_position_along_line, &normal](Iter begin, Iter end)
      {
        const auto zero_point = Point{};
        auto min_corners =
          get_position_along_line(closest_line_point_2d(normal, zero_point, *begin));
        auto max_corners = min_corners;

        for (auto & point = begin; point != end; ++point) {
          const auto point_projected = closest_line_point_2d(normal, zero_point, *point);
          const auto position_along_line = get_position_along_line(point_projected);
          min_corners = std::min(min_corners, position_along_line);
          max_corners = std::max(max_corners, position_along_line);
        }
        return std::pair<float, float>{min_corners, max_corners};
      };

    // Perform the actual computations for the extent computation
    auto minmax_1 = get_projected_min_max(begin1, end1);
    auto minmax_2 = get_projected_min_max(begin2, end2);

    // Check for any intersections
    const auto eps = std::numeric_limits<decltype(minmax_1.first)>::epsilon();
    if (minmax_1.first > minmax_2.second + eps || minmax_2.first > minmax_1.second + eps) {
      // Found separating hyperplane, stop
      return false;
    }
  }

  // No separating hyperplane found, boxes collide
  return true;
}
}  // namespace geometry
}  // namespace common
}  // namespace autoware

#endif  // GEOMETRY__INTERSECTION_HPP_
