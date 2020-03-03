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

#include "geometry/vehicle_bounding_box.hpp"

#include <motion_common/motion_common.hpp>
#include <geometry/bounding_box/rotating_calipers.hpp>
#include <geometry/convex_hull.hpp>
#include <geometry/common_2d.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <limits>
#include <vector>
#include <array>
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
using motion::motion_common::to_angle;
using geometry_msgs::msg::Point32;
using autoware::common::geometry::bounding_box::minimum_perimeter_bounding_box;

BoundingBox compute_boundingbox_from_trajectorypoint(
  const TrajectoryPoint & state,
  const VehicleConfig & vehicle_param)
{
  // Shorthands to keep the formulas sane
  const float h = to_angle(state.heading);
  const float xcog = state.x, ycog = state.y;
  const float lf = vehicle_param.length_cg_front_axel() + vehicle_param.front_overhang();
  const float lr = vehicle_param.length_cg_rear_axel() + vehicle_param.rear_overhang();
  const float wh = vehicle_param.width() * 0.5f;
  const float ch = std::cos(h), sh = std::sin(h);

  // We need a list for the bounding box call later
  std::list<Point32> vehicle_corners;

  {     // Front left
    auto p = Point32{};
    p.x = xcog + (lf * ch) - (wh * sh);
    p.y = ycog + (lf * sh) + (wh * ch);
    vehicle_corners.push_back(p);
  }
  {     // Front right
    auto p = Point32{};
    p.x = xcog + (lf * ch) + (wh * sh);
    p.y = ycog + (lf * sh) - (wh * ch);
    vehicle_corners.push_back(p);
  }
  {     // Rear right
    auto p = Point32{};
    p.x = xcog - (lr * ch) + (wh * sh);
    p.y = ycog - (lr * sh) - (wh * ch);
    vehicle_corners.push_back(p);
  }
  {     // Rear right
    auto p = Point32{};
    p.x = xcog - (lr * ch) - (wh * sh);
    p.y = ycog - (lr * sh) + (wh * ch);
    vehicle_corners.push_back(p);
  }

  return minimum_perimeter_bounding_box(vehicle_corners);
}


}  // namespace geometry
}  // namespace common
}  // namespace autoware
