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
#ifndef GEOMETRY__VEHICLE_BOUNDING_BOX_HPP_
#define GEOMETRY__VEHICLE_BOUNDING_BOX_HPP_

#include <motion_common/motion_common.hpp>
#include <motion_common/config.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/bounding_box.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <geometry/visibility_control.hpp>

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

using motion::motion_common::VehicleConfig;
using autoware_auto_msgs::msg::TrajectoryPoint;
using autoware_auto_msgs::msg::BoundingBox;

/// \brief Compute a bounding box from a vehicle state and configuration
/// \param[in] state State of the vehicle
/// \param[in] vehicle_param Parameters of the vehicle
/// \return Bounding box for the given state and vehicle parameters
GEOMETRY_PUBLIC BoundingBox compute_boundingbox_from_trajectorypoint(
  const TrajectoryPoint & state,
  const VehicleConfig & vehicle_param);

}  // namespace geometry
}  // namespace common
}  // namespace autoware

#endif  // GEOMETRY__VEHICLE_BOUNDING_BOX_HPP_
