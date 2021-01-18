/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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
 */

#pragma once

#include <costmap_2d/costmap_2d_ros.h>
#include <laserscan_to_occupancy_grid_map/cost_value.h>

namespace costmap_2d
{
class OccupancyGridMapUpdaterInterface : public Costmap2D
{
public:
  OccupancyGridMapUpdaterInterface(
    const unsigned int cells_size_x, const unsigned int cells_size_y, const float resolution)
  : Costmap2D(
      cells_size_x, cells_size_y, resolution, 0.f, 0.f, occupancy_cost_value::NO_INFORMATION){};
  virtual ~OccupancyGridMapUpdaterInterface() = default;
  virtual bool update(
    const Costmap2D & oneshot_occupancy_grid_map, const geometry_msgs::Pose & robot_pose) = 0;
};

}  // namespace costmap_2d
