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

#include <laserscan_to_occupancy_grid_map/updater/occupancy_grid_map_updater_interface.h>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace costmap_2d
{
class OccupancyGridMapBBFUpdater : public OccupancyGridMapUpdaterInterface
{
public:
  enum Index : int { OCCUPIED = 0, FREE = 1 };
  OccupancyGridMapBBFUpdater(
    const unsigned int cells_size_x, const unsigned int cells_size_y, const float resolution)
  : OccupancyGridMapUpdaterInterface(cells_size_x, cells_size_y, resolution)
  {
    probability_matrix_(Index::OCCUPIED, Index::OCCUPIED) = 0.95;
    probability_matrix_(Index::OCCUPIED, Index::FREE) =
      1.0 - probability_matrix_(OCCUPIED, OCCUPIED);
    probability_matrix_(Index::FREE, Index::FREE) = 0.8;
    probability_matrix_(Index::FREE, Index::OCCUPIED) = 1.0 - probability_matrix_(FREE, FREE);
  };
  ~OccupancyGridMapBBFUpdater() = default;
  virtual bool update(
    const Costmap2D & oneshot_occupancy_grid_map, const geometry_msgs::Pose & robot_pose) override;

private:
  inline unsigned char applyBBF(const unsigned char & z, const unsigned char & o);
  Eigen::Matrix2f probability_matrix_;
};

}  // namespace costmap_2d
