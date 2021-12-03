// Copyright 2017-2019 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "common/types.hpp"
#include "voxel_grid/voxel_grid.hpp"

namespace autoware
{
namespace perception
{
namespace filters
{
namespace voxel_grid
{
////////////////////////////////////////////////////////////////////////////////
// Instantiation of common types
template class VoxelGrid<ApproximateVoxel<PointXYZ>>;
template class VoxelGrid<ApproximateVoxel<autoware::common::types::PointXYZIF>>;
template class VoxelGrid<CentroidVoxel<PointXYZ>>;
template class VoxelGrid<CentroidVoxel<autoware::common::types::PointXYZIF>>;
}  // namespace voxel_grid
}  // namespace filters
}  // namespace perception
}  // namespace autoware
