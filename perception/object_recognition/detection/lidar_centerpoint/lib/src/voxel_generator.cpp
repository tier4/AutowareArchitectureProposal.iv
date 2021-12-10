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

#include <voxel_generator.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <torch/torch.h>

namespace centerpoint
{
VoxelGenerator::VoxelGenerator(const DensificationParam & param)
{
  pd_ptr_ = std::make_unique<PointCloudDensification>(param);
}

int VoxelGenerator::pointsToVoxels(
  at::Tensor & voxels, at::Tensor & coordinates, at::Tensor & num_points_per_voxel)
{
  // voxels (float): (max_num_voxels, max_num_points_per_voxel, num_point_features)
  // coordinates (int): (max_num_voxels, num_point_dims)
  // num_points_per_voxel (int): (max_num_voxels)

  // TODO(yukke42): not initialize all data
  at::Tensor coord_to_voxel_idx = torch::full(
    {Config::grid_size_z, Config::grid_size_y, Config::grid_size_x}, -1,
    at::TensorOptions().dtype(torch::kInt));

  auto voxels_p = voxels.data_ptr<float>();
  auto coordinates_p = coordinates.data_ptr<int>();
  auto num_points_per_voxel_p = num_points_per_voxel.data_ptr<int>();
  auto coord_to_voxel_idx_p = coord_to_voxel_idx.data_ptr<int>();

  int voxel_cnt = 0;  // @return
  float recip_voxel_size[3] = {
    1 / Config::voxel_size_x, 1 / Config::voxel_size_y, 1 / Config::voxel_size_z};
  float point[Config::num_point_features];
  int coord_zyx[Config::num_point_dims];
  bool out_of_range;
  int c, coord_idx, voxel_idx, point_cnt;
  Eigen::Vector3f point_current, point_past;

  for (auto pc_cache_iter = pd_ptr_->getPointCloudCacheIter(); !pd_ptr_->isCacheEnd(pc_cache_iter);
       pc_cache_iter++) {
    auto pc_msg = pc_cache_iter->pointcloud_msg;
    auto affine_past2current =
      pd_ptr_->getAffineGlobalToCurrent() * pc_cache_iter->affine_past2global;
    float timelag = pd_ptr_->getCurrentTimestamp() -
                    static_cast<float>(rclcpp::Time(pc_msg.header.stamp).seconds());

    for (sensor_msgs::PointCloud2ConstIterator<float> x_iter(pc_msg, "x"), y_iter(pc_msg, "y"),
         z_iter(pc_msg, "z");
         x_iter != x_iter.end(); ++x_iter, ++y_iter, ++z_iter) {
      point_past << *x_iter, *y_iter, *z_iter;
      point_current = affine_past2current * point_past;

      point[0] = point_current.x();
      point[1] = point_current.y();
      point[2] = point_current.z();
      point[3] = timelag;

      out_of_range = false;
      for (int di = 0; di < Config::num_point_dims; di++) {
        c = static_cast<int>((point[di] - pointcloud_range_[di]) * recip_voxel_size[di]);
        if (c < 0 || c >= grid_size_[di]) {
          out_of_range = true;
          break;
        }
        coord_zyx[Config::num_point_dims - di - 1] = c;
      }
      if (out_of_range) {
        continue;
      }

      coord_idx = coord_zyx[0] * Config::grid_size_y * Config::grid_size_x +
                  coord_zyx[1] * Config::grid_size_x + coord_zyx[2];
      voxel_idx = coord_to_voxel_idx_p[coord_idx];
      if (voxel_idx == -1) {
        voxel_idx = voxel_cnt;
        if (voxel_cnt >= Config::max_num_voxels) {
          continue;
        }

        voxel_cnt++;
        coord_to_voxel_idx_p[coord_idx] = voxel_idx;
        for (int di = 0; di < Config::num_point_dims; di++) {
          coordinates_p[voxel_idx * Config::num_point_dims + di] = coord_zyx[di];
        }
      }

      point_cnt = num_points_per_voxel_p[voxel_idx];
      if (point_cnt < Config::max_num_points_per_voxel) {
        for (int fi = 0; fi < Config::num_point_features; fi++) {
          voxels_p
            [voxel_idx * Config::max_num_points_per_voxel * Config::num_point_features +
             point_cnt * Config::num_point_features + fi] = point[fi];
        }
        num_points_per_voxel_p[voxel_idx]++;
      }
    }
  }

  return voxel_cnt;
}

}  // namespace centerpoint
