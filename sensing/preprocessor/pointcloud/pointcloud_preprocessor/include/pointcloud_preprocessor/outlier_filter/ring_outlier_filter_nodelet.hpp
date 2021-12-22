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

#ifndef POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__RING_OUTLIER_FILTER_NODELET_HPP_
#define POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__RING_OUTLIER_FILTER_NODELET_HPP_

#include "autoware_utils/point_types/types.hpp"
#include "pointcloud_preprocessor/filter.hpp"

#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <vector>

namespace pointcloud_preprocessor
{
using autoware::common::types::PointXYZI;
using autoware::common::types::PointXYZIRADRT;
using autoware::common::types::PointXYZIRADRTGenerator;
using point_cloud_msg_wrapper::PointCloud2Modifier;

class RingOutlierFilterComponent : public pointcloud_preprocessor::Filter
{
protected:
  virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

private:
  double distance_ratio_;
  double object_length_threshold_;
  int num_points_threshold_;

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);

  bool isCluster(const PointCloud2Modifier<PointXYZI> & tmp_modifier)
  {
    const auto x_diff = tmp_modifier.front().x - tmp_modifier.back().x;
    const auto y_diff = tmp_modifier.front().y - tmp_modifier.back().y;
    const auto z_diff = tmp_modifier.front().z - tmp_modifier.back().z;
    const auto xx = x_diff * x_diff;
    const auto yy = y_diff * y_diff;
    const auto zz = z_diff * z_diff;
    return static_cast<int>(tmp_modifier.size()) > num_points_threshold_ ||
           xx + yy + zz >= object_length_threshold_ * object_length_threshold_;
  }

public:
  explicit RingOutlierFilterComponent(const rclcpp::NodeOptions & options);
};

}  // namespace pointcloud_preprocessor
#endif  // POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__RING_OUTLIER_FILTER_NODELET_HPP_
