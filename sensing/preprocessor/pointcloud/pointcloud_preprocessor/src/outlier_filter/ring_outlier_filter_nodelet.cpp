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

#include "pointcloud_preprocessor/outlier_filter/ring_outlier_filter_nodelet.hpp"

#include <algorithm>
#include <vector>

namespace pointcloud_preprocessor
{
RingOutlierFilterComponent::RingOutlierFilterComponent(const rclcpp::NodeOptions & options)
: Filter("RingOutlierFilter", options)
{
  // set initial parameters
  {
    distance_ratio_ = static_cast<double>(declare_parameter("distance_ratio", 1.03));
    object_length_threshold_ =
      static_cast<double>(declare_parameter("object_length_threshold", 0.1));
    num_points_threshold_ = static_cast<int>(declare_parameter("num_points_threshold", 4));
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RingOutlierFilterComponent::paramCallback, this, _1));
}

void RingOutlierFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (input->width * input->height < 1) {
    return;
  }
  std::string frame_id = input->header.frame_id;

  std::vector<std::unique_ptr<PointCloud2Modifier<PointXYZIRADRT, PointXYZIRADRTGenerator>>>
    input_ring_array(128);
  std::vector<sensor_msgs::msg::PointCloud2> clouds(128);
  {
    sensor_msgs::msg::PointCloud2::SharedPtr input_ptr =
      std::make_shared<sensor_msgs::msg::PointCloud2>(*input);

    for (std::size_t i = 0; i < input_ring_array.size(); i++) {
      input_ring_array.at(i) =
        std::make_unique<PointCloud2Modifier<PointXYZIRADRT, PointXYZIRADRTGenerator>>(
          clouds.at(i), input_ptr->header.frame_id);
    }

    for (std::size_t idx = 0U; idx < input_ptr->data.size(); idx += input_ptr->point_step) {
      PointXYZIRADRT * pt = reinterpret_cast<PointXYZIRADRT *>(&input_ptr->data[idx]);
      (input_ring_array.at(pt->ring).get())
        ->push_back(PointXYZIRADRT{
          pt->x, pt->y, pt->z, pt->intensity, pt->ring, pt->azimuth, pt->distance, pt->return_type,
          pt->time_stamp});
    }
  }

  output.header = input->header;
  PointCloud2Modifier<PointXYZI> output_modifier{output, frame_id};
  output_modifier.reserve(input->row_step);

  sensor_msgs::msg::PointCloud2 tmp_cloud;
  PointCloud2Modifier<PointXYZI> tmp_modifier{tmp_cloud, frame_id};
  tmp_modifier.reserve(input->row_step);

  for (const auto & ring_modifier : input_ring_array) {
    if (ring_modifier.get()->size() < 2) {
      continue;
    }

    for (auto iter = std::begin(*ring_modifier); iter != std::end(*ring_modifier) - 1; ++iter) {
      tmp_modifier.push_back(PointXYZI{iter->x, iter->y, iter->z, iter->intensity});
      // if(std::abs(iter->distance - (iter+1)->distance) <= std::sqrt(iter->distance) * 0.08)
      const float min_dist = std::min(iter->distance, (iter + 1)->distance);
      const float max_dist = std::max(iter->distance, (iter + 1)->distance);
      float azimuth_diff = (iter + 1)->azimuth - iter->azimuth;
      azimuth_diff = azimuth_diff < 0.f ? azimuth_diff + 36000.f : azimuth_diff;

      if (max_dist < min_dist * distance_ratio_ && azimuth_diff < 100.f) {
        continue;
      }
      if (isCluster(tmp_modifier)) {
        for (auto tmp_iter = std::begin(tmp_modifier); tmp_iter != std::end(tmp_modifier);
             ++tmp_iter) {
          output_modifier.push_back(
            PointXYZI{tmp_iter->x, tmp_iter->y, tmp_iter->z, tmp_iter->intensity});
        }
      }
      tmp_modifier.clear();
    }

    if (isCluster(tmp_modifier)) {
      for (auto tmp_iter = std::begin(tmp_modifier); tmp_iter != std::end(tmp_modifier);
           ++tmp_iter) {
        output_modifier.push_back(
          PointXYZI{tmp_iter->x, tmp_iter->y, tmp_iter->z, tmp_iter->intensity});
      }
    }
    tmp_modifier.clear();
  }
}

rcl_interfaces::msg::SetParametersResult RingOutlierFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (get_param(p, "distance_ratio", distance_ratio_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new distance ratio to: %f.", distance_ratio_);
  }
  if (get_param(p, "object_length_threshold", object_length_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new object length threshold to: %f.", object_length_threshold_);
  }
  if (get_param(p, "num_points_threshold", num_points_threshold_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new num_points_threshold to: %d.", num_points_threshold_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::RingOutlierFilterComponent)
