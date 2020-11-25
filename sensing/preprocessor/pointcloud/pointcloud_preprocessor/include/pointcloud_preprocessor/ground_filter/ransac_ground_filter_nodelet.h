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
/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
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
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
*/
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#pragma once

#include <chrono>

#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_listener.h"

#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_conversions/pcl_conversions.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

#include "boost/geometry.hpp"
#include "boost/geometry/geometries/linestring.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "boost/optional.hpp"
S
#include "pointcloud_preprocessor/filter.hpp"
#include "pointcloud_preprocessor/ground_filter/gencolors.hpp"

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;

namespace pointcloud_preprocessor
{
struct PlaneBasis
{
  Eigen::Vector3d e_x;
  Eigen::Vector3d e_y;
  Eigen::Vector3d e_z;
};

struct RGB
{
  double r = 0.0;
  double g = 0.0;
  double b = 0.0;
};

class RANSACGroundFilterComponent : public pointcloud_preprocessor::Filter
{
  using PointType = pcl::PointXYZ;

protected:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr debug_pose_array_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_ground_cloud_pub_;

  std::string base_frame_ = "base_link";
  std::string unit_axis_ = "z";
  int min_trial_ = 0;
  int max_iterations_ = 0;
  int min_inliers_ = 0;
  int min_points_ = 0;
  double outlier_threshold_ = 0.1;
  double plane_slope_threshold_ = 10.0;
  double height_threshold_ = 0.1;
  bool debug_ = false;
  bool is_initilized_debug_message_ = false;
  Eigen::Vector3d unit_vec_ = Eigen::Vector3d::UnitZ();
  std::vector<RGB> color_map_{{255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                              {0, 255, 255}, {255, 255, 0}, {255, 255, 255}};

  /*!
   * Output transformed PointCloud from in_cloud_ptr->header.frame_id to in_target_frame
   * @param[in] in_target_frame Coordinate system to perform transform
   * @param[in] in_cloud_ptr PointCloud to perform transform
   * @param[out] out_cloud_ptr Resulting transformed PointCloud
   * @retval true transform succeeded
   * @retval false transform failed
   */
  bool transformPointCloud(
    const std::string & in_target_frame, const PointCloud2ConstPtr & in_cloud_ptr,
    const PointCloud2::SharedPtr & out_cloud_ptr);

  /*!
   * Returns the resulting complementary PointCloud, one with the points kept and the other removed as indicated
   * in the indices
   * @param in_cloud_ptr Input PointCloud to which the extraction will be performed
   * @param in_indices Indices of the points to be both removed and kept
   * @param out_only_indices_cloud_ptr Resulting PointCloud with the indices kept
   * @param out_removed_indices_cloud_ptr Resulting PointCloud with the indices removed
   */
  void extractPointsIndices(
    const pcl::PointCloud<PointType>::Ptr in_cloud_ptr, const pcl::PointIndices & in_indices,
    pcl::PointCloud<PointType>::Ptr out_only_indices_cloud_ptr,
    pcl::PointCloud<PointType>::Ptr out_removed_indices_cloud_ptr);

  Eigen::Affine3d getPlaneAffine(
    const pcl::PointCloud<PointType> segment_ground_cloud, const Eigen::Vector3d & plane_normal);

  void applyRecursiveRANSAC(
    const pcl::PointCloud<PointType>::Ptr & input,
    std::vector<pcl::PointIndices::Ptr> & output_inliers,
    std::vector<pcl::ModelCoefficients::Ptr> & output_coefficients,
    pcl::PointCloud<PointType>::Ptr & rest_cloud);

  void publishDebugMessage(
    const geometry_msgs::msg::PoseArray & debug_pose_array,
    const std::vector<pcl::PointCloud<PointType>> & ground_clouds,
    const std_msgs::msg::Header & header);

  void setDebugPublisher();

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RANSACGroundFilterComponent(const rclcpp::NodeOptions & options);
};
}  // namespace pointcloud_preprocessor
