/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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
 *
 *
 * v1.0 Yukihiro Saito
 */

#include "cylinder.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "autoware_perception_msgs/msg/shape.hpp"

namespace normal
{
bool CylinderModel::estimate(
  const pcl::PointCloud<pcl::PointXYZ> & cluster,
  autoware_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output)
{
  // calc min and max z for cylinder length
  double min_z = 0;
  double max_z = 0;
  for (size_t i = 0; i < cluster.size(); ++i) {
    if (cluster.at(i).z < min_z || i == 0) {min_z = cluster.at(i).z;}
    if (max_z < cluster.at(i).z || i == 0) {max_z = cluster.at(i).z;}
  }

  // calc circumscribed circle on x-y plane
  cv::Mat_<float> cv_points((int)cluster.size(), 2);
  for (size_t i = 0; i < cluster.size(); ++i) {
    cv_points(i, 0) = cluster.at(i).x;  // x
    cv_points(i, 1) = cluster.at(i).y;  // y
  }
  cv::Point2f center;
  float radius;
  cv::minEnclosingCircle(cv::Mat(cv_points).reshape(2), center, radius);
  constexpr double ep = 0.001;
  radius = std::max(radius, (float)ep);

  shape_output.type = autoware_perception_msgs::msg::Shape::CYLINDER;
  pose_output.position.x = center.x;
  pose_output.position.y = center.y;
  pose_output.position.z = min_z + (max_z - min_z) / 2.0f;
  pose_output.orientation.x = 0;
  pose_output.orientation.y = 0;
  pose_output.orientation.z = 0;
  pose_output.orientation.w = 1;
  shape_output.dimensions.x = (double)radius * 2.0;
  shape_output.dimensions.y = (double)radius * 2.0;
  shape_output.dimensions.z = std::max((max_z - min_z), ep);
  return true;
}
}  // namespace normal
