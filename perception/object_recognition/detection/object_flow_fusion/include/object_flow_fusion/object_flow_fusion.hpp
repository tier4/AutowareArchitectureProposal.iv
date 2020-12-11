// Copyright 2020 TierIV
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

#pragma once

#include <iostream>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_with_feature_array.hpp"
#include "pcl_conversions/pcl_conversions.h"
// #include "eigen_conversions/eigen_msg.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "utils.hpp"

namespace object_flow_fusion
{
class ObjectFlowFusion : public rclcpp::Node
{
public:
  ObjectFlowFusion();
  void fusion(
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstPtr& object_msg,
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstPtr& flow_msg,
    bool use_flow_pose, float flow_vel_thresh_,
    autoware_perception_msgs::msg::DynamicObjectWithFeatureArray& fusioned_msg);
private:

  bool getPolygon(
    const autoware_perception_msgs::msg::DynamicObject& object,
    const geometry_msgs::msg::Polygon& input_footprint,
    geometry_msgs::msg::Polygon& output_footprint);

  bool isInsidePolygon(
    const geometry_msgs::msg::Pose& pose,
    const geometry_msgs::msg::Polygon& footprint,
    const geometry_msgs::msg::Point& flow_point);

  bool isInsideCylinder(
    const geometry_msgs::msg::Pose& pose,
    const autoware_perception_msgs::msg::Shape& shape,
    const geometry_msgs::msg::Point& flow_point);

  bool isInsideShape(
    const autoware_perception_msgs::msg::DynamicObject& object,
    const geometry_msgs::msg::Point& flow_point,
    const geometry_msgs::msg::Polygon& footprint);

  geometry_msgs::msg::Twist getLocalTwist(
    const geometry_msgs::msg::Pose& obj_pose, const geometry_msgs::msg::Twist& base_coords_twist);

  float point_radius_;
  std::shared_ptr<Utils> utils_;
  float fusion_box_offset_;
};
} // object_flow_fusion
