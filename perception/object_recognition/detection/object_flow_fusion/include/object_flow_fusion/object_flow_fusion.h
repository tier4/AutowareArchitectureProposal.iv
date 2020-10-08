/*
 * Copyright 2020 TierIV. All rights reserved.
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

#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <autoware_perception_msgs/DynamicObjectWithFeatureArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "utils.h"

namespace object_flow_fusion
{
class ObjectFlowFusion
{
public:
  ObjectFlowFusion();
  void fusion(
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr& object_msg,
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr& flow_msg,
    bool use_flow_pose, float flow_vel_thresh_,
    autoware_perception_msgs::DynamicObjectWithFeatureArray& fusioned_msg);
private:

  bool getPolygon(
    const autoware_perception_msgs::DynamicObject& object,
    const geometry_msgs::Polygon& input_footprint,
    geometry_msgs::Polygon& output_footprint);

  bool isInsidePolygon(
    const geometry_msgs::Pose& pose,
    const geometry_msgs::Polygon& footprint,
    const geometry_msgs::Point& flow_point);

  bool isInsideCylinder(
    const geometry_msgs::Pose& pose,
    const autoware_perception_msgs::Shape& shape,
    const geometry_msgs::Point& flow_point);

  bool isInsideShape(
    const autoware_perception_msgs::DynamicObject& object,
    const geometry_msgs::Point& flow_point,
    const geometry_msgs::Polygon& footprint);

  geometry_msgs::Twist getLocalTwist(
    const geometry_msgs::Pose& obj_pose, const geometry_msgs::Twist& base_coords_twist);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  float point_radius_;
  std::shared_ptr<Utils> utils_;
  float fusion_box_offset_;
};
} // object_flow_fusion
