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

#include "object_flow_fusion/object_flow_fusion.h"
#include <eigen_conversions/eigen_msg.h>

namespace object_flow_fusion
{
ObjectFlowFusion::ObjectFlowFusion() : nh_(""), pnh_("~")
{
  pnh_.param<float>("fusion_box_offset", fusion_box_offset_, 0.1);
  utils_ = std::make_shared<Utils>();
}

bool ObjectFlowFusion::isInsidePolygon(
  const geometry_msgs::Pose& pose,
  const geometry_msgs::Polygon& footprint,
  const geometry_msgs::Point& flow_point)
{
  double formed_angle_sum = 0;
  Eigen::Vector2d flow_point2d(flow_point.x, flow_point.y);

  for (int i=0; i<footprint.points.size(); i++) {
    Eigen::Vector2d a2d, b2d;
    if (i == 0) {
      a2d = Eigen::Vector2d(footprint.points.at(footprint.points.size()-1).x,
        footprint.points.at(footprint.points.size()-1).y) - flow_point2d;
      b2d = Eigen::Vector2d(footprint.points.at(i).x,
        footprint.points.at(i).y) - flow_point2d;
    } else {
      a2d = Eigen::Vector2d(footprint.points.at(i-1).x,
        footprint.points.at(i-1).y) - flow_point2d;
      b2d = Eigen::Vector2d(footprint.points.at(i).x,
        footprint.points.at(i).y) - flow_point2d;
    }
    double formed_angle = std::acos(a2d.dot(b2d) / (a2d.norm() * b2d.norm()));
    formed_angle_sum += formed_angle;
  }

  double min_angle_range = 358.0 * M_PI / 180.0;
  double max_angle_range = 362.0 * M_PI / 180.0;
  if ( min_angle_range < formed_angle_sum && formed_angle_sum < max_angle_range) {
    return true;
  } else {
    return false;
  }
}

bool ObjectFlowFusion::isInsideCylinder(
  const geometry_msgs::Pose& pose,
  const autoware_perception_msgs::Shape& shape,
  const geometry_msgs::Point& flow_point)
{
  Eigen::Affine3d base2obj_transform;
  tf::poseMsgToEigen(pose, base2obj_transform);
  Eigen::Vector3d eigen_flow_point;
  tf::pointMsgToEigen(flow_point, eigen_flow_point);
  auto local_eigen_flow_point = base2obj_transform.inverse() * eigen_flow_point;

  double radius = shape.dimensions.x;
  if (local_eigen_flow_point.norm() < radius) {
    return true;
  }
  return false;
}

bool ObjectFlowFusion::isInsideShape(
  const autoware_perception_msgs::DynamicObject& object,
  const geometry_msgs::Point& flow_point,
  const geometry_msgs::Polygon& footprint)
{
  geometry_msgs::Pose pose = object.state.pose_covariance.pose;
  autoware_perception_msgs::Shape shape = object.shape;

  if ( shape.type == autoware_perception_msgs::Shape::POLYGON ||
    shape.type == autoware_perception_msgs::Shape::BOUNDING_BOX ) {
    return isInsidePolygon(pose, footprint, flow_point);
  } else if ( shape.type == autoware_perception_msgs::Shape::CYLINDER ) {
    return isInsideCylinder(pose, shape, flow_point);
  } else {
    return false;
  }
  return true;
}

geometry_msgs::Twist ObjectFlowFusion::getLocalTwist(
  const geometry_msgs::Pose& obj_pose, const geometry_msgs::Twist& base_coords_twist)
{
  Eigen::Affine3d base2obj_transform;
  tf::poseMsgToEigen(obj_pose, base2obj_transform);
  Eigen::Matrix3d base2obj_rot = base2obj_transform.rotation();
  Eigen::Vector3d obj_coords_vector =
    base2obj_rot.inverse() * Eigen::Vector3d(base_coords_twist.linear.x,
      base_coords_twist.linear.y,
      base_coords_twist.linear.z);
  geometry_msgs::Twist obj_coords_twist;
  obj_coords_twist.linear.x = obj_coords_vector.x();
  obj_coords_twist.linear.y = obj_coords_vector.y();
  obj_coords_twist.linear.z = obj_coords_vector.z();
  return obj_coords_twist;
}


bool ObjectFlowFusion::getPolygon(
  const autoware_perception_msgs::DynamicObject& object,
  const geometry_msgs::Polygon& input_footprint,
  geometry_msgs::Polygon& output_footprint)
{
  geometry_msgs::Pose pose = object.state.pose_covariance.pose;
  autoware_perception_msgs::Shape shape = object.shape;

  Eigen::Affine3d base2obj_transform;
  tf::poseMsgToEigen(pose, base2obj_transform);

  float offset = 0.1;
  if ( shape.type == autoware_perception_msgs::Shape::BOUNDING_BOX ) {
    auto eigen_c1 = base2obj_transform * Eigen::Vector3d(
      shape.dimensions.x * (0.5 + fusion_box_offset_),
      shape.dimensions.y * (0.5 + fusion_box_offset_),
      0);
    geometry_msgs::Point32 c1;
    c1.x = eigen_c1.x();
    c1.y = eigen_c1.y();
    output_footprint.points.push_back(c1);

    auto eigen_c2 = base2obj_transform * Eigen::Vector3d(
      shape.dimensions.x * (0.5 + fusion_box_offset_),
      -shape.dimensions.y * (0.5 + fusion_box_offset_),
      0);
    geometry_msgs::Point32 c2;
    c2.x = eigen_c2.x();
    c2.y = eigen_c2.y();
    output_footprint.points.push_back(c2);

    auto eigen_c3 = base2obj_transform * Eigen::Vector3d(
      -shape.dimensions.x * (0.5 + fusion_box_offset_),
      -shape.dimensions.y * (0.5 + fusion_box_offset_),
      0);
    geometry_msgs::Point32 c3;
    c3.x = eigen_c3.x();
    c3.y = eigen_c3.y();
    output_footprint.points.push_back(c3);

    auto eigen_c4 = base2obj_transform * Eigen::Vector3d(
      -shape.dimensions.x * (0.5 + fusion_box_offset_),
      shape.dimensions.y * (0.5 + fusion_box_offset_),
      0);
    geometry_msgs::Point32 c4;
    c4.x = eigen_c4.x();
    c4.y = eigen_c4.y();
    output_footprint.points.push_back(c4);
  } else if (shape.type == autoware_perception_msgs::Shape::POLYGON) {
    for ( auto p : input_footprint.points ) {
      auto eigen_p = base2obj_transform * Eigen::Vector3d(p.x, p.y, p.z);
      geometry_msgs::Point32 basecoords_p;
      basecoords_p.x = eigen_p.x();
      basecoords_p.y = eigen_p.y();
      basecoords_p.z = eigen_p.z();
      output_footprint.points.push_back(basecoords_p);
    }
  } else if (shape.type == autoware_perception_msgs::Shape::CYLINDER) {
    return true;
  } else {
    return false;
  }
  return true;
}

void ObjectFlowFusion::fusion(
  const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr& object_msg,
  const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr& flow_msg,
  bool use_flow_pose, float flow_vel_thresh_,
  autoware_perception_msgs::DynamicObjectWithFeatureArray& fusioned_msg)
{
  for (auto detected_object : object_msg->feature_objects) {
    geometry_msgs::Polygon footprint;
    bool has_polygon = getPolygon(detected_object.object,
      detected_object.object.shape.footprint,
      footprint);
    if (!has_polygon) {
      continue;
    }
    geometry_msgs::Twist twist_sum;
    size_t flow_count = 0;
    for (auto flow : flow_msg->feature_objects) {
      if ( isInsideShape(detected_object.object,
          flow.object.state.pose_covariance.pose.position,
          footprint) ) {
        twist_sum.linear.x += flow.object.state.twist_covariance.twist.linear.x;
        twist_sum.linear.y += flow.object.state.twist_covariance.twist.linear.y;
        twist_sum.linear.z += flow.object.state.twist_covariance.twist.linear.z;
        flow_count++;
      }
    }

    autoware_perception_msgs::DynamicObjectWithFeature feature_object;
    feature_object = detected_object;
    feature_object.object.state.twist_reliable = false;

    if ( flow_count > 0 ) {
      geometry_msgs::Twist twist_average;
      twist_average.linear.x = twist_sum.linear.x / flow_count;
      twist_average.linear.y = twist_sum.linear.y / flow_count;
      twist_average.linear.z = twist_sum.linear.z / flow_count;
      auto mps_twist_average = utils_->kph2mps(twist_average);
      double vel = std::sqrt(std::pow(twist_average.linear.x, 2) +
        std::pow(twist_average.linear.y, 2) +
        std::pow(twist_average.linear.z, 2));
      if ( use_flow_pose && vel > flow_vel_thresh_ ) {
        // TODO: fusion orientation_reliable the detection result and flow pose wisely.
        // (now just overwrite the flow pose to the detection result)

        double flow_yaw = std::atan2(mps_twist_average.linear.y, mps_twist_average.linear.x);
        Eigen::Quaterniond eigen_quaternion(Eigen::AngleAxisd(flow_yaw, Eigen::Vector3d::UnitZ()));
        geometry_msgs::Quaternion q;
        tf::quaternionEigenToMsg(eigen_quaternion, q);
        feature_object.object.state.pose_covariance.pose.orientation = q;
        feature_object.object.state.orientation_reliable = true;
      }

      auto local_twist = getLocalTwist(feature_object.object.state.pose_covariance.pose, mps_twist_average);
      feature_object.object.state.twist_covariance.twist = local_twist;
      feature_object.object.state.twist_reliable = true;
    }
    fusioned_msg.feature_objects.push_back(feature_object);
  }
}
} // object_flow_fusion
