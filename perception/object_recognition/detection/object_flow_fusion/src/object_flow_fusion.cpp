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

#include <memory>

#include "tf2_eigen/tf2_eigen.h"

#include "object_flow_fusion/object_flow_fusion.hpp"

namespace object_flow_fusion
{
ObjectFlowFusion::ObjectFlowFusion(float fusion_box_offset)
: fusion_box_offset_(fusion_box_offset)
{
  utils_ = std::make_shared<Utils>();
}

bool ObjectFlowFusion::isInsidePolygon(
  [[maybe_unused]] const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Polygon & footprint,
  const geometry_msgs::msg::Point & flow_point)
{
  double formed_angle_sum = 0;
  Eigen::Vector2d flow_point2d(flow_point.x, flow_point.y);

  for (std::size_t i = 0; i < footprint.points.size(); i++) {
    Eigen::Vector2d a2d, b2d;
    if (i == 0) {
      a2d = Eigen::Vector2d(
        footprint.points.at(footprint.points.size() - 1).x,
        footprint.points.at(footprint.points.size() - 1).y) -
        flow_point2d;
      b2d = Eigen::Vector2d(footprint.points.at(i).x, footprint.points.at(i).y) - flow_point2d;
    } else {
      a2d =
        Eigen::Vector2d(footprint.points.at(i - 1).x, footprint.points.at(i - 1).y) - flow_point2d;
      b2d = Eigen::Vector2d(footprint.points.at(i).x, footprint.points.at(i).y) - flow_point2d;
    }
    double formed_angle = std::acos(a2d.dot(b2d) / (a2d.norm() * b2d.norm()));
    formed_angle_sum += formed_angle;
  }

  double min_angle_range = 358.0 * M_PI / 180.0;
  double max_angle_range = 362.0 * M_PI / 180.0;
  if (min_angle_range < formed_angle_sum && formed_angle_sum < max_angle_range) {
    return true;
  } else {
    return false;
  }
}

bool ObjectFlowFusion::isInsideCylinder(
  const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape,
  const geometry_msgs::msg::Point & flow_point)
{
  Eigen::Affine3d base2obj_transform;
  tf2::fromMsg(pose, base2obj_transform);
  Eigen::Vector3d eigen_flow_point;
  tf2::fromMsg(flow_point, eigen_flow_point);
  auto local_eigen_flow_point = base2obj_transform.inverse() * eigen_flow_point;

  double radius = shape.dimensions.x;
  if (local_eigen_flow_point.norm() < radius) {
    return true;
  }
  return false;
}

bool ObjectFlowFusion::isInsideShape(
  const autoware_perception_msgs::msg::DynamicObject & object,
  const geometry_msgs::msg::Point & flow_point, const geometry_msgs::msg::Polygon & footprint)
{
  geometry_msgs::msg::Pose pose = object.state.pose_covariance.pose;
  autoware_perception_msgs::msg::Shape shape = object.shape;

  if (
    shape.type == autoware_perception_msgs::msg::Shape::POLYGON ||
    shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX)
  {
    return isInsidePolygon(pose, footprint, flow_point);
  } else if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    return isInsideCylinder(pose, shape, flow_point);
  } else {
    return false;
  }
  return true;
}

geometry_msgs::msg::Twist ObjectFlowFusion::getLocalTwist(
  const geometry_msgs::msg::Pose & obj_pose, const geometry_msgs::msg::Twist & base_coords_twist)
{
  Eigen::Affine3d base2obj_transform;
  tf2::fromMsg(obj_pose, base2obj_transform);
  Eigen::Matrix3d base2obj_rot = base2obj_transform.rotation();
  Eigen::Vector3d obj_coords_vector =
    base2obj_rot.inverse() *
    Eigen::Vector3d(
    base_coords_twist.linear.x, base_coords_twist.linear.y, base_coords_twist.linear.z);
  geometry_msgs::msg::Twist obj_coords_twist;
  obj_coords_twist.linear.x = obj_coords_vector.x();
  obj_coords_twist.linear.y = obj_coords_vector.y();
  obj_coords_twist.linear.z = obj_coords_vector.z();
  return obj_coords_twist;
}

bool ObjectFlowFusion::getPolygon(
  const autoware_perception_msgs::msg::DynamicObject & object,
  const geometry_msgs::msg::Polygon & input_footprint,
  geometry_msgs::msg::Polygon & output_footprint)
{
  geometry_msgs::msg::Pose pose = object.state.pose_covariance.pose;
  autoware_perception_msgs::msg::Shape shape = object.shape;

  Eigen::Affine3d base2obj_transform;
  tf2::fromMsg(pose, base2obj_transform);

  if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    auto eigen_c1 = base2obj_transform * Eigen::Vector3d(
      shape.dimensions.x * (0.5 + fusion_box_offset_),
      shape.dimensions.y * (0.5 + fusion_box_offset_), 0);
    geometry_msgs::msg::Point32 c1;
    c1.x = eigen_c1.x();
    c1.y = eigen_c1.y();
    output_footprint.points.push_back(c1);

    auto eigen_c2 = base2obj_transform * Eigen::Vector3d(
      shape.dimensions.x * (0.5 + fusion_box_offset_),
      -shape.dimensions.y * (0.5 + fusion_box_offset_), 0);
    geometry_msgs::msg::Point32 c2;
    c2.x = eigen_c2.x();
    c2.y = eigen_c2.y();
    output_footprint.points.push_back(c2);

    auto eigen_c3 = base2obj_transform * Eigen::Vector3d(
      -shape.dimensions.x * (0.5 + fusion_box_offset_),
      -shape.dimensions.y * (0.5 + fusion_box_offset_), 0);
    geometry_msgs::msg::Point32 c3;
    c3.x = eigen_c3.x();
    c3.y = eigen_c3.y();
    output_footprint.points.push_back(c3);

    auto eigen_c4 = base2obj_transform * Eigen::Vector3d(
      -shape.dimensions.x * (0.5 + fusion_box_offset_),
      shape.dimensions.y * (0.5 + fusion_box_offset_), 0);
    geometry_msgs::msg::Point32 c4;
    c4.x = eigen_c4.x();
    c4.y = eigen_c4.y();
    output_footprint.points.push_back(c4);
  } else if (shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    for (auto p : input_footprint.points) {
      auto eigen_p = base2obj_transform * Eigen::Vector3d(p.x, p.y, p.z);
      geometry_msgs::msg::Point32 basecoords_p;
      basecoords_p.x = eigen_p.x();
      basecoords_p.y = eigen_p.y();
      basecoords_p.z = eigen_p.z();
      output_footprint.points.push_back(basecoords_p);
    }
  } else if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    return true;
  } else {
    return false;
  }
  return true;
}

void ObjectFlowFusion::fusion(
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr & object_msg,
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr & flow_msg,
  bool use_flow_pose, float flow_vel_thresh_,
  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & fused_msg)
{
  for (auto detected_object : object_msg->feature_objects) {
    geometry_msgs::msg::Polygon footprint;
    bool has_polygon =
      getPolygon(detected_object.object, detected_object.object.shape.footprint, footprint);
    if (!has_polygon) {
      continue;
    }
    geometry_msgs::msg::Twist twist_sum;
    size_t flow_count = 0;
    for (auto flow : flow_msg->feature_objects) {
      if (isInsideShape(
          detected_object.object, flow.object.state.pose_covariance.pose.position, footprint))
      {
        twist_sum.linear.x += flow.object.state.twist_covariance.twist.linear.x;
        twist_sum.linear.y += flow.object.state.twist_covariance.twist.linear.y;
        twist_sum.linear.z += flow.object.state.twist_covariance.twist.linear.z;
        flow_count++;
      }
    }

    autoware_perception_msgs::msg::DynamicObjectWithFeature feature_object;
    feature_object = detected_object;
    feature_object.object.state.twist_reliable = false;

    if (flow_count > 0) {
      geometry_msgs::msg::Twist twist_average;
      twist_average.linear.x = twist_sum.linear.x / flow_count;
      twist_average.linear.y = twist_sum.linear.y / flow_count;
      twist_average.linear.z = twist_sum.linear.z / flow_count;
      auto mps_twist_average = utils_->kph2mps(twist_average);
      double vel = std::sqrt(
        std::pow(twist_average.linear.x, 2) + std::pow(twist_average.linear.y, 2) +
        std::pow(twist_average.linear.z, 2));
      if (use_flow_pose && vel > flow_vel_thresh_) {
        // TODO(T.Higashide): fusion orientation_reliable
        // the detection result and flow pose wisely.
        // (now just overwrite the flow pose to the detection result)

        double flow_yaw = std::atan2(mps_twist_average.linear.y, mps_twist_average.linear.x);
        Eigen::Quaterniond eigen_quaternion(Eigen::AngleAxisd(flow_yaw, Eigen::Vector3d::UnitZ()));
        geometry_msgs::msg::Quaternion q = tf2::toMsg(eigen_quaternion);
        feature_object.object.state.pose_covariance.pose.orientation = q;
        feature_object.object.state.orientation_reliable = true;
      }

      auto local_twist =
        getLocalTwist(feature_object.object.state.pose_covariance.pose, mps_twist_average);
      feature_object.object.state.twist_covariance.twist = local_twist;
      feature_object.object.state.twist_reliable = true;
    }
    fused_msg.feature_objects.push_back(feature_object);
  }
}
}  // namespace object_flow_fusion
