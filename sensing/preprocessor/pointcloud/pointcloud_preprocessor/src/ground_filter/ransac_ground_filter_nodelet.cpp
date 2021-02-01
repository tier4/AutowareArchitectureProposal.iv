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

#include "pointcloud_preprocessor/ground_filter/ransac_ground_filter_nodelet.h"
#include <pcl/common/centroid.h>
#include <pcl_ros/transforms.h>
#include <pluginlib/class_list_macros.h>
#include <random>

namespace
{
Eigen::Vector3d getArbitralyOrthogonalVector(const Eigen::Vector3d & input)
{
  const double x = input.x();
  const double y = input.y();
  const double z = input.z();
  const double x2 = std::pow(x, 2);
  const double y2 = std::pow(y, 2);
  const double z2 = std::pow(z, 2);

  Eigen::Vector3d unit_vec{0, 0, 0};
  if (x2 <= y2 && x2 <= z2) {
    unit_vec.x() = 0;
    unit_vec.y() = z;
    unit_vec.z() = -y;
    unit_vec = unit_vec / std::sqrt(y2 + z2);
  } else if (y2 <= x2 && y2 <= z2) {
    unit_vec.x() = -z;
    unit_vec.y() = 0;
    unit_vec.z() = x;
    unit_vec = unit_vec / std::sqrt(z2 + x2);
  } else if (z2 <= x2 && z2 <= y2) {
    unit_vec.x() = y;
    unit_vec.y() = -x;
    unit_vec.z() = 0;
    unit_vec = unit_vec / std::sqrt(x2 + y2);
  }
  return unit_vec;
}

pointcloud_preprocessor::PlaneBasis getPlaneBasis(const Eigen::Vector3d & plane_normal)
{
  pointcloud_preprocessor::PlaneBasis basis;
  basis.e_z = plane_normal;
  basis.e_x = getArbitralyOrthogonalVector(plane_normal);
  basis.e_y = basis.e_x.cross(basis.e_z);
  return basis;
}

geometry_msgs::Pose getDebugPose(const Eigen::Affine3d & plane_affine)
{
  geometry_msgs::Pose debug_pose;
  const Eigen::Quaterniond quat(plane_affine.rotation());
  debug_pose.position.x = plane_affine.translation().x();
  debug_pose.position.y = plane_affine.translation().y();
  debug_pose.position.z = plane_affine.translation().z();
  debug_pose.orientation.x = quat.x();
  debug_pose.orientation.y = quat.y();
  debug_pose.orientation.z = quat.z();
  debug_pose.orientation.w = quat.w();
  return debug_pose;
}
}  // namespace

namespace pointcloud_preprocessor
{
RANSACGroundFilterNodelet::RANSACGroundFilterNodelet()
{
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

void RANSACGroundFilterNodelet::setDebugPublisher()
{
  if (is_initilized_debug_message_) return;
  debug_pose_array_pub_ =
    advertise<geometry_msgs::PoseArray>(*pnh_, "debug/plane_pose_array", max_queue_size_);
  debug_ground_cloud_pub_ =
    advertise<sensor_msgs::PointCloud2>(*pnh_, "debug/ground/pointcloud", max_queue_size_);
  is_initilized_debug_message_ = true;
}

void RANSACGroundFilterNodelet::publishDebugMessage(
  const geometry_msgs::PoseArray & debug_pose_array,
  const pcl::PointCloud<PointType> & ground_cloud, const std_msgs::Header & header)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_ground_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (const auto & ground_point : ground_cloud.points) {
    pcl::PointXYZRGB p;
    p.x = ground_point.x;
    p.y = ground_point.y;
    p.z = ground_point.z;
    p.r = 255.0;
    p.g = 0.0;
    p.b = 0.0;
    colored_ground_ptr->points.push_back(p);
  }

  sensor_msgs::PointCloud2::Ptr ground_cloud_msg_ptr(new sensor_msgs::PointCloud2);
  ground_cloud_msg_ptr->header = header;
  pcl::toROSMsg(*colored_ground_ptr, *ground_cloud_msg_ptr);
  ground_cloud_msg_ptr->header.frame_id = base_frame_;
  debug_pose_array_pub_.publish(debug_pose_array);
  debug_ground_cloud_pub_.publish(*ground_cloud_msg_ptr);
}

bool RANSACGroundFilterNodelet::transformPointCloud(
  const std::string & in_target_frame, const sensor_msgs::PointCloud2::ConstPtr & in_cloud_ptr,
  const sensor_msgs::PointCloud2::Ptr & out_cloud_ptr)
{
  if (in_target_frame == in_cloud_ptr->header.frame_id) {
    *out_cloud_ptr = *in_cloud_ptr;
    return true;
  }

  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      in_target_frame, in_cloud_ptr->header.frame_id, in_cloud_ptr->header.stamp,
      ros::Duration(1.0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
  // tf2::doTransform(*in_cloud_ptr, *out_cloud_ptr, transform_stamped);
  Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(mat, *in_cloud_ptr, *out_cloud_ptr);
  out_cloud_ptr->header.frame_id = in_target_frame;
  return true;
}

bool RANSACGroundFilterNodelet::child_init(ros::NodeHandle & nh, bool & has_service)
{
  // Enable the dynamic reconfigure service
  has_service = true;

  srv_ = std::make_shared<
    dynamic_reconfigure::Server<pointcloud_preprocessor::RANSACGroundFilterConfig>>(nh);
  dynamic_reconfigure::Server<pointcloud_preprocessor::RANSACGroundFilterConfig>::CallbackType f =
    boost::bind(&RANSACGroundFilterNodelet::config_callback, this, _1, _2);
  srv_->setCallback(f);
  return (true);
}

void RANSACGroundFilterNodelet::extractPointsIndices(
  const pcl::PointCloud<PointType>::Ptr in_cloud_ptr, const pcl::PointIndices & in_indices,
  pcl::PointCloud<PointType>::Ptr out_only_indices_cloud_ptr,
  pcl::PointCloud<PointType>::Ptr out_removed_indices_cloud_ptr)
{
  pcl::ExtractIndices<PointType> extract_ground;
  extract_ground.setInputCloud(in_cloud_ptr);
  extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(in_indices));

  extract_ground.setNegative(false);  // true removes the indices, false leaves only the indices
  extract_ground.filter(*out_only_indices_cloud_ptr);

  extract_ground.setNegative(true);  // true removes the indices, false leaves only the indices
  extract_ground.filter(*out_removed_indices_cloud_ptr);
}

Eigen::Affine3d RANSACGroundFilterNodelet::getPlaneAffine(
  const pcl::PointCloud<PointType> segment_ground_cloud, const Eigen::Vector3d & plane_normal)
{
  pcl::CentroidPoint<pcl::PointXYZ> centroid;
  for (const auto p : segment_ground_cloud.points) {
    centroid.add(p);
  }
  pcl::PointXYZ centroid_point;
  centroid.get(centroid_point);
  Eigen::Translation<double, 3> trans(centroid_point.x, centroid_point.y, centroid_point.z);
  const pointcloud_preprocessor::PlaneBasis basis = getPlaneBasis(plane_normal);
  Eigen::Matrix3d rot;
  rot << basis.e_x.x(), basis.e_y.x(), basis.e_z.x(), basis.e_x.y(), basis.e_y.y(), basis.e_z.y(),
    basis.e_x.z(), basis.e_y.z(), basis.e_z.z();
  return trans * rot;
}

void RANSACGroundFilterNodelet::applyRANSAC(
  const pcl::PointCloud<PointType>::Ptr & input, pcl::PointIndices::Ptr & output_inliers,
  pcl::ModelCoefficients::Ptr & output_coefficients)
{
  pcl::SACSegmentation<PointType> seg;
  seg.setOptimizeCoefficients(true);
  seg.setRadiusLimits(0.3, std::numeric_limits<double>::max());
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(outlier_threshold_);
  seg.setInputCloud(input);
  seg.setMaxIterations(max_iterations_);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.segment(*output_inliers, *output_coefficients);
}

void RANSACGroundFilterNodelet::filter(
  const PointCloud2::ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);

  sensor_msgs::PointCloud2::Ptr input_transed_ptr(new sensor_msgs::PointCloud2);
  if (!transformPointCloud(base_frame_, input, input_transed_ptr)) {
    ROS_ERROR_STREAM_THROTTLE(
      10, "Failed transform from " << base_frame_ << " to " << input->header.frame_id);
    return;
  }
  pcl::PointCloud<PointType>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*input_transed_ptr, *current_sensor_cloud_ptr);

  // downsample pointcloud to reduce ransac calculation cost
  pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>);
  downsampled_cloud->points.reserve(current_sensor_cloud_ptr->points.size());
  pcl::VoxelGrid<PointType> filter;
  filter.setInputCloud(current_sensor_cloud_ptr);
  filter.setLeafSize(voxel_size_x_, voxel_size_y_, voxel_size_z_);
  filter.filter(*downsampled_cloud);

  // apply ransac
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  applyRANSAC(downsampled_cloud, inliers, coefficients);

  if (coefficients->values.empty()) {
    ROS_WARN_THROTTLE(1.0, "failed to find a plane");
    output = *input;
    return;
  }

  // filter too tilt plane to avoid mis-fitting (e.g. fitting to wall plane)
  Eigen::Vector3d plane_normal(
    coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  {
    const auto plane_slope = std::abs(
      std::acos(plane_normal.dot(unit_vec_) / (plane_normal.norm() * unit_vec_.norm())) * 180 /
      M_PI);
    if (plane_slope > plane_slope_threshold_) {
      output = *input;
      return;
    }
  }

  // extract pointcloud from indices
  pcl::PointCloud<PointType>::Ptr segment_ground_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr segment_no_ground_cloud_ptr(new pcl::PointCloud<PointType>);
  extractPointsIndices(
    downsampled_cloud, *inliers, segment_ground_cloud_ptr, segment_no_ground_cloud_ptr);
  const Eigen::Affine3d plane_affine = getPlaneAffine(*segment_ground_cloud_ptr, plane_normal);
  pcl::PointCloud<PointType>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<PointType>);

  // use not downsamped pointcloud for extract pointcloud that higher than height threshold
  for (const auto & p : current_sensor_cloud_ptr->points) {
    const Eigen::Vector3d transformed_point =
      plane_affine.inverse() * Eigen::Vector3d(p.x, p.y, p.z);
    if (std::abs(transformed_point.z()) > height_threshold_) {
      no_ground_cloud_ptr->points.push_back(p);
    }
  }

  sensor_msgs::PointCloud2::Ptr no_ground_cloud_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*no_ground_cloud_ptr, *no_ground_cloud_msg_ptr);
  no_ground_cloud_msg_ptr->header = input->header;
  sensor_msgs::PointCloud2::Ptr no_ground_cloud_transed_msg_ptr(new sensor_msgs::PointCloud2);
  if (!transformPointCloud(base_frame_, no_ground_cloud_msg_ptr, no_ground_cloud_transed_msg_ptr)) {
    ROS_ERROR_STREAM_THROTTLE(
      10, "Failed transform from " << base_frame_ << " to "
                                   << no_ground_cloud_msg_ptr->header.frame_id);
    return;
  }
  output = *no_ground_cloud_transed_msg_ptr;

  // output debug plane coords and ground pointcloud when debug flag is set
  if (debug_) {
    const geometry_msgs::Pose debug_pose = getDebugPose(plane_affine);
    geometry_msgs::PoseArray debug_pose_array;
    debug_pose_array.header.frame_id = base_frame_;
    debug_pose_array.poses.push_back(debug_pose);
    publishDebugMessage(debug_pose_array, *segment_ground_cloud_ptr, input->header);
  }
}

void RANSACGroundFilterNodelet::subscribe() { Filter::subscribe(); }

void RANSACGroundFilterNodelet::unsubscribe() { Filter::unsubscribe(); }

void RANSACGroundFilterNodelet::config_callback(
  pointcloud_preprocessor::RANSACGroundFilterConfig & config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (base_frame_ != config.base_frame) {
    base_frame_ = config.base_frame;
    NODELET_DEBUG(
      "[%s::config_callback] Setting base_frame to: %s.", getName().c_str(),
      config.base_frame.c_str());
  }

  if (max_iterations_ != config.max_iterations) {
    max_iterations_ = config.max_iterations;
    NODELET_DEBUG(
      "[%s::config_callback] Setting max_iterations to: %d.", getName().c_str(),
      config.max_iterations);
  }

  if (min_inliers_ != config.min_inliers) {
    min_inliers_ = config.min_inliers;
    NODELET_DEBUG(
      "[%s::config_callback] Setting min_inliers to: %d.", getName().c_str(), config.min_inliers);
  }

  if (min_points_ != config.min_points) {
    min_points_ = config.min_points;
    NODELET_DEBUG(
      "[%s::config_callback] Setting min_points to: %d.", getName().c_str(), config.min_points);
  }

  if (outlier_threshold_ != config.outlier_threshold) {
    outlier_threshold_ = config.outlier_threshold;
    NODELET_DEBUG(
      "[%s::config_callback] Setting outlier_threshold to: %lf.", getName().c_str(),
      config.outlier_threshold);
  }

  if (height_threshold_ != config.height_threshold) {
    height_threshold_ = config.height_threshold;
    NODELET_DEBUG(
      "[%s::config_callback] Setting height_threshold to: %lf.", getName().c_str(),
      config.height_threshold);
  }

  if (plane_slope_threshold_ != config.plane_slope_threshold) {
    plane_slope_threshold_ = config.plane_slope_threshold;
    NODELET_DEBUG(
      "[%s::config_callback] Setting plane_slope_threshold to: %lf.", getName().c_str(),
      config.plane_slope_threshold);
  }

  if (voxel_size_x_ != config.voxel_size_x) {
    voxel_size_x_ = config.voxel_size_x;
    NODELET_DEBUG(
      "[%s::config_callback] Setting voxel_size_x to: %lf.", getName().c_str(),
      config.voxel_size_x);
  }

  if (voxel_size_y_ != config.voxel_size_y) {
    voxel_size_y_ = config.voxel_size_y;
    NODELET_DEBUG(
      "[%s::config_callback] Setting voxel_size_y to: %lf.", getName().c_str(),
      config.voxel_size_y);
  }

  if (voxel_size_z_ != config.voxel_size_z) {
    voxel_size_z_ = config.voxel_size_z;
    NODELET_DEBUG(
      "[%s::config_callback] Setting voxel_size_z to: %lf.", getName().c_str(),
      config.voxel_size_z);
  }

  if (debug_ != config.debug) {
    debug_ = config.debug;
    setDebugPublisher();
  }

  const auto unit_axis = config.unit_axis;
  if (unit_axis == "x") {
    unit_vec_ = Eigen::Vector3d::UnitX();
  } else if (unit_axis == "y") {
    unit_vec_ = Eigen::Vector3d::UnitY();
  } else if (unit_axis == "z") {
    unit_vec_ = Eigen::Vector3d::UnitZ();
  } else {
    unit_vec_ = Eigen::Vector3d::UnitZ();
  }
  NODELET_DEBUG(
    "[%s::config_callback] Setting unit_axis to: %s.", getName().c_str(), config.unit_axis.c_str());
}

}  // namespace pointcloud_preprocessor

PLUGINLIB_EXPORT_CLASS(pointcloud_preprocessor::RANSACGroundFilterNodelet, nodelet::Nodelet);
