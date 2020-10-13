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

#include <Eigen/Dense>
#include <localization_error_monitor/node.hpp>

LocalizationErrorMonitor::LocalizationErrorMonitor()
{
  pnh_.param<double>("scale", scale_, 3);
  pnh_.param<double>("error_ellipse_size", error_ellipse_size_, 1.0);
  pnh_.param<double>("warn_ellipse_size", warn_ellipse_size_, 0.8);

  pose_with_cov_sub_ =
    pnh_.subscribe("input/pose_with_cov", 1, &LocalizationErrorMonitor::onPoseWithCovariance, this);
  ellipse_marker_pub_ = pnh_.advertise<visualization_msgs::Marker>("debug/ellipse_marker", 1, true);

  updater_.setHardwareID("localization_error_monitor");
  updater_.add(
    "localization_accuracy",
    boost::bind(&LocalizationErrorMonitor::checkLocalizationAccuracy, this, _1));

  timer_ = pnh_.createTimer(ros::Duration(0.1), &LocalizationErrorMonitor::onTimer, this);
}

void LocalizationErrorMonitor::onTimer(const ros::TimerEvent & event) { updater_.force_update(); }

void LocalizationErrorMonitor::checkLocalizationAccuracy(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("localization_accuracy", ellipse_.long_radius);
  int8_t diag_level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string diag_message = "ellipse size is within the expected range";
  if (warn_ellipse_size_ <= ellipse_.long_radius) {
    diag_level = diagnostic_msgs::DiagnosticStatus::WARN;
    diag_message = "ellipse size is too large";
  }
  if (error_ellipse_size_ <= ellipse_.long_radius) {
    diag_level = diagnostic_msgs::DiagnosticStatus::ERROR;
    diag_message = "ellipse size is over the expected range";
  }
  stat.summary(diag_level, diag_message);
}

visualization_msgs::Marker LocalizationErrorMonitor::createEllipseMarker(
  const Ellipse & ellipse, const geometry_msgs::PoseWithCovarianceStamped & pose_with_cov)
{
  tf2::Quaternion quat;
  quat.setEuler(0, 0, ellipse.yaw);

  const double ellipse_long_radius = std::min(ellipse.long_radius, 30.0);
  const double ellipse_short_radius = std::min(ellipse.short_radius, 30.0);
  visualization_msgs::Marker marker;
  marker.header = pose_with_cov.header;
  marker.header.stamp = ros::Time();
  marker.ns = "error_ellipse";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose_with_cov.pose.pose;
  marker.pose.orientation = tf2::toMsg(quat);
  marker.scale.x =  ellipse_long_radius * 2;
  marker.scale.y = ellipse_short_radius * 2;
  marker.scale.z = 0.01;
  marker.color.a = 0.1;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  return marker;
}

void LocalizationErrorMonitor::onPoseWithCovariance(
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & input_msg)
{
  // create xy covariance (2x2 matrix)
  // input geometry_msgs::PoseWithCovariance containe 6x6 matrix
  Eigen::Matrix2d xy_covariance;
  const auto cov = input_msg->pose.covariance;
  xy_covariance(0, 0) = cov[0 * 6 + 0];
  xy_covariance(0, 1) = cov[0 * 6 + 1];
  xy_covariance(1, 0) = cov[1 * 6 + 0];
  xy_covariance(1, 1) = cov[1 * 6 + 1];

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(xy_covariance);

  // eigen values and vectors are sorted in ascending order
  ellipse_.long_radius = scale_ * std::sqrt(eigensolver.eigenvalues()(1));
  ellipse_.short_radius = scale_ * std::sqrt(eigensolver.eigenvalues()(0));

  // principal component vector
  const Eigen::Vector2d pc_vector = eigensolver.eigenvectors().col(1);
  ellipse_.yaw = std::atan2(pc_vector.y(), pc_vector.x());

  const auto ellipse_marker = createEllipseMarker(ellipse_, *input_msg);
  ellipse_marker_pub_.publish(ellipse_marker);
}
