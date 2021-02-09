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

#ifndef LOCALIZATION_ERROR_MONITOR__NODE_HPP_
#define LOCALIZATION_ERROR_MONITOR__NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


struct Ellipse
{
  double long_radius;
  double short_radius;
  double yaw;
};

class LocalizationErrorMonitor : public rclcpp::Node
{
private:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_with_cov_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ellipse_marker_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  double scale_;
  double error_ellipse_size_;
  double warn_ellipse_size_;
  Ellipse ellipse_;
  diagnostic_updater::Updater updater_;

  void checkLocalizationAccuracy(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void onPoseWithCovariance(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr input_msg);
  visualization_msgs::msg::Marker createEllipseMarker(
    const Ellipse & ellipse,
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_with_cov);
  void onTimer();

public:
  LocalizationErrorMonitor();
  ~LocalizationErrorMonitor() = default;
};
#endif  // LOCALIZATION_ERROR_MONITOR__NODE_HPP_
