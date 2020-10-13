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

#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

struct Ellipse
{
  double long_radius;
  double short_radius;
  double yaw;
};

class LocalizationErrorMonitor
{
private:
  ros::NodeHandle nh_{""};
  ros::NodeHandle pnh_{"~"};
  ros::Subscriber pose_with_cov_sub_;
  ros::Publisher ellipse_marker_pub_;

  ros::Timer timer_;
  double scale_;
  double error_ellipse_size_;
  double warn_ellipse_size_;
  Ellipse ellipse_;
  diagnostic_updater::Updater updater_;

  void checkLocalizationAccuracy(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void onPoseWithCovariance(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & input_msg);
  visualization_msgs::Marker createEllipseMarker(
    const Ellipse & ellipse, const geometry_msgs::PoseWithCovarianceStamped & pose_with_cov);
  void onTimer(const ros::TimerEvent & event);

public:
  LocalizationErrorMonitor();
  ~LocalizationErrorMonitor() = default;
};
