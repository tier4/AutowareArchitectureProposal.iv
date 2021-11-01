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

#include "map_based_prediction.hpp"

#include <algorithm>
#include <chrono>
#include <vector>

#include "autoware_utils/autoware_utils.hpp"
#include "cubic_spline.hpp"
#include "tf2/utils.h"

MapBasedPrediction::MapBasedPrediction(
  double interpolating_resolution, double time_horizon, double sampling_delta_time)
: interpolating_resolution_(interpolating_resolution),
  time_horizon_(time_horizon),
  sampling_delta_time_(sampling_delta_time)
{
}

bool MapBasedPrediction::doPrediction(
  const DynamicObjectWithLanesArray & in_objects,
  std::vector<autoware_perception_msgs::msg::DynamicObject> & out_objects)
{
  for (auto & object_with_lanes : in_objects.objects) {
    autoware_perception_msgs::msg::DynamicObject tmp_object;
    tmp_object = object_with_lanes.object;
    for (size_t path_id = 0; path_id < object_with_lanes.lanes.size(); ++path_id) {
      std::vector<double> tmp_x;
      std::vector<double> tmp_y;
      std::vector<geometry_msgs::msg::Pose> path = object_with_lanes.lanes.at(path_id);
      for (size_t i = 0; i < path.size(); i++) {
        if (i > 0) {
          double dist = autoware_utils::calcDistance2d(path.at(i), path.at(i - 1));
          if (dist < interpolating_resolution_) {
            continue;
          }
        }
        tmp_x.push_back(path.at(i).position.x);
        tmp_y.push_back(path.at(i).position.y);
      }

      // Generate Splined Trajectory Arc-Length Position and Yaw angle
      Spline2D spline2d(tmp_x, tmp_y);
      std::vector<geometry_msgs::msg::Point> interpolated_points;
      std::vector<double> interpolated_yaws;
      for (double s = 0.0; s < spline2d.s.back(); s += interpolating_resolution_) {
        std::array<double, 2> point1 = spline2d.calc_position(s);
        geometry_msgs::msg::Point g_point;
        g_point.x = point1[0];
        g_point.y = point1[1];
        g_point.z = object_with_lanes.object.state.pose_covariance.pose.position.z;
        interpolated_points.push_back(g_point);
        interpolated_yaws.push_back(spline2d.calc_yaw(s));
      }

      // calculate initial position in Frenet coordinate
      // Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame
      // Path Planning for Highly Automated Driving on Embedded GPUs
      geometry_msgs::msg::Point object_point =
        object_with_lanes.object.state.pose_covariance.pose.position;

      const size_t nearest_segment_idx =
        autoware_utils::findNearestSegmentIndex(interpolated_points, object_point);
      const double l = autoware_utils::calcLongitudinalOffsetToSegment(
        interpolated_points, nearest_segment_idx, object_point);
      const double current_s_position =
        autoware_utils::calcSignedArcLength(interpolated_points, 0, nearest_segment_idx) + l;
      if (current_s_position > spline2d.s.back()) {
        continue;
      }
      std::array<double, 2> s_point = spline2d.calc_position(current_s_position);
      geometry_msgs::msg::Point nearest_point =
        autoware_utils::createPoint(s_point[0], s_point[1], object_point.z);
      double current_d_position = autoware_utils::calcDistance2d(nearest_point, object_point);
      const double lane_yaw = spline2d.calc_yaw(current_s_position);
      const std::vector<double> origin_v = {std::cos(lane_yaw), std::sin(lane_yaw)};
      const std::vector<double> object_v = {
        object_point.x - nearest_point.x, object_point.y - nearest_point.y};
      const double cross2d = object_v.at(0) * origin_v.at(1) - object_v.at(1) * origin_v.at(0);
      if (cross2d < 0) {
        current_d_position *= -1;
      }

      // Does not consider orientation of twist since predicting lane-direction
      const double current_d_velocity =
        object_with_lanes.object.state.twist_covariance.twist.linear.y;
      const double current_s_velocity =
        std::fabs(object_with_lanes.object.state.twist_covariance.twist.linear.x);

      // Predict Path
      autoware_perception_msgs::msg::PredictedPath predicted_path;
      getPredictedPath(
        object_point.z, current_d_position, current_d_velocity, current_s_position,
        current_s_velocity, in_objects.header, spline2d, predicted_path);
      // substitute confidence value
      predicted_path.confidence = object_with_lanes.confidence.at(path_id);

      // Check if the path is already generated
      const double CLOSE_PATH_THRESHOLD = 0.1;
      bool duplicate_flag = false;
      for (const auto & prev_path : tmp_object.state.predicted_paths) {
        const auto prev_path_end = prev_path.path.back().pose.pose.position;
        const auto current_path_end = predicted_path.path.back().pose.pose.position;
        const double dist = autoware_utils::calcDistance2d(prev_path_end, current_path_end);
        if (dist < CLOSE_PATH_THRESHOLD) {
          duplicate_flag = true;
          break;
        }
      }

      if (duplicate_flag) {
        continue;
      }

      tmp_object.state.predicted_paths.push_back(predicted_path);
    }
    normalizeLikelihood(tmp_object.state.predicted_paths);
    out_objects.push_back(tmp_object);
  }
  return true;
}

bool MapBasedPrediction::doLinearPrediction(
  const autoware_perception_msgs::msg::DynamicObjectArray & in_objects,
  std::vector<autoware_perception_msgs::msg::DynamicObject> & out_objects)
{
  for (const auto & object : in_objects.objects) {
    autoware_perception_msgs::msg::PredictedPath path;
    getLinearPredictedPath(
      object.state.pose_covariance.pose, object.state.twist_covariance.twist, in_objects.header,
      path);
    autoware_perception_msgs::msg::DynamicObject tmp_object;
    tmp_object = object;
    tmp_object.state.predicted_paths.push_back(path);
    out_objects.push_back(tmp_object);
  }

  return true;
}

void MapBasedPrediction::normalizeLikelihood(
  std::vector<autoware_perception_msgs::msg::PredictedPath> & paths)
{
  // might not be the smartest way
  double sum_confidence = 0.0;
  for (const auto & path : paths) {
    sum_confidence += path.confidence;
  }

  for (auto & path : paths) {
    path.confidence = path.confidence / sum_confidence;
  }
}

bool MapBasedPrediction::getPredictedPath(
  const double height, const double current_d_position, const double current_d_velocity,
  const double current_s_position, const double current_s_velocity,
  const std_msgs::msg::Header & origin_header, Spline2D & spline2d,
  autoware_perception_msgs::msg::PredictedPath & path)
{
  // Quintic polynomial for d
  // A = np.array([[T**3, T**4, T**5],
  //               [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
  //               [6 * T, 12 * T ** 2, 20 * T ** 3]])
  // A_inv = np.matrix([[10/(T**3), -4/(T**2), 1/(2*T)],
  //                    [-15/(T**4), 7/(T**3), -1/(T**2)],
  //                    [6/(T**5), -3/(T**4),  1/(2*T**3)]])
  // b = np.matrix([[xe - self.a0 - self.a1 * T - self.a2 * T**2],
  //                [vxe - self.a1 - 2 * self.a2 * T],
  //                [axe - 2 * self.a2]])
  double target_d_position = 0;

  double t = time_horizon_;
  Eigen::Matrix3d a_3_inv;
  a_3_inv << 10 / std::pow(t, 3), -4 / std::pow(t, 2), 1 / (2 * t), -15 / std::pow(t, 4),
    7 / std::pow(t, 3), -1 / std::pow(t, 2), 6 / std::pow(t, 5), -3 / std::pow(t, 4),
    1 / (2 * std::pow(t, 3));

  double target_d_velocity = current_d_velocity;
  double target_d_acceleration = 0;
  Eigen::Vector3d b_3;
  b_3 << target_d_position - current_d_position - current_d_velocity * t,
    target_d_velocity - current_d_velocity, target_d_acceleration;

  Eigen::Vector3d x_3;
  x_3 = a_3_inv * b_3;

  // Quadric polynomial
  // A_inv = np.matrix([[1/(T**2), -1/(3*T)],
  //                         [-1/(2*T**3), 1/(4*T**2)]])
  // b = np.matrix([[vxe - self.a1 - 2 * self.a2 * T],
  //               [axe - 2 * self.a2]])
  Eigen::Matrix2d a_2_inv;
  a_2_inv << 1 / std::pow(t, 2), -1 / (3 * t), -1 / (2 * std::pow(t, 3)), 1 / (4 * std::pow(t, 2));
  double target_s_velocity = current_s_velocity;
  Eigen::Vector2d b_2;
  b_2 << target_s_velocity - current_s_velocity, 0;
  Eigen::Vector2d x_2;
  x_2 = a_2_inv * b_2;

  // sampling points from calculated path
  double dt = sampling_delta_time_;
  std::vector<double> d_vec;
  for (double i = 0; i < t; i += dt) {
    const double calculated_d = current_d_position + current_d_velocity * i + 0 * 2 * i * i +
                                x_3(0) * i * i * i + x_3(1) * i * i * i * i +
                                x_3(2) * i * i * i * i * i;
    const double calculated_s = current_s_position + current_s_velocity * i + 2 * 0 * i * i +
                                x_2(0) * i * i * i + x_2(1) * i * i * i * i;

    geometry_msgs::msg::PoseWithCovarianceStamped tmp_point;
    if (calculated_s > spline2d.s.back()) {
      break;
    }
    std::array<double, 2> p = spline2d.calc_position(calculated_s);
    double yaw = spline2d.calc_yaw(calculated_s);
    tmp_point.pose.pose.position.x = p[0] + std::cos(yaw - M_PI / 2.0) * calculated_d;
    tmp_point.pose.pose.position.y = p[1] + std::sin(yaw - M_PI / 2.0) * calculated_d;
    tmp_point.pose.pose.position.z = height;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    tmp_point.pose.pose.orientation = tf2::toMsg(quat);
    tmp_point.header = origin_header;
    tmp_point.header.stamp = rclcpp::Time(origin_header.stamp) + rclcpp::Duration::from_seconds(i);
    path.path.push_back(tmp_point);
  }

  return true;
}

void MapBasedPrediction::getLinearPredictedPath(
  const geometry_msgs::msg::Pose & object_pose, const geometry_msgs::msg::Twist & object_twist,
  const std_msgs::msg::Header & origin_header,
  autoware_perception_msgs::msg::PredictedPath & predicted_path)
{
  const double & sampling_delta_time = sampling_delta_time_;
  const double & time_horizon = time_horizon_;
  const double ep = 0.001;

  for (double dt = 0.0; dt < time_horizon + ep; dt += sampling_delta_time) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_stamped;
    pose_cov_stamped.header = origin_header;
    pose_cov_stamped.header.stamp =
      rclcpp::Time(origin_header.stamp) + rclcpp::Duration::from_seconds(dt);
    geometry_msgs::msg::Pose object_frame_pose;
    geometry_msgs::msg::Pose world_frame_pose;
    object_frame_pose.position.x = object_twist.linear.x * dt;
    object_frame_pose.position.y = object_twist.linear.y * dt;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.0);
    object_frame_pose.orientation = tf2::toMsg(quat);
    tf2::Transform tf_object2future;
    tf2::Transform tf_world2object;
    tf2::Transform tf_world2future;

    tf2::fromMsg(object_pose, tf_world2object);
    tf2::fromMsg(object_frame_pose, tf_object2future);
    tf_world2future = tf_world2object * tf_object2future;
    tf2::toMsg(tf_world2future, world_frame_pose);
    pose_cov_stamped.pose.pose = world_frame_pose;
    predicted_path.path.push_back(pose_cov_stamped);
  }

  predicted_path.confidence = 1.0;
}
