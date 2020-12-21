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

#include <algorithm>
#include <limits>
#include <vector>

#include "boost/algorithm/clamp.hpp"
#include "boost/assert.hpp"
#include "boost/assign/list_of.hpp"
#include "boost/format.hpp"
#include "boost/geometry.hpp"
#include "boost/geometry/geometries/linestring.hpp"
#include "boost/geometry/geometries/point_xy.hpp"

#include "obstacle_stop_planner/adaptive_cruise_control.hpp"

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point, false>;
using Line = bg::model::linestring<Point>;

namespace
{
Point convertPointRosToBoost(const geometry_msgs::msg::Point & point)
{
  const Point point2d(point.x, point.y);
  return point2d;
}

geometry_msgs::msg::Vector3 rpyFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  geometry_msgs::msg::Vector3 rpy;
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;
  return rpy;
}

Polygon getPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & size,
  const double center_offset,
  const double l_margin = 0.0, const double w_margin = 0.0)
{
  Polygon obj_poly;
  geometry_msgs::msg::Vector3 obj_rpy = rpyFromQuat(pose.orientation);

  double l = size.x * std::cos(obj_rpy.y) + l_margin;
  double w = size.y * std::cos(obj_rpy.x) + w_margin;
  double co = center_offset;
  bg::exterior_ring(obj_poly) = boost::assign::list_of<Point>(l / 2.0 + co, w / 2.0)(
    -l / 2.0 + co, w / 2.0)(-l / 2.0 + co, -w / 2.0)(l / 2.0 + co, -w / 2.0)(l / 2.0 + co, w / 2.0);

  // rotate polygon
  bg::strategy::transform::rotate_transformer<bg::radian, double, 2, 2> rotate(
    -obj_rpy.z);  // original:clockwise rotation
  Polygon rotate_obj_poly;
  bg::transform(obj_poly, rotate_obj_poly, rotate);
  // translate polygon
  bg::strategy::transform::translate_transformer<double, 2, 2> translate(
    pose.position.x, pose.position.y);
  Polygon translate_obj_poly;
  bg::transform(rotate_obj_poly, translate_obj_poly, translate);
  return translate_obj_poly;
}

double getDistanceFromTwoPoint(
  const geometry_msgs::msg::Point & point1, const geometry_msgs::msg::Point & point2)
{
  const double dx = point1.x - point2.x;
  const double dy = point1.y - point2.y;
  const double dist = std::hypot(dx, dy);
  return dist;
}

constexpr double normalizeRadian(
  const double rad, const double min_rad = -boost::math::constants::pi<double>(),
  const double max_rad = boost::math::constants::pi<double>())
{
  const auto value = std::fmod(rad, 2 * boost::math::constants::pi<double>());
  if (min_rad < value && value <= max_rad) {
    return value;
  } else {
    return value - std::copysign(2 * boost::math::constants::pi<double>(), value);
  }
}

constexpr double sign(const double value)
{
  if (value > 0) {
    return 1.0;
  } else if (value < 0) {
    return -1.0;
  } else {
    return 0.0;
  }
}
}  // namespace

namespace motion_planning
{
AdaptiveCruiseController::AdaptiveCruiseController(
  const double vehicle_width, const double vehicle_length, const double wheel_base,
  const double front_overhang)
: Node("adaptive_cruise_controller"),
  vehicle_width_(vehicle_width),
  vehicle_length_(vehicle_length),
  wheel_base_(wheel_base),
  front_overhang_(front_overhang)
{
  // get parameter

  /* config */
  param_.min_behavior_stop_margin =
    declare_parameter("min_behavior_stop_margin", 2.0) + wheel_base_ + front_overhang_;
  param_.use_object_to_est_vel = declare_parameter("use_object_to_estimate_vel", true);
  param_.use_pcl_to_est_vel = declare_parameter("use_pcl_to_estimate_vel", true);
  param_.consider_obj_velocity = declare_parameter("consider_obj_velocity", true);

  /* paramter for acc */
  param_.obstacle_stop_velocity_thresh =
    declare_parameter("obstacle_stop_velocity_thresh", 1.0);
  param_.emergency_stop_acceleration = declare_parameter("emergency_stop_acceleration", -5.0);
  param_.emergency_stop_idling_time = declare_parameter("emergency_stop_idling_time", 0.5);
  param_.min_dist_stop = declare_parameter("min_dist_stop", 4.0);
  param_.max_standard_acceleration = declare_parameter("max_standard_acceleration", 0.5);
  param_.min_standard_acceleration = declare_parameter("min_standard_acceleration", -1.0);
  param_.standard_idling_time = declare_parameter("standard_idling_time", 0.5);
  param_.min_dist_standard = declare_parameter("min_dist_standard", 4.0);
  param_.obstacle_min_standard_acceleration =
    declare_parameter("obstacle_min_standard_acceleration", -1.5);
  param_.margin_rate_to_change_vel = declare_parameter("margin_rate_to_change_vel", 0.3);
  param_.lowpass_gain_ = declare_parameter("lowpass_gain_of_upper_velocity", 0.6);

  /* parameter for pid in acc */
  param_.p_coeff_pos = declare_parameter("p_coefficient_positive", 0.1);
  param_.p_coeff_neg = declare_parameter("p_coefficient_negative", 0.3);
  param_.d_coeff_pos = declare_parameter("d_coefficient_positive", 0.0);
  param_.d_coeff_neg = declare_parameter("d_coefficient_negative", 0.1);

  /* parameter for speed estimation of obstacle */
  param_.object_polygon_length_margin = declare_parameter("obect_polygon_length_margin", 2.0);
  param_.object_polygon_width_margin = declare_parameter("obect_polygon_width_margin", 0.5);
  param_.valid_est_vel_diff_time = declare_parameter("valid_estimated_vel_diff_time", 1.0);
  param_.valid_vel_que_time = declare_parameter("valid_vel_que_time", 0.5);
  param_.valid_est_vel_max = declare_parameter("valid_estimated_vel_max", 20.0);
  param_.valid_est_vel_min = declare_parameter("valid_estimated_vel_min", -20.0);
  param_.thresh_vel_to_stop = declare_parameter("thresh_vel_to_stop", 0.5);

  /* publisher */
  pub_debug_ = create_publisher<autoware_debug_msgs::msg::Float32MultiArrayStamped>(
    "debug_values",
    1);
}

void AdaptiveCruiseController::insertAdaptiveCruiseVelocity(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const int nearest_collision_point_idx,
  const geometry_msgs::msg::Pose self_pose, const pcl::PointXYZ & nearest_collision_point,
  const rclcpp::Time nearest_collision_point_time,
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr object_ptr,
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr current_velocity_ptr, bool * need_to_stop,
  autoware_planning_msgs::msg::Trajectory * output_trajectory)
{
  debug_values_.data.clear();
  debug_values_.data.resize(num_debug_values_, 0.0);

  const double current_velocity = current_velocity_ptr->twist.linear.x;
  double col_point_distance;
  double point_velocity;
  bool success_estm_vel = false;
  /*
  * calc distance to collsion point
  */
  calcDistanceToNearestPointOnPath(
    trajectory, nearest_collision_point_idx, self_pose, nearest_collision_point,
    &col_point_distance);

  /*
  * calc yaw of trajectory at collision point
  */
  const double traj_yaw = calcTrajYaw(trajectory, nearest_collision_point_idx);

  /*
  * estimate velocity of collsiion point
  */
  if (param_.use_pcl_to_est_vel) {
    if (estimatePointVelocityFromPcl(
        traj_yaw, nearest_collision_point, nearest_collision_point_time, &point_velocity))
    {
      success_estm_vel = true;
    }
  }

  if (param_.use_object_to_est_vel) {
    if (estimatePointVelocityFromObject(
        object_ptr, traj_yaw, nearest_collision_point, &point_velocity))
    {
      success_estm_vel = true;
    }
  }

  if (!success_estm_vel) {
    // if failed to estimate velocity, need to stop
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Failed to estimate velocity of forward vehicle. Insert stop line.");
    *need_to_stop = true;
    prev_upper_velocity_ = current_velocity;  // reset prev_upper_velocity
    pub_debug_->publish(debug_values_);
    return;
  }

  // calculate max(target) velocity of self
  const double upper_velocity =
    calcUpperVelocity(col_point_distance, point_velocity, current_velocity);
  pub_debug_->publish(debug_values_);

  if (upper_velocity <= param_.thresh_vel_to_stop) {
    // if upper velocity is too low, need to stop
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Upper velocity is too low. Insert stop line.");
    *need_to_stop = true;
    prev_upper_velocity_ = current_velocity;  // reset prev_upper_velocity
    return;
  }

  /*
  * insert max velocity
  */
  insertMaxVelocityToPath(current_velocity, upper_velocity, col_point_distance, output_trajectory);
  *need_to_stop = false;
}

void AdaptiveCruiseController::calcDistanceToNearestPointOnPath(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const int nearest_point_idx,
  const geometry_msgs::msg::Pose & self_pose, const pcl::PointXYZ & nearest_collision_point,
  double * distance)
{
  if (trajectory.points.size() == 0) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "input path is too short(size=0)");
    *distance = 0;
    return;
  }

  // get self polygon
  geometry_msgs::msg::Vector3 self_size;
  self_size.x = vehicle_length_;
  self_size.y = vehicle_width_;
  double self_offset = (wheel_base_ + front_overhang_) - vehicle_length_ / 2.0;
  Polygon self_poly = getPolygon(self_pose, self_size, self_offset);

  // get nearest point
  Point nearest_point2d(nearest_collision_point.x, nearest_collision_point.y);

  if (nearest_point_idx <= 2) {
    // if too nearest collision point, return direct distance
    *distance = boost::geometry::distance(self_poly, nearest_point2d);
    debug_values_.data.at(DBGVAL::FORWARD_OBJ_DISTANCE) = *distance;
    return;
  }

  /* get total distance to collision point */
  double dist_to_point = 0;
  // get distance from self to next nearest point
  dist_to_point += boost::geometry::distance(
    convertPointRosToBoost(self_pose.position),
    convertPointRosToBoost(trajectory.points.at(1).pose.position));

  // add distance from next self-nearest-point(=idx:0) to prev point of nearest_point_idx
  for (int i = 1; i < nearest_point_idx - 1; i++) {
    dist_to_point += getDistanceFromTwoPoint(
      trajectory.points.at(i).pose.position, trajectory.points.at(i + 1).pose.position);
  }

  // add distance from nearest_collision_point to prev point of nearest_point_idx
  dist_to_point += boost::geometry::distance(
    nearest_point2d,
    convertPointRosToBoost(trajectory.points.at(nearest_point_idx - 1).pose.position));

  // subtract base_link to front
  dist_to_point -= param_.min_behavior_stop_margin;

  *distance = std::max(0.0, dist_to_point);
  debug_values_.data.at(DBGVAL::FORWARD_OBJ_DISTANCE) = *distance;
}

double AdaptiveCruiseController::calcTrajYaw(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const int collsion_point_idx)
{
  return tf2::getYaw(trajectory.points.at(collsion_point_idx).pose.orientation);
}

bool AdaptiveCruiseController::estimatePointVelocityFromObject(
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr object_ptr,
  const double traj_yaw,
  const pcl::PointXYZ & nearest_collision_point, double * velocity)
{
  geometry_msgs::msg::Point nearest_collsion_p_ros;
  nearest_collsion_p_ros.x = nearest_collision_point.x;
  nearest_collsion_p_ros.y = nearest_collision_point.y;
  nearest_collsion_p_ros.z = nearest_collision_point.z;

  /* get object velocity, and current yaw */
  bool get_obj = false;
  double obj_vel;
  double obj_yaw;
  const Point collsion_point_2d = convertPointRosToBoost(nearest_collsion_p_ros);
  for (const auto & obj : object_ptr->objects) {
    const Polygon obj_poly = getPolygon(
      obj.state.pose_covariance.pose, obj.shape.dimensions, 0.0,
      param_.object_polygon_length_margin, param_.object_polygon_width_margin);
    if (boost::geometry::distance(obj_poly, collsion_point_2d) <= 0) {
      obj_vel = obj.state.twist_covariance.twist.linear.x;
      obj_yaw = tf2::getYaw(obj.state.pose_covariance.pose.orientation);
      get_obj = true;
      break;
    }
  }

  if (get_obj) {
    *velocity = obj_vel * std::cos(obj_yaw - traj_yaw);
    debug_values_.data.at(DBGVAL::ESTIMATED_VEL_OBJ) = *velocity;
    return true;
  } else {
    return false;
  }
}

bool AdaptiveCruiseController::estimatePointVelocityFromPcl(
  const double traj_yaw, const pcl::PointXYZ & nearest_collision_point,
  const rclcpp::Time & nearest_collision_point_time, double * velocity)
{
  geometry_msgs::msg::Point nearest_collsion_p_ros;
  nearest_collsion_p_ros.x = nearest_collision_point.x;
  nearest_collsion_p_ros.y = nearest_collision_point.y;
  nearest_collsion_p_ros.z = nearest_collision_point.z;

  /* estimate velocity */
  const double p_dt = nearest_collision_point_time.seconds() - prev_collsion_point_time_.seconds();

  // if get same pointcloud with previous step,
  // skip estimate process
  if (std::fabs(p_dt) > std::numeric_limits<double>::epsilon()) {
    // valid time check
    if (p_dt < 0 || param_.valid_est_vel_diff_time < p_dt) {
      prev_collsion_point_time_ = nearest_collision_point_time;
      prev_collsion_point_ = nearest_collision_point;
      prev_collsion_point_valid_ = true;
      return false;
    }
    const double p_dx = nearest_collision_point.x - prev_collsion_point_.x;
    const double p_dy = nearest_collision_point.y - prev_collsion_point_.y;
    const double p_dist = std::hypot(p_dx, p_dy);
    const double p_yaw = std::atan2(p_dy, p_dx);
    const double p_vel = p_dist / p_dt;
    const double est_velocity = p_vel * std::cos(p_yaw - traj_yaw);
    // valid velocity check
    if (est_velocity <= param_.valid_est_vel_min || param_.valid_est_vel_max <= est_velocity) {
      prev_collsion_point_time_ = nearest_collision_point_time;
      prev_collsion_point_ = nearest_collision_point;
      prev_collsion_point_valid_ = true;
      est_vel_que_.clear();
      return false;
    }

    // append new velocity and remove old velocity from que
    registerQueToVelocity(est_velocity, nearest_collision_point_time);
  }

  // calc average(median) velocity from que
  *velocity = getMedianVel(est_vel_que_);
  debug_values_.data.at(DBGVAL::ESTIMATED_VEL_PCL) = *velocity;

  prev_collsion_point_time_ = nearest_collision_point_time;
  prev_collsion_point_ = nearest_collision_point;
  prev_collsion_point_valid_ = true;
  return true;
}

double AdaptiveCruiseController::calcUpperVelocity(
  const double dist_to_col, const double obj_vel, const double self_vel)
{
  debug_values_.data.at(DBGVAL::ESTIMATED_VEL_FINAL) = obj_vel;
  if (obj_vel < param_.obstacle_stop_velocity_thresh) {
    // stop by static obstacle
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "The velocity of forward vehicle is too low. Insert stop line.");
    return 0.0;
  }

  const double thresh_dist = calcThreshDistToForwardObstacle(self_vel, obj_vel);
  if (thresh_dist >= dist_to_col) {
    // emergency stop
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Forward vehicle is too close. Insert stop line.");
    return 0.0;
  }

  const double upper_velocity = calcTargetVelocityByPID(self_vel, dist_to_col, obj_vel);
  const double lowpass_upper_velocity =
    lowpass_filter(upper_velocity, prev_upper_velocity_, param_.lowpass_gain_);
  prev_upper_velocity_ = lowpass_upper_velocity;
  debug_values_.data.at(DBGVAL::UPPER_VEL) = lowpass_upper_velocity;
  return lowpass_upper_velocity;
}

double AdaptiveCruiseController::calcThreshDistToForwardObstacle(
  const double current_vel, const double obj_vel)
{
  const double current_vel_min = std::max(1.0, std::fabs(current_vel));
  const double obj_vel_min = std::max(0.0, obj_vel);
  const double minimum_distance = param_.min_dist_stop;
  const double idling_distance = current_vel_min * param_.emergency_stop_idling_time;
  const double braking_distance =
    (-1.0 * current_vel_min * current_vel_min) / (2.0 * param_.emergency_stop_acceleration);
  const double obj_braking_distance =
    (-1.0 * obj_vel_min * obj_vel_min) / (2.0 * param_.emergency_stop_acceleration);

  return minimum_distance + std::max(
    0.0, idling_distance + braking_distance -
    obj_braking_distance * param_.consider_obj_velocity);
}

double AdaptiveCruiseController::calcBaseDistToForwardObstacle(
  const double current_vel, const double obj_vel)
{
  const double obj_vel_min = std::max(0.0, obj_vel);
  const double minimum_distance = param_.min_dist_standard;
  const double idling_distance = current_vel * param_.standard_idling_time;
  const double braking_distance =
    (-1.0 * current_vel * current_vel) / (2.0 * param_.min_standard_acceleration);
  const double obj_braking_distance =
    (-1.0 * obj_vel_min * obj_vel_min) / (2.0 * param_.obstacle_min_standard_acceleration);
  return minimum_distance + std::max(
    0.0, idling_distance + braking_distance -
    obj_braking_distance * param_.consider_obj_velocity);
}

double AdaptiveCruiseController::calcTargetVelocity_P(
  const double target_dist, const double current_dist)
{
  const double diff_dist = current_dist - target_dist;
  double add_vel_p;
  if (diff_dist >= 0) {
    add_vel_p = diff_dist * param_.p_coeff_pos;
  } else {
    add_vel_p = diff_dist * param_.p_coeff_neg;
  }
  return add_vel_p;
}

double AdaptiveCruiseController::calcTargetVelocity_I(
  const double target_dist, const double current_dist)
{
  // not implemented
  return 0.0;
}

double AdaptiveCruiseController::calcTargetVelocity_D(
  const double target_dist, const double current_dist)
{
  if (rclcpp::Clock().now().seconds() - prev_target_vehicle_time_ >= param_.d_coeff_valid_time) {
    // invalid time(prev is too old)
    return 0.0;
  }

  double diff_vel = (target_dist - prev_target_vehicle_dist_) /
    (rclcpp::Clock().now().seconds() - prev_target_vehicle_time_);

  if (std::fabs(diff_vel) >= param_.d_coeff_valid_diff_vel) {
    // invalid(discontinuous) diff_vel
    return 0.0;
  }

  double add_vel_d = 0;

  add_vel_d = diff_vel;
  if (add_vel_d >= 0) {diff_vel *= param_.d_coeff_pos;}
  if (add_vel_d < 0) {diff_vel *= param_.d_coeff_neg;}
  add_vel_d = boost::algorithm::clamp(add_vel_d, -param_.d_max_vel_norm, param_.d_max_vel_norm);

  // add buffer
  prev_target_vehicle_dist_ = current_dist;
  prev_target_vehicle_time_ = rclcpp::Clock().now().seconds();

  return add_vel_d;
}

double AdaptiveCruiseController::calcTargetVelocityByPID(
  const double current_vel, const double current_dist, const double obj_vel)
{
  const double target_dist = calcBaseDistToForwardObstacle(current_vel, obj_vel);
  RCLCPP_DEBUG_STREAM(get_logger(), "[adaptive cruise control] target_dist" << target_dist);

  const double add_vel_p = calcTargetVelocity_P(target_dist, current_dist);
  //** I is not implemented **
  const double add_vel_i = calcTargetVelocity_I(target_dist, current_dist);
  const double add_vel_d = calcTargetVelocity_D(target_dist, current_dist);

  double target_vel = current_vel + add_vel_p + add_vel_i + add_vel_d;
  debug_values_.data.at(DBGVAL::CURRENT_VEL) = current_vel;
  debug_values_.data.at(DBGVAL::UPPER_VEL_P) = add_vel_p;
  debug_values_.data.at(DBGVAL::UPPER_VEL_I) = add_vel_i;
  debug_values_.data.at(DBGVAL::UPPER_VEL_D) = add_vel_d;
  debug_values_.data.at(DBGVAL::UPPER_VEL_RAW) = target_vel;
  return target_vel;
}

void AdaptiveCruiseController::insertMaxVelocityToPath(
  const double current_vel, const double target_vel, const double dist_to_collsion_point,
  autoware_planning_msgs::msg::Trajectory * output_trajectory)
{
  double target_acc = sign(target_vel - current_vel) *
    ((target_vel - current_vel) * (target_vel - current_vel)) /
    (2 * dist_to_collsion_point * (1 - param_.margin_rate_to_change_vel));
  double margin_to_insert = dist_to_collsion_point * param_.margin_rate_to_change_vel;

  const double clipped_acc = boost::algorithm::clamp(
    target_acc, param_.min_standard_acceleration, param_.max_standard_acceleration);
  double pre_vel = current_vel;
  double total_dist = -margin_to_insert;
  for (size_t i = 1; i < output_trajectory->points.size(); i++) {
    // calc velocity of each point by gradient deceleration
    const auto current_p = output_trajectory->points[i];
    const auto prev_p = output_trajectory->points[i - 1];
    const double p_dist = getDistanceFromTwoPoint(current_p.pose.position, prev_p.pose.position);
    total_dist += p_dist;
    if (current_p.twist.linear.x > target_vel && total_dist >= 0) {
      double diff_vel;
      if (std::fabs(clipped_acc) < 1e-05) {
        diff_vel = 0.0;
      } else {
        diff_vel = std::sqrt(2 * p_dist / std::fabs(clipped_acc)) * clipped_acc;
      }
      double next_pre_vel;
      if (target_acc >= 0) {
        next_pre_vel = std::min(pre_vel + diff_vel, target_vel);
      } else {
        next_pre_vel = std::max(pre_vel + diff_vel, target_vel);
      }

      if (total_dist >= 0) {
        const double max_velocity = std::max(target_vel, next_pre_vel);
        if (output_trajectory->points[i].twist.linear.x > max_velocity) {
          output_trajectory->points[i].twist.linear.x = max_velocity;
        }
      }
      pre_vel = next_pre_vel;
    }
  }
}

void AdaptiveCruiseController::registerQueToVelocity(
  const double vel,
  const rclcpp::Time & vel_time)
{
  // remove old msg from que
  std::vector<int> delete_idxs;
  for (size_t i = 0; i < est_vel_que_.size(); i++) {
    if (
      rclcpp::Clock().now().seconds() - est_vel_que_.at(i).header.stamp.sec >
      param_.valid_vel_que_time)
    {
      delete_idxs.push_back(i);
    }
  }
  for (int delete_idx = delete_idxs.size() - 1; 0 <= delete_idx; --delete_idx) {
    est_vel_que_.erase(est_vel_que_.begin() + delete_idxs.at(delete_idx));
  }

  // append new que
  geometry_msgs::msg::TwistStamped new_vel;
  new_vel.header.stamp = vel_time;
  new_vel.twist.linear.x = vel;
  est_vel_que_.emplace_back(new_vel);
}

double AdaptiveCruiseController::getMedianVel(
  const std::vector<geometry_msgs::msg::TwistStamped> vel_que)
{
  if (vel_que.size() == 0) {
    RCLCPP_WARN_STREAM(get_logger(), "size of vel que is 0. Something has wrong.");
    return 0.0;
  }

  std::vector<double> raw_vel_que;
  for (const auto vel : vel_que) {
    raw_vel_que.emplace_back(vel.twist.linear.x);
  }

  double med_vel;
  if (raw_vel_que.size() % 2 == 0) {
    size_t med1 = (raw_vel_que.size()) / 2 - 1;
    size_t med2 = (raw_vel_que.size()) / 2;
    std::nth_element(raw_vel_que.begin(), raw_vel_que.begin() + med1, raw_vel_que.end());
    const double vel1 = raw_vel_que[med1];
    std::nth_element(raw_vel_que.begin(), raw_vel_que.begin() + med2, raw_vel_que.end());
    const double vel2 = raw_vel_que[med2];
    med_vel = (vel1 + vel2) / 2;
  } else {
    size_t med = (raw_vel_que.size() - 1) / 2;
    std::nth_element(raw_vel_que.begin(), raw_vel_que.begin() + med, raw_vel_que.end());
    med_vel = raw_vel_que[med];
  }

  return med_vel;
}

double AdaptiveCruiseController::lowpass_filter(
  const double current_value, const double prev_value, const double gain)
{
  return gain * prev_value + (1.0 - gain) * current_value;
}

}  // namespace motion_planning
