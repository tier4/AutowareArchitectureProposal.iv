// Copyright 2021 Tier IV, Inc.
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

#ifndef VELOCITY_CONTROLLER__SMOOTH_STOP_HPP_
#define VELOCITY_CONTROLLER__SMOOTH_STOP_HPP_

#include <rclcpp/rclcpp.hpp>

#include <boost/optional.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

/**
 * @brief Smooth stop class to implement vehicle specific deceleration profiles
 */
class SmoothStop
{
public:
  /**
   * @brief initialize the state of the smooth stop
   * @param [in] pred_vel_in_target predicted ego velocity when the stop command will be executed
   * @param [in] pred_stop_dist predicted stop distance when the stop command will be executed
   */
  void init(const double pred_vel_in_target, const double pred_stop_dist)
  {
    weak_acc_time_ = rclcpp::Clock{RCL_ROS_TIME}.now();

    // when distance to stopline is near the car
    if (pred_stop_dist < std::numeric_limits<double>::epsilon()) {
      strong_acc_ = params_.min_strong_acc;
      return;
    }

    strong_acc_ = -std::pow(pred_vel_in_target, 2) / (2 * pred_stop_dist);
    strong_acc_ = std::max(std::min(strong_acc_, params_.max_strong_acc), params_.min_strong_acc);
  }

  /**
   * @brief set the parameters of this smooth stop
   * @param [in] max_strong_acc maximum strong acceleration value [m/s²]
   * @param [in] min_strong_acc minumum strong acceleration value [m/s²]
   * @param [in] weak_acc weak acceleration value [m/s²]
   * @param [in] weak_stop_acc weak stopping acceleration value [m/s²]
   * @param [in] strong_stop_acc strong stopping acceleration value [m/s²]
   * @param [in] min_fast_vel minumum velocity to consider ego to be running fast [m/s]
   * @param [in] min_running_vel minimum velocity to consider ego to be running [m/s]
   * @param [in] min_running_acc minimum acceleration to consider ego to be running [m/s]
   * @param [in] weak_stop_time time allowed for stopping with a weak acceleration [s]
   * @param [in] weak_stop_dist distance to the stop point bellow which a weak accel is applied [m]
   * @param [in] strong_stop_dist distance to the stop point bellow which a strong accel is applied
   * [m]
   */
  void setParams(
    double max_strong_acc, double min_strong_acc, double weak_acc, double weak_stop_acc,
    double strong_stop_acc, double min_fast_vel, double min_running_vel, double min_running_acc,
    double weak_stop_time, double weak_stop_dist, double strong_stop_dist)
  {
    params_.max_strong_acc = max_strong_acc;
    params_.min_strong_acc = min_strong_acc;
    params_.weak_acc = weak_acc;
    params_.weak_stop_acc = weak_stop_acc;
    params_.strong_stop_acc = strong_stop_acc;

    params_.min_fast_vel = min_fast_vel;
    params_.min_running_vel = min_running_vel;
    params_.min_running_acc = min_running_acc;
    params_.weak_stop_time = weak_stop_time;

    params_.weak_stop_dist = weak_stop_dist;
    params_.strong_stop_dist = strong_stop_dist;
  }

  /**
   * @brief predict time when car stops by fitting some latest observed velocity history
   *        with linear function (v = at + b)
   * @param [in] vel_hist history of previous ego velocities as (rclcpp::Time, double[m/s]) pairs
   */
  boost::optional<double> calcTimeToStop(
    const std::vector<std::pair<rclcpp::Time, double>> & vel_hist) const
  {
    // return when vel_hist is empty
    const size_t vel_hist_size = vel_hist.size();
    if (vel_hist_size == 0) {
      return {};
    }

    // calculate some variables for fitting
    const rclcpp::Time current_ros_time = rclcpp::Clock{RCL_ROS_TIME}.now();
    double mean_t = 0.0;
    double mean_v = 0.0;
    double sum_tv = 0.0;
    double sum_tt = 0.0;
    for (const auto & vel : vel_hist) {
      const double t = (vel.first - current_ros_time).seconds();
      const double v = vel.second;

      mean_t += t / vel_hist_size;
      mean_v += v / vel_hist_size;
      sum_tv += t * v;
      sum_tt += t * t;
    }

    // return when gradient a (of v = at + b) cannot be calculated.
    // See the following calculation of a
    if (
      std::abs(vel_hist_size * mean_t * mean_t - sum_tt) < std::numeric_limits<double>::epsilon()) {
      return {};
    }

    // calculate coefficients of linear function (v = at + b)
    const double a =
      (vel_hist_size * mean_t * mean_v - sum_tv) / (vel_hist_size * mean_t * mean_t - sum_tt);
    const double b = mean_v - a * mean_t;

    // return when v is independent of time (v = b)
    if (std::abs(a) < std::numeric_limits<double>::epsilon()) {
      return {};
    }

    // calculate time to stop by substituting v = 0 for v = at + b
    const double time_to_stop = -b / a;
    if (time_to_stop > 0) {
      return time_to_stop;
    }

    return {};
  }

  /**
   * @brief calculate accel command while stopping
   *        Decrease velocity with strong_acc_,
   *        then loose brake pedal with params_.weak_acc to stop smoothly
   *        If the car is still running, input params_.weak_stop_acc
   *        and then params_.strong_stop_acc in steps not to exceed stopline too much
   * @param [in] stop_dist distance left to travel before stopping [m]
   * @param [in] current_vel current velocity of ego [m/s]
   * @param [in] current_acc current acceleration of ego [m/s²]
   * @param [in] vel_hist history of previous ego velocities as (rclcpp::Time, double[m/s]) pairs
   * @param [in] delay_time assumed time delay when the stop command will actually be executed
   */
  double calculate(
    const double stop_dist, const double current_vel, const double current_acc,
    const std::vector<std::pair<rclcpp::Time, double>> & vel_hist, const double delay_time)
  {
    // predict time to stop
    const auto time_to_stop = calcTimeToStop(vel_hist);

    // calculate some flags
    const bool is_fast_vel = std::abs(current_vel) > params_.min_fast_vel;
    const bool is_running = std::abs(current_vel) > params_.min_running_vel ||
                            std::abs(current_acc) > params_.min_running_acc;

    // when exceeding the stopline (stop_dist is negative in these cases.)
    if (stop_dist < params_.strong_stop_dist) {  // when exceeding the stopline much
      return params_.strong_stop_acc;
    } else if (stop_dist < params_.weak_stop_dist) {  // when exceeding the stopline a bit
      return params_.weak_stop_acc;
    }

    // when the car is running
    if (is_running) {
      // when the car will not stop in a certain time
      if (time_to_stop && *time_to_stop > params_.weak_stop_time + delay_time) {
        return strong_acc_;
      } else if (!time_to_stop && is_fast_vel) {
        return strong_acc_;
      }

      weak_acc_time_ = rclcpp::Clock{RCL_ROS_TIME}.now();
      return params_.weak_acc;
    }

    // for 0.5 seconds after the car stopped
    if ((rclcpp::Clock{RCL_ROS_TIME}.now() - weak_acc_time_).seconds() < 0.5) {
      return params_.weak_acc;
    }

    // when the car is not running
    return params_.strong_stop_acc;
  }

private:
  struct Params
  {
    double max_strong_acc;
    double min_strong_acc;
    double weak_acc;
    double weak_stop_acc;
    double strong_stop_acc;

    double min_fast_vel;
    double min_running_vel;
    double min_running_acc;
    double weak_stop_time;

    double weak_stop_dist;
    double strong_stop_dist;
  };
  Params params_;

  double strong_acc_;
  rclcpp::Time weak_acc_time_;
};

#endif  // VELOCITY_CONTROLLER__SMOOTH_STOP_HPP_
