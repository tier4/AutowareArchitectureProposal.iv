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
 *
 *
 * v1.0 Yukihiro Saito
 */

#pragma once
#include <kalman_filter/kalman_filter.hpp>
#include "autoware_perception_msgs/DynamicObject.h"
#include "tracker_base.hpp"

class NormalVehicleTracker : public Tracker
{
private:
  autoware_perception_msgs::DynamicObject object_;

private:
  KalmanFilter ekf_;
  ros::Time last_update_time_;
  enum IDX {
    X = 0,
    Y = 1,
    YAW = 2,
    VX = 3,
    WZ = 4,
  };
  char dim_x_ = 5;
  float process_noise_covariance_pos_x_;
  float process_noise_covariance_pos_y_;
  float process_noise_covariance_yaw_;
  float process_noise_covariance_wz_;
  float process_noise_covariance_vx_;
  float initial_measurement_noise_covariance_vx_;
  float initial_measurement_noise_covariance_wz_;
  // if use_measurement_covariance_ is false, use following params
  bool use_measurement_covariance_;
  float measurement_noise_covariance_pos_x_;
  float measurement_noise_covariance_pos_y_;
  float measurement_noise_covariance_yaw_;
  float initial_measurement_noise_covariance_pos_x_;
  float initial_measurement_noise_covariance_pos_y_;
  float initial_measurement_noise_covariance_yaw_;

  double max_vx_;
  double max_wz_;

private:
  struct BoundingBox
  {
    double width;
    double length;
    double height;
  };
  BoundingBox bounding_box_;

public:
  NormalVehicleTracker(
    const ros::Time & time, const autoware_perception_msgs::DynamicObject & object);

  bool predict(const ros::Time & time) override;
  bool predict(const double dt, KalmanFilter & ekf);
  bool measure(
    const autoware_perception_msgs::DynamicObject & object, const ros::Time & time) override;
  bool measureWithPose(const autoware_perception_msgs::DynamicObject & object);
  bool measureWithShape(const autoware_perception_msgs::DynamicObject & object);
  bool getEstimatedDynamicObject(
    const ros::Time & time, autoware_perception_msgs::DynamicObject & object) override;
  virtual ~NormalVehicleTracker(){};
};
