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
#include "big_vehicle_tracker.hpp"
#include "normal_vehicle_tracker.hpp"
#include "tracker_base.hpp"

class MultipleVehicleTracker : public Tracker
{
private:
  BigVehicleTracker big_vehicle_tracker_;
  NormalVehicleTracker normal_vehicle_tracker_;

public:
  MultipleVehicleTracker(
    const ros::Time & time, const autoware_perception_msgs::DynamicObject & object);

  bool predict(const ros::Time & time) override;
  bool measure(
    const autoware_perception_msgs::DynamicObject & object, const ros::Time & time) override;
  bool getEstimatedDynamicObject(
    const ros::Time & time, autoware_perception_msgs::DynamicObject & object) override;
  virtual ~MultipleVehicleTracker(){};
};
