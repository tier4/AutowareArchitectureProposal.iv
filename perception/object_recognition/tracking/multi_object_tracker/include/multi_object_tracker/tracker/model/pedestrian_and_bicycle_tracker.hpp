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
#include "kalman_filter/kalman_filter.hpp"
#include "autoware_perception_msgs/msg/dynamic_object.hpp"
#include "multi_object_tracker/tracker/model/bicycle_tracker.hpp"
#include "multi_object_tracker/tracker/model/pedestrian_tracker.hpp"
#include "multi_object_tracker/tracker/model/tracker_base.hpp"

class PedestrianAndBicycleTracker : public Tracker
{
private:
  PedestrianTracker pedestrian_tracker_;
  BicycleTracker bicycle_tracker_;

public:
  PedestrianAndBicycleTracker(
    const rclcpp::Time & time, const autoware_perception_msgs::msg::DynamicObject & object);

  bool predict(const rclcpp::Time & time) override;
  bool measure(
    const autoware_perception_msgs::msg::DynamicObject & object, const rclcpp::Time & time) override;
  bool getEstimatedDynamicObject(
    const rclcpp::Time & time, autoware_perception_msgs::msg::DynamicObject & object) override;
  virtual ~PedestrianAndBicycleTracker(){};
};
