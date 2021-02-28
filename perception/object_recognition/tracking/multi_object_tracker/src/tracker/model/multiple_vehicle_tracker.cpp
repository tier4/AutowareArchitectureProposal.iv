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

#include "multi_object_tracker/tracker/model/multiple_vehicle_tracker.hpp"
#include "autoware_utils/autoware_utils.hpp"

MultipleVehicleTracker::MultipleVehicleTracker(
  const rclcpp::Time & time, const autoware_perception_msgs::msg::DynamicObject & object)
: big_vehicle_tracker_(time, object),
  normal_vehicle_tracker_(time, object),
  Tracker(time, object.semantic.type)
{
}

bool MultipleVehicleTracker::predict(const rclcpp::Time & time)
{
  big_vehicle_tracker_.predict(time);
  normal_vehicle_tracker_.predict(time);
  return true;
}

bool MultipleVehicleTracker::measure(
  const autoware_perception_msgs::msg::DynamicObject & object, const rclcpp::Time & time)
{
  big_vehicle_tracker_.measure(object, time);
  normal_vehicle_tracker_.measure(object, time);
  setType(object.semantic.type);
  return true;
}

bool MultipleVehicleTracker::getEstimatedDynamicObject(
  const rclcpp::Time & time, autoware_perception_msgs::msg::DynamicObject & object)
{
  if (getType() == autoware_perception_msgs::msg::Semantic::CAR) {
    normal_vehicle_tracker_.getEstimatedDynamicObject(time, object);
  } else if (
    getType() == autoware_perception_msgs::msg::Semantic::BUS ||
    getType() == autoware_perception_msgs::msg::Semantic::TRUCK) {
    big_vehicle_tracker_.getEstimatedDynamicObject(time, object);
  }
  object.id = getUUID();
  object.semantic.type = getType();
  return true;
}
