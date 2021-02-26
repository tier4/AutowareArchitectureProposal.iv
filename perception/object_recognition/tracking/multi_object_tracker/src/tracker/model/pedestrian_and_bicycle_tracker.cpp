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

#include "multi_object_tracker/tracker/model/pedestrian_and_bicycle_tracker.hpp"
#include "autoware_utils/autoware_utils.hpp"

PedestrianAndBicycleTracker::PedestrianAndBicycleTracker(
  const rclcpp::Time & time, const autoware_perception_msgs::msg::DynamicObject & object)
: pedestrian_tracker_(time, object),
  bicycle_tracker_(time, object),
  Tracker(time, object.semantic.type)
{
}

bool PedestrianAndBicycleTracker::predict(const rclcpp::Time & time)
{
  pedestrian_tracker_.predict(time);
  bicycle_tracker_.predict(time);
  return true;
}

bool PedestrianAndBicycleTracker::measure(
  const autoware_perception_msgs::msg::DynamicObject & object, const rclcpp::Time & time)
{
  pedestrian_tracker_.measure(object, time);
  bicycle_tracker_.measure(object, time);
  setType(object.semantic.type);
  return true;
}

bool PedestrianAndBicycleTracker::getEstimatedDynamicObject(
  const rclcpp::Time & time, autoware_perception_msgs::msg::DynamicObject & object)
{
  if (getType() == autoware_perception_msgs::msg::Semantic::PEDESTRIAN) {
    pedestrian_tracker_.getEstimatedDynamicObject(time, object);
  } else if (
    getType() == autoware_perception_msgs::msg::Semantic::BICYCLE ||
    getType() == autoware_perception_msgs::msg::Semantic::MOTORBIKE) {
    bicycle_tracker_.getEstimatedDynamicObject(time, object);
  }
  object.id = getUUID();
  object.semantic.type = getType();
  return true;
}
