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

#pragma once

#include <string>

#include <ros/ros.h>

#include <autoware_utils/ros/wait_for_param.h>

struct VehicleInfo
{
  // Basic
  double wheel_radius;
  double wheel_width;
  double wheel_base;
  double wheel_tread;
  double front_overhang;
  double rear_overhang;
  double left_overhang;
  double right_overhang;
  double vehicle_height;

  // Additional
  double vehicle_length;
  double vehicle_width;
  double min_longitudinal_offset;
  double max_longitudinal_offset;
  double min_lateral_offset;
  double max_lateral_offset;
  double min_height_offset;
  double max_height_offset;
};

inline VehicleInfo waitForVehicleInfo()
{
  const std::string ns = "/vehicle_info";

  ros::NodeHandle nh("");

  VehicleInfo i;

  // Basic
  i.wheel_radius = waitForParam<double>(nh, ns + "/wheel_radius");
  i.wheel_width = waitForParam<double>(nh, ns + "/wheel_width");
  i.wheel_base = waitForParam<double>(nh, ns + "/wheel_base");
  i.wheel_tread = waitForParam<double>(nh, ns + "/wheel_tread");
  i.front_overhang = waitForParam<double>(nh, ns + "/front_overhang");
  i.rear_overhang = waitForParam<double>(nh, ns + "/rear_overhang");
  i.left_overhang = waitForParam<double>(nh, ns + "/left_overhang");
  i.right_overhang = waitForParam<double>(nh, ns + "/right_overhang");
  i.vehicle_height = waitForParam<double>(nh, ns + "/vehicle_height");

  // Additional
  i.vehicle_length = waitForParam<double>(nh, ns + "/vehicle_length");
  i.vehicle_width = waitForParam<double>(nh, ns + "/vehicle_width");
  i.min_longitudinal_offset = waitForParam<double>(nh, ns + "/min_longitudinal_offset");
  i.max_longitudinal_offset = waitForParam<double>(nh, ns + "/max_longitudinal_offset");
  i.min_lateral_offset = waitForParam<double>(nh, ns + "/min_lateral_offset");
  i.max_lateral_offset = waitForParam<double>(nh, ns + "/max_lateral_offset");
  i.min_height_offset = waitForParam<double>(nh, ns + "/min_height_offset");
  i.max_height_offset = waitForParam<double>(nh, ns + "/max_height_offset");

  return i;
}
