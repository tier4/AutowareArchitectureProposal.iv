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

/**
 * @file velodyne_monitor_node.cpp
 * @brief Velodyne monitor node class
 */

#include <ros/ros.h>

#include <velodyne_monitor/velodyne_monitor.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "velodyne_monitor");

  VelodyneMonitor monitor;

  ros::spin();

  return 0;
}
