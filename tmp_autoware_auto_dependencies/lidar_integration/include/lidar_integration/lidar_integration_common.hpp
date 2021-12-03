// Copyright 2020 Silexica GmbH, Lichtstr. 25, Cologne, Germany. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.


#ifndef LIDAR_INTEGRATION__LIDAR_INTEGRATION_COMMON_HPP_
#define LIDAR_INTEGRATION__LIDAR_INTEGRATION_COMMON_HPP_

#include <rclcpp/rclcpp.hpp>

#define LIDAR_INTEGRATION_LOGGER rclcpp::get_logger("lidar_integration")

#define LIDAR_INTEGRATION_INFO(...) \
  RCLCPP_INFO(LIDAR_INTEGRATION_LOGGER, __VA_ARGS__)

#define LIDAR_INTEGRATION_ERROR(...) \
  RCLCPP_ERROR(LIDAR_INTEGRATION_LOGGER, __VA_ARGS__)

#define LIDAR_INTEGRATION_FATAL(...) { \
    RCLCPP_ERROR(LIDAR_INTEGRATION_LOGGER, __VA_ARGS__); \
    exit(-1); \
}

#endif  // LIDAR_INTEGRATION__LIDAR_INTEGRATION_COMMON_HPP_
