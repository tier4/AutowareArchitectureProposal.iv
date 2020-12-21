// Copyright 2019 Autoware Foundation
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

#ifndef MISSION_PLANNER_LANELET2_IMPL_UTILITY_FUNCTIONS_H
#define MISSION_PLANNER_LANELET2_IMPL_UTILITY_FUNCTIONS_H
#include <string>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_core/primitives/LaneletSequence.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
bool exists(const std::unordered_set<lanelet::Id> & set, const lanelet::Id & id);

template <typename T>
bool exists(const std::vector<T> & vectors, const T & item)
{
  for (const auto & i : vectors) {
    if (i == item) {
      return true;
    }
  }
  return false;
}

void setColor(std_msgs::msg::ColorRGBA * cl, double r, double g, double b, double a);
void insertMarkerArray(
  visualization_msgs::msg::MarkerArray * a1, const visualization_msgs::msg::MarkerArray & a2);
std::string toString(const geometry_msgs::msg::Pose & pose);
bool getClosestLanelet(
  const geometry_msgs::msg::Pose & search_pose, const lanelet::LaneletMapPtr & lanelet_map,
  lanelet::Lanelet * closest_lanelet, const rclcpp::Logger & logger, double distance_thresh = 10.0);
#endif  // MISSION_PLANNER_LANELET2_IMPL_UTILITY_FUNCTIONS_H
