/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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
 * v1.0 Yukihiro Saito
 */

#pragma once
#include <cmath>
#include "autoware_perception_msgs/msg/shape.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace utils
{
double getPolygonArea(const geometry_msgs::msg::Polygon & footprint);
double getRectangleArea(const geometry_msgs::msg::Vector3 & dimensions);
double getCircleArea(const geometry_msgs::msg::Vector3 & dimensions);
double getArea(const autoware_perception_msgs::msg::Shape & shape);
}  // namespace utils
