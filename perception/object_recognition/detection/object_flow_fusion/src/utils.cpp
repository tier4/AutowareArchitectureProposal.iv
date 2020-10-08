/*
 * Copyright 2020 TierIV. All rights reserved.
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

#include "object_flow_fusion/utils.h"

namespace object_flow_fusion
{
geometry_msgs::Vector3 Utils::mptopic2kph(
  const geometry_msgs::Vector3& twist,
  double topic_rate)
{
  // convert twist to [km/h] from [m/topic_rate]
  geometry_msgs::Vector3 converted_twist;
  converted_twist.x = (3600.0 / 1000) * twist.x / topic_rate;
  converted_twist.y = (3600.0 / 1000) * twist.y / topic_rate;
  converted_twist.z = (3600.0 / 1000) * twist.z / topic_rate;
  return converted_twist;
}

geometry_msgs::Vector3 Utils::kph2mptopic(
  const geometry_msgs::Vector3& twist,
  double topic_rate)
{
  // convert twist to [km/h] from [m/topic_rate]
  geometry_msgs::Vector3 converted_twist;
  converted_twist.x = (1000.0 / 3600) * twist.x * topic_rate;
  converted_twist.y = (1000.0 / 3600) * twist.y * topic_rate;
  converted_twist.z = (1000.0 / 3600) * twist.z * topic_rate;
  return converted_twist;
}

geometry_msgs::Vector3 Utils::kph2mps(const geometry_msgs::Vector3& twist)
{
  // convert twist to [km/h] from [m/topic_rate]
  geometry_msgs::Vector3 converted_twist;
  converted_twist.x = (1000.0 / 3600) * twist.x;
  converted_twist.y = (1000.0 / 3600) * twist.y;
  converted_twist.z = (1000.0 / 3600) * twist.z;
  return converted_twist;
}

geometry_msgs::Twist Utils::kph2mps(const geometry_msgs::Twist& twist)
{
  // convert twist to [km/h] from [m/topic_rate]
  geometry_msgs::Twist converted_twist;
  converted_twist.linear.x = (1000.0 / 3600) * twist.linear.x;
  converted_twist.linear.y = (1000.0 / 3600) * twist.linear.y;
  converted_twist.linear.z = (1000.0 / 3600) * twist.linear.z;
  return converted_twist;
}
} // object_flow_fusion
