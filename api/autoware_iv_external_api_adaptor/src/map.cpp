// Copyright 2021 Tier IV, Inc.
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

#include "map.hpp"

namespace external_api
{

Map::Map(const rclcpp::NodeOptions & options)
: Node("external_api_map", options)
{
  using namespace std::placeholders;

  pub_map_info_ = create_publisher<autoware_external_api_msgs::msg::MapHash>(
    "/api/external/get/map/info/hash", rclcpp::QoS(1).transient_local());
  sub_map_info_ = create_subscription<autoware_external_api_msgs::msg::MapHash>(
    "/api/autoware/get/map/info/hash", rclcpp::QoS(1).transient_local(),
    std::bind(&Map::getMapHash, this, _1));
}

void Map::getMapHash(const autoware_external_api_msgs::msg::MapHash::SharedPtr message)
{
  pub_map_info_->publish(*message);
}

}  // namespace external_api

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::Map)
