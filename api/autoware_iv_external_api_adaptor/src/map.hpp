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

#ifndef MAP_HPP_
#define MAP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "autoware_external_api_msgs/msg/map_hash.hpp"

namespace external_api
{

class Map : public rclcpp::Node
{
public:
  explicit Map(const rclcpp::NodeOptions & options);

private:
  using MapHash = autoware_external_api_msgs::msg::MapHash;

  // ros interface
  rclcpp::Publisher<MapHash>::SharedPtr pub_map_info_;
  rclcpp::Subscription<MapHash>::SharedPtr sub_map_info_;

  // ros callback
  void getMapHash(const autoware_external_api_msgs::msg::MapHash::SharedPtr message);
};

}  // namespace external_api

#endif  // MAP_HPP_
