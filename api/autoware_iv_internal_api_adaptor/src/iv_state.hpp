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

#ifndef IV_STATE_HPP_
#define IV_STATE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>

namespace internal_api
{
class IVState : public rclcpp::Node
{
public:
  explicit IVState(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<autoware_auto_system_msgs::msg::AutowareState>::SharedPtr sub_state_;
  rclcpp::Publisher<autoware_system_msgs::msg::AutowareState>::SharedPtr pub_state_;
  void onState(const autoware_auto_system_msgs::msg::AutowareState::ConstSharedPtr message);
};

}  // namespace internal_api

#endif  // IV_STATE_HPP_
