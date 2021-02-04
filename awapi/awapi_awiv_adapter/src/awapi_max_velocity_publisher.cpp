// Copyright 2020 Tier IV, Inc.
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

#include "awapi_awiv_adapter/awapi_max_velocity_publisher.hpp"

namespace autoware_api
{
AutowareIvMaxVelocityPublisher::AutowareIvMaxVelocityPublisher(
  rclcpp::Node & node, const double default_max_velocity)
: default_max_velocity_(default_max_velocity)
{
  // publisher
  pub_state_ = node.create_publisher<std_msgs::msg::Float32>("output/max_velocity", 1);
}

void AutowareIvMaxVelocityPublisher::statePublisher(const AutowareInfo & aw_info)
{
  std_msgs::msg::Float32 max_velocity;
  if (calcMaxVelocity(
      aw_info.max_velocity_ptr, aw_info.temporary_stop_ptr, &max_velocity.data))  // publish info
  {
    pub_state_->publish(max_velocity);
  }
}

bool AutowareIvMaxVelocityPublisher::calcMaxVelocity(
  const std_msgs::msg::Float32::ConstSharedPtr & max_velocity_ptr,
  const std_msgs::msg::Bool::ConstSharedPtr & temporary_stop_ptr, float * max_velocity)
{
  if (!max_velocity_ptr && !temporary_stop_ptr) {
    // receive no max velocity information
    return false;
  }

  // input max velocity
  *max_velocity =
    static_cast<float>(max_velocity_ptr ? max_velocity_ptr->data : default_max_velocity_);

  if (temporary_stop_ptr && temporary_stop_ptr->data) {
    // if temporary_stop is true, max velocity is 0
    *max_velocity = static_cast<float>(0.0);
  }

  return true;
}

}  // namespace autoware_api
