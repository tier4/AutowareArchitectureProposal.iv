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

#ifndef DIAGNOSTICS_HPP_
#define DIAGNOSTICS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "autoware_api_utils/autoware_api_utils.hpp"
#include "autoware_external_api_msgs/msg/classified_diagnostics.hpp"

namespace external_api
{

class Diagnostics : public rclcpp::Node
{
public:
  explicit Diagnostics(const rclcpp::NodeOptions & options);

private:
  // ros interface
  rclcpp::Publisher<autoware_external_api_msgs::msg::ClassifiedDiagnostics>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ros callback
  void onTimer();
};

}  // namespace external_api

#endif  // DIAGNOSTICS_HPP_
