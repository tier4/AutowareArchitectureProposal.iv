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

#ifndef AWAPI_AWIV_MESSAGE_CONVERtER_HPP__
#define AWAPI_AWIV_MESSAGE_CONVERtER_HPP__

#include "autoware_auto_system_msgs/msg/hazard_status_stamped.hpp"
#include "autoware_system_msgs/msg/hazard_status_stamped.hpp"

namespace autoware_api
{

auto convert(const autoware_auto_system_msgs::msg::HazardStatusStamped & status)
{
  autoware_system_msgs::msg::HazardStatusStamped iv_status;
  iv_status.header.stamp = status.stamp;
  iv_status.status.level = status.status.level;
  iv_status.status.emergency = status.status.emergency;
  iv_status.status.emergency_holding = status.status.emergency_holding;
  iv_status.status.diagnostics_nf = status.status.diag_no_fault;
  iv_status.status.diagnostics_sf = status.status.diag_safe_fault;
  iv_status.status.diagnostics_lf = status.status.diag_latent_fault;
  iv_status.status.diagnostics_spf = status.status.diag_single_point_fault;
  return iv_status;
}

}  // namespace autoware_api

#endif  // AWAPI_AWIV_MESSAGE_CONVERtER_HPP__
