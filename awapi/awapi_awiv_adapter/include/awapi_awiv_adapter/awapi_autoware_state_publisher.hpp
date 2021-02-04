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

#ifndef AWAPI_AWIV_ADAPTER__AWAPI_AUTOWARE_STATE_PUBLISHER_HPP_
#define AWAPI_AWIV_ADAPTER__AWAPI_AUTOWARE_STATE_PUBLISHER_HPP_

#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "awapi_awiv_adapter/awapi_autoware_util.hpp"
#include "autoware_api_msgs/msg/awapi_autoware_status.hpp"

namespace autoware_api
{
class AutowareIvAutowareStatePublisher
{
public:
  explicit AutowareIvAutowareStatePublisher(rclcpp::Node & node);
  void statePublisher(const AutowareInfo & aw_info);

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  // publisher
  rclcpp::Publisher<autoware_api_msgs::msg::AwapiAutowareStatus>::SharedPtr pub_state_;

  // parameter

  /* parameter for judging goal now */
  bool arrived_goal_;
  autoware_system_msgs::msg::AutowareState::_state_type prev_state_;

  /* parameter for judging diag leaf */
  std::set<std::string> diag_name_set_;

  void getAutowareStateInfo(
    const autoware_system_msgs::msg::AutowareState::ConstSharedPtr & autoware_state_ptr,
    autoware_api_msgs::msg::AwapiAutowareStatus * status);
  void getControlModeInfo(
    const autoware_vehicle_msgs::msg::ControlMode::ConstSharedPtr & control_mode_ptr,
    autoware_api_msgs::msg::AwapiAutowareStatus * status);
  void getGateModeInfo(
    const autoware_control_msgs::msg::GateMode::ConstSharedPtr & gate_mode_ptr,
    autoware_api_msgs::msg::AwapiAutowareStatus * status);
  void getIsEmergencyInfo(
    const std_msgs::msg::Bool::ConstSharedPtr & is_emergency_ptr,
    autoware_api_msgs::msg::AwapiAutowareStatus * status);
  void getHazardStatusInfo(
    const autoware_system_msgs::msg::HazardStatusStamped::ConstSharedPtr & hazard_status_ptr,
    autoware_api_msgs::msg::AwapiAutowareStatus * status);
  void getStopReasonInfo(
    const autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr & stop_reason_ptr,
    autoware_api_msgs::msg::AwapiAutowareStatus * status);
  void getDiagInfo(
    const AutowareInfo & aw_info, autoware_api_msgs::msg::AwapiAutowareStatus * status);
  void getErrorDiagInfo(
    const AutowareInfo & aw_info, autoware_api_msgs::msg::AwapiAutowareStatus * status);
  void getGlobalRptInfo(
    const pacmod_msgs::msg::GlobalRpt::ConstSharedPtr & global_rpt_ptr,
    autoware_api_msgs::msg::AwapiAutowareStatus * status);

  bool isGoal(const autoware_system_msgs::msg::AutowareState::ConstSharedPtr & autoware_state);
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> extractLeafDiag(
    const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & diag_vec);
  std::string splitStringByLastSlash(const std::string & str);
  void updateDiagNameSet(const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & diag_vec);
  bool isLeaf(const diagnostic_msgs::msg::DiagnosticStatus & diag);
};

}  // namespace autoware_api

#endif  // AWAPI_AWIV_ADAPTER__AWAPI_AUTOWARE_STATE_PUBLISHER_HPP_
