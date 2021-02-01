/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include <regex>

#include <awapi_awiv_adapter/awapi_autoware_state_publisher.h>
#include <awapi_awiv_adapter/diagnostics_filter.h>

namespace autoware_api
{
AutowareIvAutowareStatePublisher::AutowareIvAutowareStatePublisher()
: nh_(), pnh_("~"), arrived_goal_(false)
{
  // publisher
  pub_state_ = pnh_.advertise<autoware_api_msgs::AwapiAutowareStatus>("output/autoware_status", 1);
}

void AutowareIvAutowareStatePublisher::statePublisher(const AutowareInfo & aw_info)
{
  autoware_api_msgs::AwapiAutowareStatus status;

  //input header
  status.header.frame_id = "base_link";
  status.header.stamp = ros::Time::now();

  // get all info
  getAutowareStateInfo(aw_info.autoware_state_ptr, &status);
  getControlModeInfo(aw_info.control_mode_ptr, &status);
  getGateModeInfo(aw_info.gate_mode_ptr, &status);
  getIsEmergencyInfo(aw_info.is_emergency_ptr, &status);
  getCurrentMaxVelInfo(aw_info.current_max_velocity_ptr, &status);
  getHazardStatusInfo(aw_info, &status);
  getStopReasonInfo(aw_info.stop_reason_ptr, &status);
  getDiagInfo(aw_info, &status);
  getErrorDiagInfo(aw_info, &status);
  getGlobalRptInfo(aw_info.global_rpt_ptr, &status);

  // publish info
  pub_state_.publish(status);
}

void AutowareIvAutowareStatePublisher::getAutowareStateInfo(
  const autoware_system_msgs::AutowareState::ConstPtr & autoware_state_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!autoware_state_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] autoware_state is nullptr");
    return;
  }

  // get autoware_state
  status->autoware_state = autoware_state_ptr->state;
  status->arrived_goal = isGoal(autoware_state_ptr);
}

void AutowareIvAutowareStatePublisher::getControlModeInfo(
  const autoware_vehicle_msgs::ControlMode::ConstPtr & control_mode_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!control_mode_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] control mode is nullptr");
    return;
  }

  // get control mode
  status->control_mode = control_mode_ptr->data;
}

void AutowareIvAutowareStatePublisher::getGateModeInfo(
  const autoware_control_msgs::GateMode::ConstPtr & gate_mode_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!gate_mode_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] gate mode is nullptr");
    return;
  }

  // get control mode
  status->gate_mode = gate_mode_ptr->data;
}

void AutowareIvAutowareStatePublisher::getIsEmergencyInfo(
  const std_msgs::Bool::ConstPtr & is_emergency_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!is_emergency_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] is_emergency is nullptr");
    return;
  }

  // get emergency
  status->emergency_stopped = is_emergency_ptr->data;
}

void AutowareIvAutowareStatePublisher::getCurrentMaxVelInfo(
  const std_msgs::Float32::ConstPtr current_max_velocity_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!current_max_velocity_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(
      5.0, "[AutowareIvAutowareStatePublisher] currrent_max_velocity is nullptr");
    return;
  }

  //get current max velocity
  status->current_max_velocity = current_max_velocity_ptr->data;
}

void AutowareIvAutowareStatePublisher::getHazardStatusInfo(
  const AutowareInfo & aw_info, autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!aw_info.autoware_state_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] autoware_state is nullptr");
    return;
  }

  if (!aw_info.control_mode_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] control_mode is nullptr");
    return;
  }

  if (!aw_info.hazard_status_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] hazard_status is nullptr");
    return;
  }

  // get emergency
  status->hazard_status = *aw_info.hazard_status_ptr;

  // filter leaf diagnostics
  status->hazard_status.status.diagnostics_spf =
    diagnostics_filter::extractLeafDiagnostics(status->hazard_status.status.diagnostics_spf);
  status->hazard_status.status.diagnostics_lf =
    diagnostics_filter::extractLeafDiagnostics(status->hazard_status.status.diagnostics_lf);
  status->hazard_status.status.diagnostics_sf =
    diagnostics_filter::extractLeafDiagnostics(status->hazard_status.status.diagnostics_sf);
  status->hazard_status.status.diagnostics_nf =
    diagnostics_filter::extractLeafDiagnostics(status->hazard_status.status.diagnostics_nf);

  // filter by state
  if (aw_info.autoware_state_ptr->state != autoware_system_msgs::AutowareState::Emergency) {
    status->hazard_status.status.diagnostics_spf = {};
    status->hazard_status.status.diagnostics_lf = {};
    status->hazard_status.status.diagnostics_sf = {};
  }

  // filter by control_mode
  if (aw_info.control_mode_ptr->data == autoware_vehicle_msgs::ControlMode::MANUAL) {
    status->hazard_status.status.diagnostics_spf = {};
    status->hazard_status.status.diagnostics_lf = {};
    status->hazard_status.status.diagnostics_sf = {};
  }
}

void AutowareIvAutowareStatePublisher::getStopReasonInfo(
  const autoware_planning_msgs::StopReasonArray::ConstPtr & stop_reason_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!stop_reason_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] stop reason is nullptr");
    return;
  }

  status->stop_reason = *stop_reason_ptr;
}

void AutowareIvAutowareStatePublisher::getDiagInfo(
  const AutowareInfo & aw_info, autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!aw_info.diagnostic_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] diagnostics is nullptr");
    return;
  }

  // get diag
  status->diagnostics = diagnostics_filter::extractLeafDiagnostics(aw_info.diagnostic_ptr->status);
}

// This function is tentative and should be replaced with getHazardStatusInfo.
// TODO(Kenji Miyake): Make getErrorDiagInfo users to use getHazardStatusInfo.
void AutowareIvAutowareStatePublisher::getErrorDiagInfo(
  const AutowareInfo & aw_info, autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!aw_info.autoware_state_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] autoware_state is nullptr");
    return;
  }

  if (!aw_info.control_mode_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] control_mode is nullptr");
    return;
  }

  if (!aw_info.diagnostic_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] diagnostics is nullptr");
    return;
  }

  if (!aw_info.hazard_status_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] hazard_status is nullptr");
    return;
  }

  // filter by state
  if (aw_info.autoware_state_ptr->state != autoware_system_msgs::AutowareState::Emergency) {
    status->error_diagnostics = {};
    return;
  }

  // filter by control_mode
  if (aw_info.control_mode_ptr->data == autoware_vehicle_msgs::ControlMode::MANUAL) {
    status->error_diagnostics = {};
    return;
  }

  // get diag
  using diagnostic_msgs::DiagnosticStatus;
  const auto & hazard_status = aw_info.hazard_status_ptr->status;
  std::vector<diagnostic_msgs::DiagnosticStatus> error_diagnostics;

  for (const auto & hazard_diag : hazard_status.diagnostics_spf) {
    auto diag = hazard_diag;
    diag.message = "[Single Point Fault]" + hazard_diag.message;
    error_diagnostics.push_back(diag);
  }
  for (const auto & hazard_diag : hazard_status.diagnostics_lf) {
    auto diag = hazard_diag;
    diag.message = "[Latent Fault]" + hazard_diag.message;
    error_diagnostics.push_back(diag);
  }
  for (const auto & hazard_diag : hazard_status.diagnostics_sf) {
    auto diag = hazard_diag;
    diag.message = "[Safe Fault]" + hazard_diag.message;
    error_diagnostics.push_back(diag);
  }
  for (const auto & hazard_diag : hazard_status.diagnostics_nf) {
    auto diag = hazard_diag;
    diag.message = "[No Fault]" + hazard_diag.message;
    diag.level = DiagnosticStatus::OK;
    error_diagnostics.push_back(diag);
  }

  // filter leaf diag
  status->error_diagnostics = diagnostics_filter::extractLeafDiagnostics(error_diagnostics);
}

void AutowareIvAutowareStatePublisher::getGlobalRptInfo(
  const pacmod_msgs::GlobalRpt::ConstPtr & global_rpt_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!global_rpt_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] global_rpt is nullptr");
    return;
  }

  // get global_rpt
  status->autonomous_overriden = global_rpt_ptr->override_active;
}

bool AutowareIvAutowareStatePublisher::isGoal(
  const autoware_system_msgs::AutowareState::ConstPtr & autoware_state)
{
  //rename
  const auto & aw_state = autoware_state->state;

  if (aw_state == autoware_system_msgs::AutowareState::ArrivedGoal) {
    arrived_goal_ = true;
  } else if (
    prev_state_ == autoware_system_msgs::AutowareState::Driving &&
    aw_state == autoware_system_msgs::AutowareState::WaitingForRoute) {
    arrived_goal_ = true;
  }

  if (
    aw_state == autoware_system_msgs::AutowareState::WaitingForEngage ||
    aw_state == autoware_system_msgs::AutowareState::Driving) {
    //cancel goal state
    arrived_goal_ = false;
  }

  prev_state_ = aw_state;

  return arrived_goal_;
}

}  // namespace autoware_api
