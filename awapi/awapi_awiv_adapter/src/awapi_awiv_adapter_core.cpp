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

#include <functional>
#include <memory>
#include <utility>

#include "awapi_awiv_adapter/awapi_awiv_adapter_core.hpp"

namespace autoware_api
{
using std::placeholders::_1;

AutowareIvAdapter::AutowareIvAdapter()
: Node("awapi_awiv_adapter_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  // get param
  status_pub_hz_ = this->declare_parameter("status_pub_hz", 5.0);
  stop_reason_timeout_ = this->declare_parameter("stop_reason_timeout", 0.5);
  stop_reason_thresh_dist_ = this->declare_parameter("stop_reason_thresh_dist", 100.0);
  const double default_max_velocity = waitForParam<double>(
    this, declare_parameter("node/max_velocity", ""),
    declare_parameter("param/max_velocity", ""));
  const bool em_stop_param = waitForParam<bool>(
    this, declare_parameter("node/emergency_stop", ""),
    declare_parameter("param/emergency_stop", ""));
  emergencyParamCheck(em_stop_param);

  // setup instance
  vehicle_state_publisher_ = std::make_unique<AutowareIvVehicleStatePublisher>(*this);
  autoware_state_publisher_ = std::make_unique<AutowareIvAutowareStatePublisher>(*this);
  stop_reason_aggregator_ =
    std::make_unique<AutowareIvStopReasonAggregator>(
    *this, stop_reason_timeout_,
    stop_reason_thresh_dist_);
  lane_change_state_publisher_ = std::make_unique<AutowareIvLaneChangeStatePublisher>(*this);
  obstacle_avoidance_state_publisher_ =
    std::make_unique<AutowareIvObstacleAvoidanceStatePublisher>(*this);
  max_velocity_publisher_ =
    std::make_unique<AutowareIvMaxVelocityPublisher>(*this, default_max_velocity);

  // publisher
  pub_door_control_ =
    this->create_publisher<pacmod_msgs::msg::SystemCmdInt>("output/door_control", 1);
  pub_door_status_ =
    this->create_publisher<autoware_api_msgs::msg::DoorStatus>("output/door_status", 1);

  // subscriber
  sub_steer_ = this->create_subscription<autoware_vehicle_msgs::msg::Steering>(
    "input/steer", 1, std::bind(&AutowareIvAdapter::callbackSteer, this, _1));
  sub_vehicle_cmd_ = this->create_subscription<autoware_vehicle_msgs::msg::VehicleCommand>(
    "input/vehicle_cmd", 1, std::bind(&AutowareIvAdapter::callbackVehicleCmd, this, _1));
  sub_turn_signal_cmd_ = this->create_subscription<autoware_vehicle_msgs::msg::TurnSignal>(
    "input/turn_signal", 1, std::bind(&AutowareIvAdapter::callbackTurnSignal, this, _1));
  sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "input/twist", 1, std::bind(&AutowareIvAdapter::callbackTwist, this, _1));
  sub_gear_ = this->create_subscription<autoware_vehicle_msgs::msg::ShiftStamped>(
    "input/gear", 1, std::bind(&AutowareIvAdapter::callbackGear, this, _1));
  sub_battery_ = this->create_subscription<std_msgs::msg::Float32>(
    "input/battery", 1, std::bind(&AutowareIvAdapter::callbackBattery, this, _1));
  sub_nav_sat_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "input/nav_sat", 1, std::bind(&AutowareIvAdapter::callbackNavSat, this, _1));
  sub_autoware_state_ = this->create_subscription<autoware_system_msgs::msg::AutowareState>(
    "input/autoware_state", 1, std::bind(&AutowareIvAdapter::callbackAutowareState, this, _1));
  sub_control_mode_ = this->create_subscription<autoware_vehicle_msgs::msg::ControlMode>(
    "input/control_mode", 1, std::bind(&AutowareIvAdapter::callbackControlMode, this, _1));
  sub_gate_mode_ = this->create_subscription<autoware_control_msgs::msg::GateMode>(
    "input/gate_mode", 1, std::bind(&AutowareIvAdapter::callbackGateMode, this, _1));
  sub_emergency_ = this->create_subscription<std_msgs::msg::Bool>(
    "input/is_emergency", 1, std::bind(&AutowareIvAdapter::callbackIsEmergency, this, _1));
  sub_hazard_status_ =
    this->create_subscription<autoware_system_msgs::msg::HazardStatusStamped>(
    "input/hazard_status", 1, std::bind(&AutowareIvAdapter::callbackHazardStatus, this, _1));
  sub_stop_reason_ = this->create_subscription<autoware_planning_msgs::msg::StopReasonArray>(
    "input/stop_reason", 100, std::bind(&AutowareIvAdapter::callbackStopReason, this, _1));
  sub_diagnostics_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "input/diagnostics", 1, std::bind(&AutowareIvAdapter::callbackDiagnostics, this, _1));
  sub_global_rpt_ = this->create_subscription<pacmod_msgs::msg::GlobalRpt>(
    "input/global_rpt", 1, std::bind(&AutowareIvAdapter::callbackGlobalRpt, this, _1));
  sub_lane_change_available_ = this->create_subscription<std_msgs::msg::Bool>(
    "input/lane_change_avaiable", 1,
    std::bind(&AutowareIvAdapter::callbackLaneChangeAvailable, this, _1));
  sub_lane_change_ready_ = this->create_subscription<std_msgs::msg::Bool>(
    "input/lane_change_ready", 1, std::bind(&AutowareIvAdapter::callbackLaneChangeReady, this, _1));
  sub_lane_change_candidate_ = this->create_subscription<autoware_planning_msgs::msg::Path>(
    "input/lane_change_candidate_path", 1,
    std::bind(&AutowareIvAdapter::callbackLaneChangeCandidatePath, this, _1));
  sub_obstacle_avoid_ready_ = this->create_subscription<std_msgs::msg::Bool>(
    "input/obstacle_avoid_ready", 1,
    std::bind(&AutowareIvAdapter::callbackLaneObstacleAvoidReady, this, _1));
  sub_obstacle_avoid_candidate_ =
    this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/obstacle_avoid_candidate_path", 1,
    std::bind(&AutowareIvAdapter::callbackLaneObstacleAvoidCandidatePath, this, _1));
  sub_max_velocity_ = this->create_subscription<std_msgs::msg::Float32>(
    "input/max_velocity", 1, std::bind(&AutowareIvAdapter::callbackMaxVelocity, this, _1));
  sub_temporary_stop_ = this->create_subscription<std_msgs::msg::Bool>(
    "input/temporary_stop", 1, std::bind(&AutowareIvAdapter::callbackTemporaryStop, this, _1));
  sub_autoware_traj_ =
    this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/autoware_trajectory", 1,
    std::bind(&AutowareIvAdapter::callbackAutowareTrajectory, this, _1));
  sub_door_control_ = this->create_subscription<std_msgs::msg::Bool>(
    "input/door_control", 1, std::bind(&AutowareIvAdapter::callbackDoorControl, this, _1));
  sub_door_status_ = this->create_subscription<pacmod_msgs::msg::SystemRptInt>(
    "input/door_status", 1, std::bind(&AutowareIvAdapter::callbackDoorStatus, this, _1));

  // timer
  auto timer_callback = std::bind(&AutowareIvAdapter::timerCallback, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / status_pub_hz_));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void AutowareIvAdapter::emergencyParamCheck(const bool emergency_stop_param)
{
  if (!emergency_stop_param) {
    RCLCPP_WARN_STREAM(get_logger(), "parameter[use_external_emergency_stop] is false.");
    RCLCPP_WARN_STREAM(get_logger(), "autoware/put/emergency is not valid");
  }
}

void AutowareIvAdapter::timerCallback()
{
  // get current pose
  getCurrentPose();

  // publish vehicle state
  vehicle_state_publisher_->statePublisher(aw_info_);

  // publish autoware state
  autoware_state_publisher_->statePublisher(aw_info_);

  // publish lane change state
  lane_change_state_publisher_->statePublisher(aw_info_);

  // publish obstacle_avoidance state
  obstacle_avoidance_state_publisher_->statePublisher(aw_info_);

  // publish pacmod door status
  pub_door_status_->publish(pacmod_util::getDoorStatusMsg(aw_info_.door_state_ptr));
}

void AutowareIvAdapter::callbackSteer(
  const autoware_vehicle_msgs::msg::Steering::ConstSharedPtr msg_ptr)
{
  aw_info_.steer_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackVehicleCmd(
  const autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr msg_ptr)
{
  aw_info_.vehicle_cmd_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackTurnSignal(
  const autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg_ptr)
{
  aw_info_.turn_signal_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackTwist(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg_ptr)
{
  aw_info_.twist_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackGear(
  const autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg_ptr)
{
  aw_info_.gear_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackBattery(const std_msgs::msg::Float32::ConstSharedPtr msg_ptr)
{
  aw_info_.battery_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackNavSat(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg_ptr)
{
  aw_info_.nav_sat_ptr = msg_ptr;
}

void AutowareIvAdapter::getCurrentPose()
{
  try {
    auto transform = tf_buffer_.lookupTransform("map", "base_link", rclcpp::Time(0));
    geometry_msgs::msg::PoseStamped ps;
    ps.header = transform.header;
    ps.pose.position.x = transform.transform.translation.x;
    ps.pose.position.y = transform.transform.translation.y;
    ps.pose.position.z = transform.transform.translation.z;
    ps.pose.orientation = transform.transform.rotation;
    aw_info_.current_pose_ptr = std::make_shared<geometry_msgs::msg::PoseStamped>(ps);
  } catch (tf2::TransformException & ex) {
    RCLCPP_DEBUG_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), 2000 /* ms */, "cannot get self pose");
  }
}

void AutowareIvAdapter::callbackAutowareState(
  const autoware_system_msgs::msg::AutowareState::ConstSharedPtr msg_ptr)
{
  aw_info_.autoware_state_ptr = msg_ptr;
}
void AutowareIvAdapter::callbackControlMode(
  const autoware_vehicle_msgs::msg::ControlMode::ConstSharedPtr msg_ptr)
{
  aw_info_.control_mode_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackGateMode(
  const autoware_control_msgs::msg::GateMode::ConstSharedPtr msg_ptr)
{
  aw_info_.gate_mode_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackIsEmergency(
  const std_msgs::msg::Bool::ConstSharedPtr msg_ptr)
{
  aw_info_.is_emergency_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackHazardStatus(
  const autoware_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg_ptr)
{
  aw_info_.hazard_status_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackStopReason(
  const autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr msg_ptr)
{
  aw_info_.stop_reason_ptr = stop_reason_aggregator_->updateStopReasonArray(msg_ptr, aw_info_);
}

void AutowareIvAdapter::callbackDiagnostics(
  const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg_ptr)
{
  aw_info_.diagnostic_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackGlobalRpt(const pacmod_msgs::msg::GlobalRpt::ConstSharedPtr msg_ptr)
{
  aw_info_.global_rpt_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackLaneChangeAvailable(
  const std_msgs::msg::Bool::ConstSharedPtr msg_ptr)
{
  aw_info_.lane_change_available_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackLaneChangeReady(const std_msgs::msg::Bool::ConstSharedPtr msg_ptr)
{
  aw_info_.lane_change_ready_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackLaneChangeCandidatePath(
  const autoware_planning_msgs::msg::Path::ConstSharedPtr msg_ptr)
{
  aw_info_.lane_change_candidate_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackLaneObstacleAvoidReady(
  const std_msgs::msg::Bool::ConstSharedPtr msg_ptr)
{
  aw_info_.obstacle_avoid_ready_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackLaneObstacleAvoidCandidatePath(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg_ptr)
{
  aw_info_.obstacle_avoid_candidate_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackMaxVelocity(const std_msgs::msg::Float32::ConstSharedPtr msg_ptr)
{
  aw_info_.max_velocity_ptr = msg_ptr;
  max_velocity_publisher_->statePublisher(aw_info_);
}

void AutowareIvAdapter::callbackTemporaryStop(const std_msgs::msg::Bool::ConstSharedPtr msg_ptr)
{
  if (aw_info_.temporary_stop_ptr) {
    if (aw_info_.temporary_stop_ptr->data == msg_ptr->data) {
      // if same value as last time is sent, ignore msg.
      return;
    }
  }

  aw_info_.temporary_stop_ptr = msg_ptr;
  max_velocity_publisher_->statePublisher(aw_info_);
}

void AutowareIvAdapter::callbackAutowareTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg_ptr)
{
  aw_info_.autoware_planning_traj_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackDoorControl(
  const std_msgs::msg::Bool::ConstSharedPtr msg_ptr)
{
  pub_door_control_->publish(pacmod_util::createClearOverrideDoorCommand(this->get_clock()));
  rclcpp::Rate(10.0).sleep();  // avoid message loss
  pub_door_control_->publish(pacmod_util::createDoorCommand(this->get_clock(), msg_ptr));
}

void AutowareIvAdapter::callbackDoorStatus(
  const pacmod_msgs::msg::SystemRptInt::ConstSharedPtr msg_ptr)
{
  aw_info_.door_state_ptr = msg_ptr;
}

}  // namespace autoware_api
