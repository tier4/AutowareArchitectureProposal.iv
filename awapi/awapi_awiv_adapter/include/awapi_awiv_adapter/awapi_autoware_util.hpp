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

#ifndef AWAPI_AWIV_ADAPTER__AWAPI_AUTOWARE_UTIL_HPP_
#define AWAPI_AWIV_ADAPTER__AWAPI_AUTOWARE_UTIL_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_api_msgs/msg/stop_command.hpp>
#include <autoware_api_msgs/msg/velocity_limit.hpp>
#include <autoware_control_msgs/msg/gate_mode.hpp>
#include <autoware_planning_msgs/msg/is_avoidance_possible.hpp>
#include <autoware_planning_msgs/msg/lane_change_status.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/stop_reason_array.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <autoware_system_msgs/msg/emergency_state_stamped.hpp>
#include <autoware_system_msgs/msg/hazard_status_stamped.hpp>
#include <autoware_v2x_msgs/msg/infrastructure_command_array.hpp>
#include <autoware_v2x_msgs/msg/virtual_traffic_light_state_array.hpp>
#include <autoware_vehicle_msgs/msg/battery_status.hpp>
#include <autoware_vehicle_msgs/msg/control_mode.hpp>
#include <autoware_vehicle_msgs/msg/shift_stamped.hpp>
#include <autoware_vehicle_msgs/msg/steering.hpp>
#include <autoware_vehicle_msgs/msg/turn_signal.hpp>
#include <autoware_vehicle_msgs/msg/vehicle_command.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pacmod_msgs/msg/global_rpt.hpp>
#include <pacmod_msgs/msg/system_rpt_int.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace autoware_api
{
struct AutowareInfo
{
  std::shared_ptr<geometry_msgs::msg::PoseStamped> current_pose_ptr;
  autoware_vehicle_msgs::msg::Steering::ConstSharedPtr steer_ptr;
  autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr vehicle_cmd_ptr;
  autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr turn_signal_ptr;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_ptr;
  autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr gear_ptr;
  autoware_vehicle_msgs::msg::BatteryStatus::ConstSharedPtr battery_ptr;
  sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat_ptr;
  autoware_system_msgs::msg::AutowareState::ConstSharedPtr autoware_state_ptr;
  autoware_vehicle_msgs::msg::ControlMode::ConstSharedPtr control_mode_ptr;
  autoware_control_msgs::msg::GateMode::ConstSharedPtr gate_mode_ptr;
  autoware_system_msgs::msg::EmergencyStateStamped::ConstSharedPtr emergency_state_ptr;
  autoware_system_msgs::msg::HazardStatusStamped::ConstSharedPtr hazard_status_ptr;
  autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr stop_reason_ptr;
  autoware_v2x_msgs::msg::InfrastructureCommandArray::ConstSharedPtr v2x_command_ptr;
  autoware_v2x_msgs::msg::VirtualTrafficLightStateArray::ConstSharedPtr v2x_state_ptr;
  diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diagnostic_ptr;
  pacmod_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt_ptr;
  autoware_planning_msgs::msg::LaneChangeStatus::ConstSharedPtr lane_change_available_ptr;
  autoware_planning_msgs::msg::LaneChangeStatus::ConstSharedPtr lane_change_ready_ptr;
  autoware_planning_msgs::msg::Path::ConstSharedPtr lane_change_candidate_ptr;
  autoware_planning_msgs::msg::IsAvoidancePossible::ConstSharedPtr obstacle_avoid_ready_ptr;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr obstacle_avoid_candidate_ptr;
  autoware_api_msgs::msg::VelocityLimit::ConstSharedPtr max_velocity_ptr;
  autoware_planning_msgs::msg::VelocityLimit::ConstSharedPtr current_max_velocity_ptr;
  autoware_api_msgs::msg::StopCommand::ConstSharedPtr temporary_stop_ptr;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr autoware_planning_traj_ptr;
  pacmod_msgs::msg::SystemRptInt::ConstSharedPtr door_state_ptr;
};

template <class T>
T waitForParam(
  rclcpp::Node * node, const std::string & remote_node_name, const std::string & param_name)
{
  std::chrono::seconds sec(1);

  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, remote_node_name);

  while (!param_client->wait_for_service(sec)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service.");
      return {};
    }
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 1000 /* ms */, "waiting for node: %s, param: %s\n",
      remote_node_name.c_str(), param_name.c_str());
  }

  if (param_client->has_parameter(param_name)) {
    return param_client->get_parameter<T>(param_name);
  }

  return {};
}

double lowpass_filter(const double current_value, const double prev_value, const double gain);

namespace planning_util
{
bool calcClosestIndex(
  const autoware_planning_msgs::msg::Trajectory & traj, const geometry_msgs::msg::Pose & pose,
  size_t & output_closest_idx, const double dist_thr = 10.0, const double angle_thr = M_PI / 2.0);

inline geometry_msgs::msg::Pose getPose(
  const autoware_planning_msgs::msg::Trajectory & traj, const int idx)
{
  return traj.points.at(idx).pose;
}

inline double calcDist2d(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot((a.x - b.x), (a.y - b.y));
}

double normalizeEulerAngle(double euler);

double calcArcLengthFromWayPoint(
  const autoware_planning_msgs::msg::Trajectory & input_path, const size_t src_idx,
  const size_t dst_idx);

double calcDistanceAlongTrajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Pose & target_pose);

}  // namespace planning_util

}  // namespace autoware_api

#endif  // AWAPI_AWIV_ADAPTER__AWAPI_AUTOWARE_UTIL_HPP_
