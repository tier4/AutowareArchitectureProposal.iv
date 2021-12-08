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

#include "iv_msgs.hpp"

#include <autoware_iv_auto_msgs_converter/autoware_iv_auto_msgs_converter.hpp>

namespace internal_api
{
IVMsgs::IVMsgs(const rclcpp::NodeOptions & options) : Node("external_api_iv_msgs", options)
{
  using std::placeholders::_1;

  pub_state_ = create_publisher<AutowareStateIV>("/api/iv_msgs/autoware/state", rclcpp::QoS(1));
  sub_state_ = create_subscription<AutowareStateAuto>(
    "/autoware/state", rclcpp::QoS(1), std::bind(&IVMsgs::onState, this, _1));
  sub_emergency_ = create_subscription<EmergencyStateAuto>(
    "/system/emergency/emergency_state", rclcpp::QoS(1), std::bind(&IVMsgs::onEmergency, this, _1));

  pub_control_mode_ =
    create_publisher<ControlModeIV>("/api/iv_msgs/vehicle/status/control_mode", rclcpp::QoS(1));
  sub_control_mode_ = create_subscription<ControlModeAuto>(
    "/vehicle/status/control_mode", rclcpp::QoS(1), std::bind(&IVMsgs::onControlMode, this, _1));

  pub_trajectory_ = create_publisher<TrajectoryIV>(
    "/api/iv_msgs/planning/scenario_planning/trajectory", rclcpp::QoS(1));
  sub_trajectory_ = create_subscription<TrajectoryAuto>(
    "/planning/scenario_planning/trajectory", rclcpp::QoS(1),
    std::bind(&IVMsgs::onTrajectory, this, _1));

  pub_dynamic_objects_ = create_publisher<DynamicObjectsIV>(
    "/api/iv_msgs/perception/object_recognition/tracking/objects", rclcpp::QoS(1));
  sub_tracked_objects_ = create_subscription<TrackedObjectsAuto>(
    "/perception/object_recognition/tracking/objects", rclcpp::QoS(1),
    std::bind(&IVMsgs::onTrackedObjects, this, _1));
}

void IVMsgs::onState(const AutowareStateAuto::ConstSharedPtr message)
{
  auto state = autoware_iv_auto_msgs_converter::convert(*message);
  if (emergency_) {
    switch (emergency_->state) {
      case EmergencyStateAuto::MRM_OPERATING:
      case EmergencyStateAuto::MRM_SUCCEEDED:
      case EmergencyStateAuto::MRM_FAILED:
        state.state = AutowareStateIV::EMERGENCY;
        break;
    }
  }
  pub_state_->publish(state);
}

void IVMsgs::onEmergency(const EmergencyStateAuto::ConstSharedPtr message) { emergency_ = message; }

void IVMsgs::onControlMode(const ControlModeAuto::ConstSharedPtr message)
{
  pub_control_mode_->publish(autoware_iv_auto_msgs_converter::convert(*message));
}

void IVMsgs::onTrajectory(const TrajectoryAuto::ConstSharedPtr message)
{
  pub_trajectory_->publish(autoware_iv_auto_msgs_converter::convert(*message));
}

void IVMsgs::onTrackedObjects(const TrackedObjectsAuto::ConstSharedPtr message)
{
  pub_dynamic_objects_->publish(autoware_iv_auto_msgs_converter::convert(*message));
}

}  // namespace internal_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(internal_api::IVMsgs)
