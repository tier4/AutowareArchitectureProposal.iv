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

#include "rclcpp/rclcpp.hpp"

#include "awapi_awiv_adapter/awapi_autoware_util.hpp"
#include "autoware_api_msgs/msg/awapi_vehicle_status.hpp"

namespace autoware_api
{
class AutowareIvVehicleStatePublisher
{
public:
  AutowareIvVehicleStatePublisher(rclcpp::Node & node);
  void statePublisher(const AutowareInfo & aw_info);

private:
  // publisher
  rclcpp::Publisher<autoware_api_msgs::msg::AwapiVehicleStatus>::SharedPtr pub_state_;

  autoware_api_msgs::msg::AwapiVehicleStatus initVehicleStatus();
  void getPoseInfo(
    const std::shared_ptr<geometry_msgs::msg::PoseStamped> & pose_ptr,
    autoware_api_msgs::msg::AwapiVehicleStatus * status);
  void getSteerInfo(
    const autoware_vehicle_msgs::msg::Steering::ConstSharedPtr & steer_ptr,
    autoware_api_msgs::msg::AwapiVehicleStatus * status);
  void getVehicleCmdInfo(
    const autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr & vehicle_cmd_ptr,
    autoware_api_msgs::msg::AwapiVehicleStatus * status);
  void getTurnSignalInfo(
    const autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr & turn_signal_ptr,
    autoware_api_msgs::msg::AwapiVehicleStatus * status);
  void getTwistInfo(
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr & twist_ptr,
    autoware_api_msgs::msg::AwapiVehicleStatus * status);
  void getGearInfo(
    const autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr & gear_ptr,
    autoware_api_msgs::msg::AwapiVehicleStatus * status);
  void getBatteryInfo(
    const std_msgs::msg::Float32::ConstSharedPtr & battery_ptr,
    autoware_api_msgs::msg::AwapiVehicleStatus * status);
  void getGpsInfo(
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr & nav_sat_ptr,
    autoware_api_msgs::msg::AwapiVehicleStatus * status);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  //parameters
  geometry_msgs::msg::TwistStamped::ConstSharedPtr previous_twist_ptr_;
  autoware_vehicle_msgs::msg::Steering::ConstSharedPtr previous_steer_ptr_;
  double prev_accel_;
  double prev_steer_vel_;

  // defined value
  const double accel_lowpass_gain_ = 0.2;
  const double steer_vel_lowpass_gain_ = 0.2;
};

}  // namespace autoware_api
