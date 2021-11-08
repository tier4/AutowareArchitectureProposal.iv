// Copyright 2015-2019 Autoware Foundation
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

#ifndef VEHICLE_CMD_GATE__VEHICLE_CMD_FILTER_HPP_
#define VEHICLE_CMD_GATE__VEHICLE_CMD_FILTER_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

class VehicleCmdFilter
{
public:
  VehicleCmdFilter();
  ~VehicleCmdFilter() = default;

  void setWheelBase(float v) { wheel_base_ = v; }
  void setVelLim(float v) { vel_lim_ = v; }
  void setLonAccLim(float v) { lon_acc_lim_ = v; }
  void setLonJerkLim(float v) { lon_jerk_lim_ = v; }
  void setLatAccLim(float v) { lat_acc_lim_ = v; }
  void setLatJerkLim(float v) { lat_jerk_lim_ = v; }
  void setPrevCmd(const autoware_auto_control_msgs::msg::AckermannControlCommand & v)
  {
    prev_cmd_ = v;
  }

  void limitLongitudinalWithVel(
    autoware_auto_control_msgs::msg::AckermannControlCommand & input) const;
  void limitLongitudinalWithAcc(
    const float dt, autoware_auto_control_msgs::msg::AckermannControlCommand & input) const;
  void limitLongitudinalWithJerk(
    const float dt, autoware_auto_control_msgs::msg::AckermannControlCommand & input) const;
  void limitLateralWithLatAcc(
    const float dt, autoware_auto_control_msgs::msg::AckermannControlCommand & input) const;
  void limitLateralWithLatJerk(
    const float dt, autoware_auto_control_msgs::msg::AckermannControlCommand & input) const;

private:
  float wheel_base_;
  float vel_lim_;
  float lon_acc_lim_;
  float lon_jerk_lim_;
  float lat_acc_lim_;
  float lat_jerk_lim_;
  autoware_auto_control_msgs::msg::AckermannControlCommand & prev_cmd_;

  auto calcLatAcc(const autoware_auto_control_msgs::msg::AckermannControlCommand & cmd) const;
  auto calcSteerFromLatacc(const float v, const float latacc) const;
  auto limitDiff(const float curr, const float prev, const float diff_lim) const;
};

#endif  // VEHICLE_CMD_GATE__VEHICLE_CMD_FILTER_HPP_
