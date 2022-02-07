// Copyright 2021 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc_4ws.hpp"

#include <algorithm>

SimModelDelaySteerAcc4ws::SimModelDelaySteerAcc4ws(
    float64_t vx_lim, float64_t f_steer_lim, float64_t r_steer_lim, float64_t vx_rate_lim, float64_t f_steer_rate_lim, float64_t r_steer_rate_lim,
    float64_t wheelbase, float64_t dt, float64_t acc_delay, float64_t acc_time_constant,
    float64_t f_steer_delay, float64_t f_steer_time_constant,
    float64_t r_steer_delay, float64_t r_steer_time_constant)
: SimModelInterface(7 /* dim x */, 3 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  f_steer_lim_(f_steer_lim),
  r_steer_lim_(r_steer_lim),
  f_steer_rate_lim_(f_steer_rate_lim),
  r_steer_rate_lim_(r_steer_rate_lim),
  wheelbase_(wheelbase),
  acc_delay_(acc_delay),
  acc_time_constant_(std::max(acc_time_constant, MIN_TIME_CONSTANT)),
  f_steer_delay_(f_steer_delay),
  f_steer_time_constant_(std::max(f_steer_time_constant, MIN_TIME_CONSTANT)),
  r_steer_delay_(r_steer_delay),
  r_steer_time_constant_(std::max(r_steer_time_constant, MIN_TIME_CONSTANT))
{
  initializeInputQueue(dt);
}

float64_t SimModelDelaySteerAcc4ws::getX() {return state_(IDX::X);}
float64_t SimModelDelaySteerAcc4ws::getY() {return state_(IDX::Y);}
float64_t SimModelDelaySteerAcc4ws::getYaw() {return state_(IDX::YAW);}
float64_t SimModelDelaySteerAcc4ws::getVx() {return state_(IDX::VX);}
float64_t SimModelDelaySteerAcc4ws::getVy() {return 0.0;}
float64_t SimModelDelaySteerAcc4ws::getAx() {return state_(IDX::ACCX);}
float64_t SimModelDelaySteerAcc4ws::getWz()
{
  return state_(IDX::VX) * std::cos((state_(IDX::R_STEER))) * (std::tan(state_(IDX::F_STEER))-std::tan(state_(IDX::R_STEER))) / wheelbase_;
}
float64_t SimModelDelaySteerAcc4ws::getSteer() {return state_(IDX::F_STEER);}
float64_t SimModelDelaySteerAcc4ws::getRearSteer() {return state_(IDX::R_STEER);}
void SimModelDelaySteerAcc4ws::update(const float64_t & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  acc_input_queue_.push_back(input_(IDX_U::ACCX_DES));
  delayed_input(IDX_U::ACCX_DES) = acc_input_queue_.front();
  acc_input_queue_.pop_front();
  f_steer_input_queue_.push_back(input_(IDX_U::F_STEER_DES));
  delayed_input(IDX_U::F_STEER_DES) = f_steer_input_queue_.front();
  f_steer_input_queue_.pop_front();
  r_steer_input_queue_.push_back(input_(IDX_U::R_STEER_DES));
  delayed_input(IDX_U::R_STEER_DES) = r_steer_input_queue_.front();
  r_steer_input_queue_.pop_front();

  const auto prev_vx = state_(IDX::VX);

  updateRungeKutta(dt, delayed_input);

  state_(IDX::VX) = std::max(-vx_lim_, std::min(state_(IDX::VX), vx_lim_));

  state_(IDX::VX) = calcVelocityWithGear(state_, gear_);

  // calc acc directly after gear consideration
  state_(IDX::ACCX) = (state_(IDX::VX) - prev_vx) / std::max(dt, 1.0e-5);
}

void SimModelDelaySteerAcc4ws::initializeInputQueue(const float64_t & dt)
{
  size_t acc_input_queue_size = static_cast<size_t>(round(acc_delay_ / dt));
  acc_input_queue_.resize(acc_input_queue_size);
  std::fill(acc_input_queue_.begin(), acc_input_queue_.end(), 0.0);

  size_t f_steer_input_queue_size = static_cast<size_t>(round(f_steer_delay_ / dt));
  f_steer_input_queue_.resize(f_steer_input_queue_size);
  std::fill(f_steer_input_queue_.begin(), f_steer_input_queue_.end(), 0.0);
  
  size_t r_steer_input_queue_size = static_cast<size_t>(round(r_steer_delay_ / dt));
  r_steer_input_queue_.resize(r_steer_input_queue_size);
  std::fill(r_steer_input_queue_.begin(), r_steer_input_queue_.end(), 0.0);
}

Eigen::VectorXd SimModelDelaySteerAcc4ws::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  auto sat = [](float64_t val, float64_t u, float64_t l) {return std::max(std::min(val, u), l);};

  const float64_t vel = sat(state(IDX::VX), vx_lim_, -vx_lim_);
  const float64_t acc = sat(state(IDX::ACCX), vx_rate_lim_, -vx_rate_lim_);
  const float64_t yaw = state(IDX::YAW);
  const float64_t f_steer = state(IDX::F_STEER);
  const float64_t r_steer = state(IDX::R_STEER);
  const float64_t acc_des = sat(input(IDX_U::ACCX_DES), vx_rate_lim_, -vx_rate_lim_);
  const float64_t f_steer_des = sat(input(IDX_U::F_STEER_DES), f_steer_lim_, -f_steer_lim_);
  const float64_t r_steer_des = sat(input(IDX_U::R_STEER_DES), r_steer_lim_, -r_steer_lim_);
  float64_t f_steer_rate = -(f_steer - f_steer_des) / f_steer_time_constant_;
  float64_t r_steer_rate = -(r_steer - r_steer_des) / r_steer_time_constant_;
  f_steer_rate = sat(f_steer_rate, f_steer_rate_lim_, -f_steer_rate_lim_);
  r_steer_rate = sat(r_steer_rate, r_steer_rate_lim_, -r_steer_rate_lim_);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vel * cos(yaw+r_steer);
  d_state(IDX::Y) = vel * sin(yaw+r_steer);
  d_state(IDX::YAW) = vel * cos(r_steer)*(std::tan(f_steer)-std::tan(r_steer)) / wheelbase_;
  d_state(IDX::VX) = acc;
  d_state(IDX::F_STEER) = f_steer_rate;
  d_state(IDX::R_STEER) = r_steer_rate;
  d_state(IDX::ACCX) = -(acc - acc_des) / acc_time_constant_;

  return d_state;
}

float64_t SimModelDelaySteerAcc4ws::calcVelocityWithGear(
  const Eigen::VectorXd & state, const uint8_t gear) const
{
  using autoware_auto_vehicle_msgs::msg::GearCommand;
  if (
    gear == GearCommand::DRIVE || gear == GearCommand::DRIVE_2 || gear == GearCommand::DRIVE_3 ||
    gear == GearCommand::DRIVE_4 || gear == GearCommand::DRIVE_5 || gear == GearCommand::DRIVE_6 ||
    gear == GearCommand::DRIVE_7 || gear == GearCommand::DRIVE_8 || gear == GearCommand::DRIVE_9 ||
    gear == GearCommand::DRIVE_10 || gear == GearCommand::DRIVE_11 ||
    gear == GearCommand::DRIVE_12 || gear == GearCommand::DRIVE_13 ||
    gear == GearCommand::DRIVE_14 || gear == GearCommand::DRIVE_15 ||
    gear == GearCommand::DRIVE_16 || gear == GearCommand::DRIVE_17 ||
    gear == GearCommand::DRIVE_18 || gear == GearCommand::LOW || gear == GearCommand::LOW_2) {
    if (state(IDX::VX) < 0.0) {
      return 0.0;
    }
  } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
    if (state(IDX::VX) > 0.0) {
      return 0.0;
    }
  } else if (gear == GearCommand::PARK) {
    return 0.0;
  } else {
    return 0.0;
  }

  return state(IDX::VX);
}
