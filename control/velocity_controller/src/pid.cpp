// Copyright 2018 Tier IV, Inc. All rights reserved.
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

#include "velocity_controller/pid.hpp"

#include <algorithm>
#include <array>
#include <memory>
#include <utility>

PIDController::PIDController() : error_integral_(0.0), prev_error_(0.0), is_first_time_(true) {}

double PIDController::calculate(
  const double error, const double dt, const bool enable_integration,
  std::array<double, 3> & pid_contributions)
{
  const auto & p = params_;

  double ret_p = p.kp * error;
  ret_p = std::min(std::max(ret_p, p.min_ret_p), p.max_ret_p);

  if (enable_integration) {
    error_integral_ += error * dt;
    error_integral_ = std::min(std::max(error_integral_, p.min_ret_i / p.ki), p.max_ret_i / p.ki);
  }
  const double ret_i = p.ki * error_integral_;

  double error_differential;
  if (is_first_time_) {
    error_differential = 0;
    is_first_time_ = false;
  } else {
    error_differential = (error - prev_error_) / dt;
  }
  double ret_d = p.kd * error_differential;
  ret_d = std::min(std::max(ret_d, p.min_ret_d), p.max_ret_d);

  prev_error_ = error;

  pid_contributions.at(0) = ret_p;
  pid_contributions.at(1) = ret_i;
  pid_contributions.at(2) = ret_d;

  double ret = ret_p + ret_i + ret_d;
  ret = std::min(std::max(ret, p.min_ret), p.max_ret);

  return ret;
}

void PIDController::setGains(const double kp, const double ki, const double kd)
{
  params_.kp = kp;
  params_.ki = ki;
  params_.kd = kd;
}

void PIDController::setLimits(
  const double max_ret, const double min_ret, const double max_ret_p, const double min_ret_p,
  const double max_ret_i, const double min_ret_i, const double max_ret_d, const double min_ret_d)
{
  params_.max_ret = max_ret;
  params_.min_ret = min_ret;
  params_.max_ret_p = max_ret_p;
  params_.min_ret_p = min_ret_p;
  params_.max_ret_d = max_ret_d;
  params_.min_ret_d = min_ret_d;
  params_.max_ret_i = max_ret_i;
  params_.min_ret_i = min_ret_i;
}

void PIDController::reset()
{
  error_integral_ = 0.0;
  prev_error_ = 0.0;
  is_first_time_ = true;
}
