// Copyright 2018-2019 Tier IV, Inc.
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

#ifndef VELOCITY_CONTROLLER__PID_HPP_
#define VELOCITY_CONTROLLER__PID_HPP_

#include <vector>

class PIDController
{
public:
  PIDController();

  double calculate(
    const double error, const double dt, const bool is_integrated,
    std::vector<double> & pid_contributions);
  void setGains(const double kp, const double ki, const double kd);
  void setLimits(
    const double max_ret, const double min_ret, const double max_ret_p, const double min_ret_p,
    const double max_ret_i, const double min_ret_i, const double max_ret_d, const double min_ret_d);
  void reset();

private:
  // parameters
  struct Params
  {
    double kp;
    double ki;
    double kd;
    double max_ret_p;
    double min_ret_p;
    double max_ret_i;
    double min_ret_i;
    double max_ret_d;
    double min_ret_d;
    double max_ret;
    double min_ret;
  };
  Params params_;

  // states
  double error_integral_;
  double prev_error_;
  bool is_first_time_;
};

#endif  // VELOCITY_CONTROLLER__PID_HPP_
