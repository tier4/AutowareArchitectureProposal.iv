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

#ifndef MOTION_VELOCITY_OPTIMIZER__OPTIMIZER__OPTIMIZER_BASE_HPP_
#define MOTION_VELOCITY_OPTIMIZER__OPTIMIZER__OPTIMIZER_BASE_HPP_

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <limits>
#include <vector>
struct OptimizerParam
{
  double max_accel;
  double min_decel;
  double pseudo_jerk_weight;
  double over_v_weight;
  double over_a_weight;
};

class OptimizerBase
{
public:
  virtual bool solve(
    const double initial_vel, const double initial_acc, const int closest,
    const autoware_planning_msgs::msg::Trajectory & input,
    autoware_planning_msgs::msg::Trajectory * output) = 0;

  virtual void setParam(const OptimizerParam & param) = 0;
  virtual ~OptimizerBase() = default;
};

#endif  // MOTION_VELOCITY_OPTIMIZER__OPTIMIZER__OPTIMIZER_BASE_HPP_
