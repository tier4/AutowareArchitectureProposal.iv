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

#ifndef MOTION_VELOCITY_OPTIMIZER_L2_PSEUDO_JERK_OPTIMIZER_HPP
#define MOTION_VELOCITY_OPTIMIZER_L2_PSEUDO_JERK_OPTIMIZER_HPP

#include <autoware_planning_msgs/Trajectory.h>
#include <osqp_interface/osqp_interface.h>
#include <ros/ros.h>
#include <vector>

#include <motion_velocity_optimizer/optimizer/optimizer_base.hpp>

class L2PseudoJerkOptimizer : public OptimizerBase
{
public:
  struct OptimizerParam
  {
    double max_accel;
    double min_decel;
    double pseudo_jerk_weight;
    double over_v_weight;
    double over_a_weight;
  };

public:
  explicit L2PseudoJerkOptimizer(const OptimizerParam & p);
  bool solve(
    const double initial_vel, const double initial_acc, const int closest,
    const autoware_planning_msgs::Trajectory & input,
    autoware_planning_msgs::Trajectory * output) override;

  void setAccel(const double max_accel) override;

  void setDecel(const double min_decel) override;

private:
  OptimizerParam param_;
  osqp::OSQPInterface qp_solver_;
};

#endif  // MOTION_VELOCITY_OPTIMIZER_L2_PSEUDO_JERK_OPTIMIZER_HPP
