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

#ifndef MOTION_VELOCITY_OPTIMIZER__OPTIMIZER__LINF_PSEUDO_JERK_OPTIMIZER_HPP_
#define MOTION_VELOCITY_OPTIMIZER__OPTIMIZER__LINF_PSEUDO_JERK_OPTIMIZER_HPP_

#include "motion_velocity_optimizer/optimizer/optimizer_base.hpp"

#include <osqp_interface/osqp_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <vector>

class LinfPseudoJerkOptimizer : public OptimizerBase
{
public:
  explicit LinfPseudoJerkOptimizer(const OptimizerParam & p);
  virtual ~LinfPseudoJerkOptimizer() = default;

  bool solve(
    const double initial_vel, const double initial_acc, const int closest,
    const autoware_planning_msgs::msg::Trajectory & input,
    autoware_planning_msgs::msg::Trajectory * output) override;

  void setParam(const OptimizerParam & param) override;

private:
  OptimizerParam param_;
  osqp::OSQPInterface qp_solver_;
};

#endif  // MOTION_VELOCITY_OPTIMIZER__OPTIMIZER__LINF_PSEUDO_JERK_OPTIMIZER_HPP_
