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

#ifndef MOTION_VELOCITY_SMOOTHER__SMOOTHER__LINF_PSEUDO_JERK_SMOOTHER_HPP_
#define MOTION_VELOCITY_SMOOTHER__SMOOTHER__LINF_PSEUDO_JERK_SMOOTHER_HPP_

#include <vector>

#include "boost/optional.hpp"

#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/trajectory/trajectory.hpp"
#include "osqp_interface/osqp_interface.hpp"

#include "motion_velocity_smoother/smoother/smoother_base.hpp"

namespace motion_velocity_smoother
{
using TrajectoryPointArray = std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>;
class LinfPseudoJerkSmoother : public SmootherBase
{
public:
  struct Param
  {
    double pseudo_jerk_weight;
    double over_v_weight;
    double over_a_weight;
  };

  explicit LinfPseudoJerkSmoother(const Param & smoother_param);

  bool apply(
    const double initial_vel, const double initial_acc,
    const TrajectoryPointArray & input, TrajectoryPointArray & output,
    std::vector<TrajectoryPointArray> & debug_trajectories) override;

  boost::optional<TrajectoryPointArray> resampleTrajectory(
    const TrajectoryPointArray & input, const double v_current,
    const int closest_id) const override;

  void setParam(const Param & smoother_param);

private:
  Param smoother_param_;
  osqp::OSQPInterface qp_solver_;
  rclcpp::Logger
    logger_{rclcpp::get_logger("smoother").get_child("linf_pseudo_jerk_smoother")};
};
}  // namespace motion_velocity_smoother

#endif  // MOTION_VELOCITY_SMOOTHER__SMOOTHER__LINF_PSEUDO_JERK_SMOOTHER_HPP_
