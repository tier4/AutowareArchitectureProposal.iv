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

#ifndef MOTION_VELOCITY_SMOOTHER__SMOOTHER__JERK_FILTERED_SMOOTHER_HPP_
#define MOTION_VELOCITY_SMOOTHER__SMOOTHER__JERK_FILTERED_SMOOTHER_HPP_

#include "motion_velocity_smoother/smoother/smoother_base.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/trajectory/trajectory.hpp>
#include <osqp_interface/osqp_interface.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <boost/optional.hpp>

#include <vector>

namespace motion_velocity_smoother
{
using autoware_planning_msgs::msg::Trajectory;

class JerkFilteredSmoother : public SmootherBase
{
public:
  struct Param
  {
    double jerk_weight;
    double over_v_weight;
    double over_a_weight;
    double over_j_weight;
    double jerk_filter_ds;
  };

  explicit JerkFilteredSmoother(const Param & p);

  bool apply(
    const double initial_vel, const double initial_acc, const Trajectory & input,
    Trajectory & output, std::vector<Trajectory> & debug_trajectories) override;

  boost::optional<Trajectory> resampleTrajectory(
    const Trajectory & input, const double v_current, const int closest_id) const override;

  void setParam(const Param & param);

private:
  Param smoother_param_;
  osqp::OSQPInterface qp_solver_;
  rclcpp::Logger logger_{rclcpp::get_logger("smoother").get_child("jerk_filtered_smoother")};

  Trajectory forwardJerkFilter(
    const double v0, const double a0, const double a_max, const double a_stop, const double j_max,
    const Trajectory & input) const;
  Trajectory backwardJerkFilter(
    const double v0, const double a0, const double a_min, const double a_stop, const double j_min,
    const Trajectory & input) const;
  Trajectory mergeFilteredTrajectory(
    const double v0, const double a0, const double a_min, const double j_min,
    const Trajectory & forward_filtered, const Trajectory & backward_filtered) const;
};
}  // namespace motion_velocity_smoother

#endif  // MOTION_VELOCITY_SMOOTHER__SMOOTHER__JERK_FILTERED_SMOOTHER_HPP_
