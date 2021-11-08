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

// *INDENT-OFF*
#ifndef MOTION_VELOCITY_SMOOTHER__SMOOTHER__ANALYTICAL_JERK_CONSTRAINED_SMOOTHER__ANALYTICAL_JERK_CONSTRAINED_SMOOTHER_HPP_  // NOLINT
#define MOTION_VELOCITY_SMOOTHER__SMOOTHER__ANALYTICAL_JERK_CONSTRAINED_SMOOTHER__ANALYTICAL_JERK_CONSTRAINED_SMOOTHER_HPP_  // NOLINT
// *INDENT-ON*

#include "autoware_utils/trajectory/trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"

#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <string>
#include <utility>
#include <vector>

// *INDENT-OFF*
#include "motion_velocity_smoother/smoother/analytical_jerk_constrained_smoother/velocity_planning_utils.hpp"
// *INDENT-ON*
#include "motion_velocity_smoother/smoother/smoother_base.hpp"

namespace motion_velocity_smoother
{
class AnalyticalJerkConstrainedSmoother : public SmootherBase
{
public:
  struct Param
  {
    struct
    {
      double ds_resample;
      double num_resample;
      double delta_yaw_threshold;
    } resample;
    struct
    {
      bool enable_constant_velocity_while_turning;
      double constant_velocity_dist_threshold;
    } latacc;
    struct
    {
      double max_acc;
      double min_acc;
      double max_jerk;
      double min_jerk;
      double kp;
    } forward;
    struct
    {
      double start_jerk;
      double min_jerk_mild_stop;
      double min_jerk;
      double min_acc_mild_stop;
      double min_acc;
      double span_jerk;
    } backward;
  };

  explicit AnalyticalJerkConstrainedSmoother(const Param & p);

  bool apply(
    const double initial_vel, const double initial_acc, const TrajectoryPoints & input,
    TrajectoryPoints & output, std::vector<TrajectoryPoints> & debug_trajectories) override;

  boost::optional<TrajectoryPoints> resampleTrajectory(
    const TrajectoryPoints & input, const double v_current, const int closest_id) const override;

  boost::optional<TrajectoryPoints> applyLateralAccelerationFilter(
    const TrajectoryPoints & input) const override;

  void setParam(const Param & param);

private:
  Param smoother_param_;
  rclcpp::Logger logger_{
    rclcpp::get_logger("smoother").get_child("analytical_jerk_constrained_smoother")};

  bool searchDecelTargetIndices(
    const TrajectoryPoints & trajectory, const size_t closest_index,
    std::vector<std::pair<size_t, double>> & decel_target_indices) const;
  bool applyForwardJerkFilter(
    const TrajectoryPoints & base_trajectory, const size_t start_index, const double initial_vel,
    const double initial_acc, const Param & params, TrajectoryPoints & output_trajectory) const;
  bool applyBackwardDecelFilter(
    const std::vector<size_t> & start_indices, const size_t decel_target_index,
    const double decel_target_vel, const Param & params,
    TrajectoryPoints & output_trajectory) const;
  bool calcEnoughDistForDecel(
    const TrajectoryPoints & trajectory, const size_t start_index, const double decel_target_vel,
    const double planning_jerk, const Param & params, const std::vector<double> & dist_to_target,
    bool & is_enough_dist, int & type, std::vector<double> & times, double & stop_dist) const;
  bool applyDecelVelocityFilter(
    const size_t decel_start_index, const double decel_target_vel, const double planning_jerk,
    const Param & params, const int type, const std::vector<double> & times,
    TrajectoryPoints & output_trajectory) const;

  // debug
  std::string strTimes(const std::vector<double> & times) const;
  std::string strStartIndices(const std::vector<size_t> & start_indices) const;
};
}  // namespace motion_velocity_smoother

#endif
