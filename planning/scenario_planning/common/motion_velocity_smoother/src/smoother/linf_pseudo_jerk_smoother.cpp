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

#include "motion_velocity_smoother/smoother/linf_pseudo_jerk_smoother.hpp"

#include "motion_velocity_smoother/trajectory_utils.hpp"

#include <eigen3/Eigen/Core>

#include <algorithm>
#include <chrono>
#include <limits>
#include <vector>

namespace motion_velocity_smoother
{
LinfPseudoJerkSmoother::LinfPseudoJerkSmoother(const Param & smoother_param)
: smoother_param_(smoother_param)
{
  qp_solver_.updateMaxIter(20000);
  qp_solver_.updateRhoInterval(5000);
  qp_solver_.updateEpsRel(1.0e-4);  // def: 1.0e-4
  qp_solver_.updateEpsAbs(1.0e-8);  // def: 1.0e-4
  qp_solver_.updateVerbose(false);
}

void LinfPseudoJerkSmoother::setParam(const Param & smoother_param)
{
  smoother_param_ = smoother_param;
}

bool LinfPseudoJerkSmoother::apply(
  const double initial_vel, const double initial_acc, const Trajectory & input, Trajectory & output,
  std::vector<Trajectory> & debug_trajectories)
{
  debug_trajectories.clear();

  const auto ts = std::chrono::system_clock::now();

  output = input;

  if (std::fabs(input.points.front().twist.linear.x) < 0.1) {
    RCLCPP_DEBUG(
      logger_,
      "closest v_max < 0.1, keep stopping. "
      "return.");
    return false;
  }

  const size_t N{input.points.size()};

  if (N < 2) {
    return false;
  }

  std::vector<double> interval_dist_arr = trajectory_utils::calcTrajectoryIntervalDistance(input);

  std::vector<double> v_max(N, 0.0);
  for (size_t i = 0; i < N; ++i) {
    v_max.at(i) = input.points.at(i).twist.linear.x;
  }

  /*
   * x = [b0, b1, ..., bN, |  a0, a1, ..., aN, | delta0, delta1, ..., deltaN, | sigma0, sigma1, ...,
   * sigmaN, | psi] in R^{4N+1} b: velocity^2 a: acceleration delta: 0 < bi < v_max^2 + delta sigma:
   * a_min < ai - sigma < a_max psi: a'*curr_v -  psi < 0, - a'*curr_v - psi < 0 (<=> |a'|*curr_v <
   * psi)
   */
  const size_t l_variables{4 * N + 1};
  const size_t l_constraints{3 * N + 1 + 2 * (N - 1)};

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(
    l_constraints, l_variables);  // the matrix size depends on constraint numbers.

  std::vector<double> lower_bound(l_constraints, 0.0);
  std::vector<double> upper_bound(l_constraints, 0.0);

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(l_variables, l_variables);
  std::vector<double> q(l_variables, 0.0);

  const double a_max{base_param_.max_accel};
  const double a_min{base_param_.min_decel};
  const double smooth_weight{smoother_param_.pseudo_jerk_weight};
  const double over_v_weight{smoother_param_.over_v_weight};
  const double over_a_weight{smoother_param_.over_a_weight};

  // design objective function
  for (unsigned int i = 0; i < N; ++i) {  // bi
    q[i] = -1.0;                          // |v_max^2 - b| -> minimize (-bi)
  }

  for (unsigned int i = 2 * N; i < 3 * N; ++i) {  // over velocity cost
    P(i, i) += over_v_weight;
  }

  for (unsigned int i = 3 * N; i < 4 * N; ++i) {  // over acceleration cost
    P(i, i) += over_a_weight;
  }

  // pseudo jerk (Linf): minimize psi, subject to |a'|*curr_v < psi
  q[4 * N] = smooth_weight;

  /* design constraint matrix
  0 < b - delta < v_max^2
  NOTE: The delta allows b to be negative. This is actually invalid because the definition is b=v^2.
  But mathematically, the strict b>0 constraint may make the problem infeasible, such as the case of
  v=0 & a<0. To avoid the infeasibility, we allow b<0. The negative b is dealt as b=0 when it is
  converted to v with sqrt. If the weight of delta^2 is large (the value of delta is very small),
  b is almost 0, and is not a big problem.
  */
  for (unsigned int i = 0; i < N; ++i) {
    const int j = 2 * N + i;
    A(i, i) = 1.0;   // b_i
    A(i, j) = -1.0;  // -delta_i
    upper_bound[i] = v_max[i] * v_max[i];
    lower_bound[i] = 0.0;
  }

  // a_min < a - sigma < a_max
  for (unsigned int i = N; i < 2 * N; ++i) {
    const int j = 2 * N + i;
    A(i, i) = 1.0;   // a_i
    A(i, j) = -1.0;  // -sigma_i
    if (i != N && v_max[i - N] < std::numeric_limits<double>::epsilon()) {
      upper_bound[i] = 0.0;
      lower_bound[i] = 0.0;
    } else {
      upper_bound[i] = a_max;
      lower_bound[i] = a_min;
    }
  }

  // b' = 2a
  for (unsigned int i = 2 * N; i < 3 * N - 1; ++i) {
    const unsigned int j = i - 2 * N;
    const double ds_inv = 1.0 / std::max(interval_dist_arr.at(j), 0.0001);
    A(i, j) = -ds_inv;
    A(i, j + 1) = ds_inv;
    A(i, j + N) = -2.0;
    upper_bound[i] = 0.0;
    lower_bound[i] = 0.0;
  }

  // initial condition
  const double v0 = initial_vel;
  {
    const unsigned int i = 3 * N - 1;
    A(i, 0) = 1.0;  // b0
    upper_bound[i] = v0 * v0;
    lower_bound[i] = v0 * v0;

    A(i + 1, N) = 1.0;  // a0
    upper_bound[i + 1] = initial_acc;
    lower_bound[i + 1] = initial_acc;
  }

  // constraint for slack variable (a[i+1] - a[i] <= psi[i], a[i] - a[i+1] <= psi[i])
  for (unsigned int i = 3 * N + 1; i < 4 * N; ++i) {
    const unsigned int ia = i - (3 * N + 1) + N;
    const unsigned int ip = 4 * N;
    const unsigned int j = i - (3 * N + 1);
    const double ds_inv = 1.0 / std::max(interval_dist_arr.at(j), 0.0001);

    A(i, ia) = -ds_inv;
    A(i, ia + 1) = ds_inv;
    A(i, ip) = -1;
    lower_bound[i] = -OSQP_INFTY;
    upper_bound[i] = 0;

    A(i + N - 1, ia) = ds_inv;
    A(i + N - 1, ia + 1) = -ds_inv;
    A(i + N - 1, ip) = -1;
    lower_bound[i + N - 1] = -OSQP_INFTY;
    upper_bound[i + N - 1] = 0;
  }

  const auto tf1 = std::chrono::system_clock::now();
  const double dt_ms1 =
    std::chrono::duration_cast<std::chrono::nanoseconds>(tf1 - ts).count() * 1.0e-6;

  // execute optimization
  const auto ts2 = std::chrono::system_clock::now();
  const auto result = qp_solver_.optimize(P, A, q, lower_bound, upper_bound);

  // [b0, b1, ..., bN, |  a0, a1, ..., aN, |
  //  delta0, delta1, ..., deltaN, | sigma0, sigma1, ..., sigmaN]
  const std::vector<double> optval = std::get<0>(result);

  /* get velocity & acceleration */
  for (unsigned int i = 0; i < N; ++i) {
    double v = optval.at(i);
    output.points.at(i).twist.linear.x = std::sqrt(std::max(v, 0.0));
    output.points.at(i).accel.linear.x = optval.at(i + N);
  }
  for (unsigned int i = N; i < output.points.size(); ++i) {
    output.points.at(i).twist.linear.x = 0.0;
    output.points.at(i).accel.linear.x = 0.0;
  }

  // -- to check the all optimization variables --
  // ROS_DEBUG("[after optimize Linf] idx, vel, acc, over_vel, over_acc ");
  // for (unsigned int i = 0; i < N; ++i) {
  //   ROS_DEBUG(
  //     "i = %d, v: %f, v_max: %f a: %f, b: %f, delta: %f, sigma: %f", i, std::sqrt(optval.at(i)),
  //     v_max[i], optval.at(i + N), optval.at(i), optval.at(i + 2 * N), optval.at(i + 3 * N));
  // }

  const int status_val = std::get<3>(result);
  if (status_val != 1) {
    RCLCPP_WARN(logger_, "optimization failed : %s", qp_solver_.getStatusMessage().c_str());
  }

  const auto tf2 = std::chrono::system_clock::now();
  const double dt_ms2 =
    std::chrono::duration_cast<std::chrono::nanoseconds>(tf2 - ts2).count() * 1.0e-6;
  RCLCPP_DEBUG(logger_, "init time = %f [ms], optimization time = %f [ms]", dt_ms1, dt_ms2);
  return true;
}

boost::optional<Trajectory> LinfPseudoJerkSmoother::resampleTrajectory(
  const Trajectory & input, const double v_current, const int closest_id) const
{
  return resampling::resampleTrajectory(input, v_current, closest_id, base_param_.resample_param);
}

}  // namespace motion_velocity_smoother
