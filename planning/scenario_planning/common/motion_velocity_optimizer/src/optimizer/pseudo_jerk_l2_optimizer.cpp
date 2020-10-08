
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
#include <chrono>
#include <eigen3/Eigen/Core>
#include <motion_velocity_optimizer/motion_velocity_optimizer_utils.hpp>
#include <motion_velocity_optimizer/optimizer/l2_pseudo_jerk_optimizer.hpp>

L2PseudoJerkOptimizer::L2PseudoJerkOptimizer(const L2PseudoJerkOptimizer::OptimizerParam & p)
{
  param_ = p;
}

void L2PseudoJerkOptimizer::setAccel(const double max_accel) { param_.max_accel = max_accel; }

void L2PseudoJerkOptimizer::setDecel(const double min_decel) { param_.min_decel = min_decel; }

bool L2PseudoJerkOptimizer::solve(
  const double initial_vel, const double initial_acc, const int closest,
  const autoware_planning_msgs::Trajectory & input, autoware_planning_msgs::Trajectory * output)
{
  auto ts = std::chrono::system_clock::now();

  *output = input;

  if (static_cast<int>(input.points.size()) < closest) {
    ROS_WARN("[MotionVelocityOptimizer] invalid closest.");
    return false;
  }

  if (std::fabs(input.points.at(closest).twist.linear.x) < 0.1) {
    ROS_DEBUG("[MotionVelocityOptimizer] closest vmax < 0.1. assume vehicle stopped. return.");
    return false;
  }

  const unsigned int N = input.points.size() - closest;

  if (N < 2) {
    ROS_WARN("[MotionVelocityOptimizer] trajectory length is not enough.");
    return false;
  }

  std::vector<double> interval_dist_arr;
  vpu::calcTrajectoryIntervalDistance(input, interval_dist_arr);

  std::vector<double> vmax(N, 0.0);
  for (unsigned int i = 0; i < N; ++i) {
    vmax.at(i) = input.points.at(i + closest).twist.linear.x;
  }

  /*
   * x = [b0, b1, ..., bN, |  a0, a1, ..., aN, | delta0, delta1, ..., deltaN, | sigma0, sigme1, ..., sigmaN] in R^{4N}
   * b: velocity^2
   * a: acceleration
   * delta: 0 < bi < vmax^2 + delta
   * sigma: amin < ai - sigma < amax
   */

  const uint32_t l_variables = 4 * N;
  const uint32_t l_constraints = 3 * N + 1;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(
    l_constraints, l_variables);  // the matrix size depends on constraint numbers.

  std::vector<double> lower_bound(l_constraints, 0.0);
  std::vector<double> upper_bound(l_constraints, 0.0);

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(l_variables, l_variables);
  std::vector<double> q(l_variables, 0.0);

  const double amax = param_.max_accel;
  const double amin = param_.min_decel;
  const double smooth_weight = param_.pseudo_jerk_weight;
  const double over_v_weight = param_.over_v_weight;
  const double over_a_weight = param_.over_a_weight;

  /* design objective function */
  for (unsigned int i = 0; i < N; ++i) {  // bi
    q[i] = -1.0;                          // |vmax^2 - b| -> minimize (-bi)
  }

  // pseudo jerk: d(ai)/ds -> minimize weight * (a1 - a0)^2
  for (unsigned int i = N; i < 2 * N - 1; ++i) {
    unsigned int j = i - N;
    const double w_x_dsinv = smooth_weight * (1.0 / std::max(interval_dist_arr.at(j), 0.0001));
    P(i, i) += w_x_dsinv;
    P(i, i + 1) -= w_x_dsinv;
    P(i + 1, i) -= w_x_dsinv;
    P(i + 1, i + 1) += w_x_dsinv;
  }

  for (unsigned int i = 2 * N; i < 3 * N; ++i) {  // over velocity cost
    P(i, i) += over_v_weight;
  }

  for (unsigned int i = 3 * N; i < 4 * N; ++i) {  // over acceleration cost
    P(i, i) += over_a_weight;
  }

  /* design constraint matrix */
  // 0 < b - delta < vmax^2
  // NOTE: The delta allows b to be negative. This is actully invalid because the definition is b=v^2.
  // But mathematically, the strict b>0 constraint may make the problem infeasible, such as the case of
  // v=0 & a<0. To avoid the infesibility, we allow b<0. The negative b is dealt as b=0 when it is
  // converted to v with sqrt. If the weight of delta^2 is large (the value of delta is very small),
  // b is almost 0, and is not a big problem.
  for (unsigned int i = 0; i < N; ++i) {
    const int j = 2 * N + i;
    A(i, i) = 1.0;   // b_i
    A(i, j) = -1.0;  // -delta_i
    upper_bound[i] = vmax[i] * vmax[i];
    lower_bound[i] = 0.0;
  }

  // amin < a - sigma < amax
  for (unsigned int i = N; i < 2 * N; ++i) {
    const int j = 2 * N + i;
    A(i, i) = 1.0;   // a_i
    A(i, j) = -1.0;  // -sigma_i
    if (i != N && vmax[i - N] < std::numeric_limits<double>::epsilon()) {
      upper_bound[i] = 0.0;
      lower_bound[i] = 0.0;
    } else {
      upper_bound[i] = amax;
      lower_bound[i] = amin;
    }
  }

  // b' = 2a
  for (unsigned int i = 2 * N; i < 3 * N - 1; ++i) {
    const unsigned int j = i - 2 * N;
    const double dsinv = 1.0 / std::max(interval_dist_arr.at(j), 0.0001);
    A(i, j) = -dsinv;
    A(i, j + 1) = dsinv;
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

  auto tf1 = std::chrono::system_clock::now();
  double dt_ms1 = std::chrono::duration_cast<std::chrono::nanoseconds>(tf1 - ts).count() * 1.0e-6;

  /* execute optimization */
  auto ts2 = std::chrono::system_clock::now();
  const auto result = qp_solver_.optimize(P, A, q, lower_bound, upper_bound);

  // [b0, b1, ..., bN, |  a0, a1, ..., aN, | delta0, delta1, ..., deltaN, | sigma0, sigme1, ..., sigmaN]
  const std::vector<double> optval = std::get<0>(result);

  /* get velocity & acceleration */
  for (int i = 0; i < closest; ++i) {
    double v = optval.at(0);
    output->points.at(i).twist.linear.x = std::sqrt(std::max(v, 0.0));
    output->points.at(i).accel.linear.x = optval.at(N);
  }
  for (unsigned int i = 0; i < N; ++i) {
    double v = optval.at(i);
    output->points.at(i + closest).twist.linear.x = std::sqrt(std::max(v, 0.0));
    output->points.at(i + closest).accel.linear.x = optval.at(i + N);
  }
  for (unsigned int i = N + closest; i < output->points.size(); ++i) {
    output->points.at(i).twist.linear.x = 0.0;
    output->points.at(i).accel.linear.x = 0.0;
  }

  // -- to check the all optimization variables --
  // ROS_DEBUG("[after optimize Linf] idx, vel, acc, over_vel, over_acc ");
  // for (unsigned int i = 0; i < N; ++i) {
  //   ROS_DEBUG(
  //     "i = %d, v: %f, vmax: %f a: %f, b: %f, delta: %f, sigma: %f\n", i, std::sqrt(optval.at(i)),
  //     vmax[i], optval.at(i + N), optval.at(i), optval.at(i + 2 * N), optval.at(i + 3 * N));
  // }

  auto tf2 = std::chrono::system_clock::now();
  double dt_ms2 = std::chrono::duration_cast<std::chrono::nanoseconds>(tf2 - ts2).count() * 1.0e-6;
  ROS_DEBUG("[optimization] init time = %f [ms], optimization time = %f [ms]", dt_ms1, dt_ms2);

  return true;
}
