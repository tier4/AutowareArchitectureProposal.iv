// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "osqp/osqp.h"
#include "osqp_interface/csc_matrix_conv.hpp"
#include "osqp_interface/osqp_interface.hpp"

namespace autoware
{
namespace common
{
namespace osqp
{
OSQPInterface::OSQPInterface(const c_float eps_abs, const bool8_t polish)
{
  m_settings = std::make_unique<OSQPSettings>();
  m_data = std::make_unique<OSQPData>();

  if (m_settings) {
    osqp_set_default_settings(m_settings.get());
    m_settings->alpha = 1.6;  // Change alpha parameter
    m_settings->eps_rel = 1.0E-4;
    m_settings->eps_abs = eps_abs;
    m_settings->eps_prim_inf = 1.0E-4;
    m_settings->eps_dual_inf = 1.0E-4;
    m_settings->warm_start = true;
    m_settings->max_iter = 4000;
    m_settings->verbose = false;
    m_settings->polish = polish;
  }
  m_exitflag = 0;
}

OSQPInterface::OSQPInterface(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<float64_t> & q,
  const std::vector<float64_t> & l, const std::vector<float64_t> & u, const c_float eps_abs)
: OSQPInterface(eps_abs)
{
  initializeProblem(P, A, q, l, u);
}

int64_t OSQPInterface::initializeProblem(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<float64_t> & q,
  const std::vector<float64_t> & l, const std::vector<float64_t> & u)
{
  /*******************
   * SET UP MATRICES
   *******************/
  CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P);
  CSC_Matrix A_csc = calCSCMatrix(A);
  // Dynamic float arrays
  std::vector<float64_t> q_tmp(q.begin(), q.end());
  std::vector<float64_t> l_tmp(l.begin(), l.end());
  std::vector<float64_t> u_tmp(u.begin(), u.end());
  float64_t * q_dyn = q_tmp.data();
  float64_t * l_dyn = l_tmp.data();
  float64_t * u_dyn = u_tmp.data();

  /**********************
   * OBJECTIVE FUNCTION
   **********************/
  // Number of constraints
  c_int constr_m = A.rows();
  // Number of parameters
  m_param_n = P.rows();

  /*****************
   * POPULATE DATA
   *****************/
  m_data->m = constr_m;
  m_data->n = m_param_n;
  m_data->P = csc_matrix(
    m_data->n, m_data->n, static_cast<c_int>(P_csc.m_vals.size()), P_csc.m_vals.data(),
    P_csc.m_row_idxs.data(),
    P_csc.m_col_idxs.data());
  m_data->q = q_dyn;
  m_data->A = csc_matrix(
    m_data->m, m_data->n, static_cast<c_int>(A_csc.m_vals.size()), A_csc.m_vals.data(),
    A_csc.m_row_idxs.data(),
    A_csc.m_col_idxs.data());
  m_data->l = l_dyn;
  m_data->u = u_dyn;

  // Setup workspace
  m_exitflag = osqp_setup(&m_work, m_data.get(), m_settings.get());
  m_work_initialized = true;

  return m_exitflag;
}

OSQPInterface::~OSQPInterface()
{
  // Cleanup dynamic OSQP memory
  if (m_work) {
    osqp_cleanup(m_work);
  }
}

std::tuple<std::vector<float64_t>, std::vector<float64_t>, int64_t, int64_t> OSQPInterface::solve()
{
  // Solve Problem
  osqp_solve(m_work);

  /********************
   * EXTRACT SOLUTION
   ********************/
  float64_t * sol_x = m_work->solution->x;
  float64_t * sol_y = m_work->solution->y;
  std::vector<float64_t> sol_primal(sol_x, sol_x + m_param_n);
  std::vector<float64_t> sol_lagrange_multiplier(sol_y, sol_y + m_param_n);
  // Solver polish status
  int64_t status_polish = m_work->info->status_polish;
  // Solver solution status
  int64_t status_solution = m_work->info->status_val;
  // Result tuple
  std::tuple<std::vector<float64_t>, std::vector<float64_t>, int64_t, int64_t> result =
    std::make_tuple(sol_primal, sol_lagrange_multiplier, status_polish, status_solution);

  m_latest_work_info = *(m_work->info);

  return result;
}

std::tuple<std::vector<float64_t>, std::vector<float64_t>, int64_t,
  int64_t> OSQPInterface::optimize()
{
  // Run the solver on the stored problem representation.
  std::tuple<std::vector<float64_t>, std::vector<float64_t>, int64_t, int64_t> result = solve();
  return result;
}

std::tuple<std::vector<float64_t>, std::vector<float64_t>, int64_t,
  int64_t> OSQPInterface::optimize(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<float64_t> & q,
  const std::vector<float64_t> & l, const std::vector<float64_t> & u)
{
  // Allocate memory for problem
  initializeProblem(P, A, q, l, u);

  // Run the solver on the stored problem representation.
  std::tuple<std::vector<float64_t>, std::vector<float64_t>, int64_t, int64_t> result = solve();

  return result;
}

}  // namespace osqp
}  // namespace common
}  // namespace autoware
