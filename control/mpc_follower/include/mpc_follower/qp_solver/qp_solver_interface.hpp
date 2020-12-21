// Copyright 2018-2019 Autoware Foundation
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

/**
 * @file qp_solver_interface.h
 * @brief qp solver interface class
 * @author Takamasa Horibe
 * @date 2019.05.01
 */

#ifndef CONTROL_MPC_FOLLOWER_INCLUDE_MPC_FOLLOWER_QP_SOLVER_QP_SOLVER_INTERFACE_H
#define CONTROL_MPC_FOLLOWER_INCLUDE_MPC_FOLLOWER_QP_SOLVER_QP_SOLVER_INTERFACE_H

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/LU"

class QPSolverInterface
{
public:
  /**
   * @brief destructor
   */
  virtual ~QPSolverInterface() = default;

  /**
   * @brief solve QP problem : minimize J = U' * Hmat * U + fvec' * U without constraint
   * @param [in] Hmat parameter matrix in object function
   * @param [in] fvec parameter matrix in object function
   * @param [in] A parameter matrix for constraint lbA < A*U < ubA
   * @param [in] lb parameter matrix for constraint lb < U < ub
   * @param [in] up parameter matrix for constraint lb < U < ub
   * @param [in] lbA parameter matrix for constraint lbA < A*U < ubA
   * @param [in] ubA parameter matrix for constraint lbA < A*U < ubA
   * @param [out] U optimal variable vector
   * @return bool to check the problem is solved
   */
  virtual bool solve(
    const Eigen::MatrixXd & Hmat, const Eigen::MatrixXd & fvec, const Eigen::MatrixXd & A,
    const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const Eigen::VectorXd & lbA,
    const Eigen::VectorXd & ubA, Eigen::VectorXd & U) = 0;
};
#endif
