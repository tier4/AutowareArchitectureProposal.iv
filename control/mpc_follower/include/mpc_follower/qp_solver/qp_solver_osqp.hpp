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
 * @file qp_solver_osqp.h
 * @brief qp solver with Eigen
 * @author Takamasa Horibe
 * @date 2020.02
 */

#ifndef MPC_FOLLOWER__QP_SOLVER__QP_SOLVER_OSQP_HPP_
#define MPC_FOLLOWER__QP_SOLVER__QP_SOLVER_OSQP_HPP_

#include "mpc_follower/qp_solver/qp_solver_interface.hpp"

#include <eigen3/Eigen/Dense>
#include <osqp_interface/osqp_interface.hpp>
#include <rclcpp/rclcpp.hpp>

class QPSolverOSQP : public QPSolverInterface
{
public:
  /**
   * @brief constructor
   */
  explicit QPSolverOSQP(const rclcpp::Logger & logger);

  /**
   * @brief destructor
   */
  virtual ~QPSolverOSQP() = default;

  /**
   * @brief solve QP problem : minimize J = U' * Hmat * U + fvec' * U without constraint
   * @param [in] Hmat parameter matrix in object function
   * @param [in] fvec parameter matrix in object function
   * @param [in] A parameter matrix for constraint lbA < A*U < ubA (not used here)
   * @param [in] lb parameter matrix for constraint lb < U < ub (not used here)
   * @param [in] up parameter matrix for constraint lb < U < ub (not used here)
   * @param [in] lbA parameter matrix for constraint lbA < A*U < ubA (not used here)
   * @param [in] ubA parameter matrix for constraint lbA < A*U < ubA (not used here)
   * @param [out] U optimal variable vector
   * @return bool to check the problem is solved
   */
  bool solve(
    const Eigen::MatrixXd & Hmat, const Eigen::MatrixXd & fvec, const Eigen::MatrixXd & A,
    const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const Eigen::VectorXd & lbA,
    const Eigen::VectorXd & ubA, Eigen::VectorXd & U) override;

private:
  osqp::OSQPInterface osqpsolver_;
  rclcpp::Logger logger_;
};
#endif  // MPC_FOLLOWER__QP_SOLVER__QP_SOLVER_OSQP_HPP_
