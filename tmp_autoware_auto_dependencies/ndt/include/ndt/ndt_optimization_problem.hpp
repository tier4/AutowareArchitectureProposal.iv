// Copyright 2019 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.


// This file contains modified code from the following open source projects
// published under the licenses listed below:
//
// Software License Agreement (BSD License)
//
//  Point Cloud Library (PCL) - www.pointclouds.org
//  Copyright (c) 2010-2011, Willow Garage, Inc.
//  Copyright (c) 2012-, Open Perception, Inc.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   * Neither the name of the copyright holder(s) nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

#ifndef NDT__NDT_OPTIMIZATION_PROBLEM_HPP_
#define NDT__NDT_OPTIMIZATION_PROBLEM_HPP_

#include <helper_functions/float_comparisons.hpp>
#include <ndt/ndt_map.hpp>
#include <ndt/ndt_scan.hpp>
#include <ndt/ndt_config.hpp>
#include <optimization/optimization_problem.hpp>
#include <optimization/utils.hpp>
#include <ndt/utils.hpp>
#include <ndt/constraints.hpp>
#include <experimental/optional>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <tuple>
#include "common/types.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;

namespace autoware
{
namespace localization
{
namespace ndt
{

template<typename ScalarT>
bool8_t is_valid_probability(ScalarT p)
{
  using common::helper_functions::comparisons::abs_lte;
  using common::helper_functions::comparisons::abs_gte;
  constexpr auto eps = std::numeric_limits<ScalarT>::epsilon();
  return std::isfinite(p) && abs_lte(p, 1.0, eps) && abs_gte(p, 0.0, eps);
}

/// P2D ndt objective. This class implements the P2D ndt score function, its analytical
/// jacobian and hessian values.
/// \tparam MapT Type of map to be used. This type should conform the interface specified in
/// `P2DNDTOptimizationMapConstraint`
template<typename MapT,
  Requires = traits::P2DNDTOptimizationMapConstraint<MapT>::value>
class P2DNDTObjective : public common::optimization::CachedExpression<P2DNDTObjective<MapT>,
    EigenPose<Real>, 1U, 6U, common::optimization::EigenComparator>
{
public:
  // getting aliases from the base class.
  using ExpressionT = common::optimization::CachedExpression<P2DNDTObjective<MapT>,
      EigenPose<Real>, 1U, 6U, common::optimization::EigenComparator>;
  using DomainValue = typename ExpressionT::DomainValue;
  using Value = typename ExpressionT::Value;
  using Jacobian = typename ExpressionT::Jacobian;
  using Hessian = typename ExpressionT::Hessian;
  using Map = MapT;
  using Scan = P2DNDTScan;
  using Point = Eigen::Vector3d;
  using Comparator = common::optimization::EigenComparator;
  using ComputeMode = common::optimization::ComputeMode;
  using PointGrad = Eigen::Matrix<float64_t, 3, 6>;
  using PointHessian = Eigen::Matrix<float64_t, 18, 6>;

  /// Constructor.
  ///
  /// It should be noted here that ndt optimization problem does not take ownership of neither the
  /// scan nor the map but uses the references. Hence an optimization problem must not outlive the
  /// scan or the map.
  ///
  /// @param      scan    Scan to align with the map.
  /// @param      map     NDT map to be aligned.
  /// @param      config  Optimization config for this objective.
  ///
  P2DNDTObjective(
    const P2DNDTScan & scan, const Map & map, const P2DNDTOptimizationConfig config)
  : m_scan_ref(scan), m_map_ref(map)
  {
    init(config.outlier_ratio());
  }

  void evaluate_(const DomainValue & x, const ComputeMode & mode)
  {
    // Convert pose vector to transform matrix for easy point transformation
    Eigen::Transform<float64_t, 3, Eigen::Affine, Eigen::ColMajor> transform;
    transform.setIdentity();
    transform_adapters::pose_to_transform(x, transform);

    Value score{0.0};

    Jacobian jacobian;
    std::experimental::optional<GradientAngleParameters> grad_params;

    Hessian hessian;
    std::experimental::optional<HessianAngleParameters> hessian_params;

    {
      // Angle parameters to be used by all elements (eq. 6.12) [Magnusson 2009]
      const AngleParameters angle_params{x};
      // Only construct jacobian/hessian variables if they are needed.
      if (mode.jacobian() || mode.hessian()) {
        jacobian.setZero();
        grad_params.emplace(angle_params);
      }
      if (mode.hessian()) {
        hessian.setZero();
        hessian_params.emplace(angle_params);
      }
    }

    for (const auto & pt : m_scan_ref) {
      PointGrad point_gradient;
      PointHessian point_hessian;

      if (mode.jacobian() || mode.hessian()) {
        point_gradient.setZero();
        point_gradient.block<3, 3>(0, 0).setIdentity();
        compute_point_gradients(grad_params.value(), pt, point_gradient);

        if (mode.hessian()) {
          point_hessian.setZero();
          compute_point_hessians(hessian_params.value(), pt, point_hessian);
        }
      }

      const Point pt_trans = transform * pt;
      const auto & cells = m_map_ref.cell(pt_trans);

      for (const auto & cell : cells) {
        const Point pt_trans_norm = pt_trans - cell.centroid();
        // Cell iteration used for compatibility with maps with multi-cell lookup
        if (!cell.usable()) {
          continue;
        }
        const auto & inv_cov = cell.inverse_covariance();
        // e^(-d_2/2 * (x_k - mu_k)^T Sigma_k^-1 (x_k - mu_k)) Equation 6.9 [Magnusson 2009]
        Real e_minus_half_d2_x_cov_x =
          std::exp(-m_gauss_d2 * pt_trans_norm.dot(inv_cov * pt_trans_norm) / 2.0);

        if (mode.score()) {
          score += -m_gauss_d1 * e_minus_half_d2_x_cov_x;
        }

        if (!mode.jacobian() && !mode.hessian()) {
          continue;
        }
        const auto d2_e_minus_half_d2_x_cov_x = m_gauss_d2 * e_minus_half_d2_x_cov_x;

        // Error checking for invalid values.
        if (!is_valid_probability(d2_e_minus_half_d2_x_cov_x)) {
          continue;
        }

        // Reusable portion of Equation 6.12 and 6.13 [Magnusson 2009]
        const auto d1_d2_e_minus_half_d2_x_cov_x = m_gauss_d1 * d2_e_minus_half_d2_x_cov_x;

        for (auto i = 0U; i < jacobian.rows(); ++i) {
          const Point cov_dxd_pi = inv_cov * point_gradient.col(i);
          if (mode.jacobian()) {
            jacobian(i) += pt_trans_norm.dot(cov_dxd_pi) * d1_d2_e_minus_half_d2_x_cov_x;
          }
          if (mode.hessian()) {
            for (auto j = 0U; j < hessian.cols(); ++j) {
              hessian(i, j) += d1_d2_e_minus_half_d2_x_cov_x *
                (-m_gauss_d2 * pt_trans_norm.dot(cov_dxd_pi) *
                pt_trans_norm.dot(inv_cov * point_gradient.col(j)) +
                pt_trans_norm.dot(inv_cov * point_hessian.block<3, 1>(3 * i, j)) +
                point_gradient.col(j).dot(cov_dxd_pi));
            }
          }
        }
      }
    }
    if (mode.score()) {
      this->set_score(score);
    }
    if (mode.jacobian()) {
      this->set_jacobian(jacobian);
    }
    if (mode.hessian()) {
      this->set_hessian(hessian);
    }
  }

private:
  /// Struct encapculating the intermediate parameters used in (eq. 6.17) [Magnusson 2009]
  struct AngleParameters
  {
public:
    static constexpr auto approx_thresh{10e-5};
    AngleParameters() = delete;
    explicit AngleParameters(const DomainValue & pose)
    {
      if (std::fabs(pose(3)) < approx_thresh) {
        cx = 1.0;
        sx = 0.0;
      } else {
        cx = std::cos(pose(3));
        sx = std::sin(pose(3));
      }

      if (std::fabs(pose(4)) < approx_thresh) {
        cy = 1.0;
        sy = 0.0;
      } else {
        cy = std::cos(pose(4));
        sy = std::sin(pose(4));
      }

      if (std::fabs(pose(5)) < approx_thresh) {
        cz = 1.0;
        sz = 0.0;
      } else {
        cz = std::cos(pose(5));
        sz = std::sin(pose(5));
      }
    }

    Real cx{0.0};
    Real cy{0.0};
    Real cz{0.0};
    Real sx{1.0};
    Real sy{1.0};
    Real sz{1.0};
  };

  /// Struct encapculating the intermediate parameters used in (eq. 6.19) [Magnusson 2009]
  struct GradientAngleParameters
  {
public:
    GradientAngleParameters() = delete;
    explicit GradientAngleParameters(const AngleParameters & params)
    {
      j_ang_a(0) = -params.sx * params.sz + params.cx * params.sy * params.cz;
      j_ang_a(1) = -params.sx * params.cz - params.cx * params.sy * params.sz;
      j_ang_a(2) = -params.cx * params.cy;

      j_ang_b(0) = params.cx * params.sz + params.sx * params.sy * params.cz;
      j_ang_b(1) = params.cx * params.cz - params.sx * params.sy * params.sz;
      j_ang_b(2) = -params.sx * params.cy;

      j_ang_c(0) = -params.sy * params.cz;
      j_ang_c(1) = params.sy * params.sz;
      j_ang_c(2) = params.cy;

      j_ang_d(0) = params.sx * params.cy * params.cz;
      j_ang_d(1) = -params.sx * params.cy * params.sz;
      j_ang_d(2) = params.sx * params.sy;

      j_ang_e(0) = -params.cx * params.cy * params.cz;
      j_ang_e(1) = params.cx * params.cy * params.sz;
      j_ang_e(2) = -params.cx * params.sy;

      j_ang_f(0) = -params.cy * params.sz;
      j_ang_f(1) = -params.cy * params.cz;
      j_ang_f(2) = 0.0;

      j_ang_g(0) = params.cx * params.cz - params.sx * params.sy * params.sz;
      j_ang_g(1) = -params.cx * params.sz - params.sx * params.sy * params.cz;
      j_ang_g(2) = 0.0;

      j_ang_h(0) = params.sx * params.cz + params.cx * params.sy * params.sz;
      j_ang_h(1) = params.cx * params.sy * params.cz - params.sx * params.sz;
      j_ang_h(2) = 0.0;
    }

    Point j_ang_a, j_ang_b, j_ang_c, j_ang_d, j_ang_e, j_ang_f, j_ang_g, j_ang_h;
  };

  /// Struct encapculating the intermediate parameters used in (eq. 6.21) [Magnusson 2009]
  struct HessianAngleParameters
  {
public:
    HessianAngleParameters() = delete;
    explicit HessianAngleParameters(const AngleParameters & params)
    {
      h_ang_a2(0) = -params.cx * params.sz - params.sx * params.sy * params.cz;
      h_ang_a2(1) = -params.cx * params.cz + params.sx * params.sy * params.sz;
      h_ang_a2(2) = params.sx * params.cy;

      h_ang_a3(0) = -params.sx * params.sz + params.cx * params.sy * params.cz;
      h_ang_a3(1) = -params.cx * params.sy * params.sz - params.sx * params.cz;
      h_ang_a3(2) = -params.cx * params.cy;

      h_ang_b2(0) = params.cx * params.cy * params.cz;
      h_ang_b2(1) = -params.cx * params.cy * params.sz;
      h_ang_b2(2) = params.cx * params.sy;

      h_ang_b3(0) = params.sx * params.cy * params.cz;
      h_ang_b3(1) = -params.sx * params.cy * params.sz;
      h_ang_b3(2) = params.sx * params.sy;

      h_ang_c2(0) = -params.sx * params.cz - params.cx * params.sy * params.sz;
      h_ang_c2(1) = params.sx * params.sz - params.cx * params.sy * params.cz;
      h_ang_c2(2) = 0.0;

      h_ang_c3(0) = params.cx * params.cz - params.sx * params.sy * params.sz;
      h_ang_c3(1) = -params.sx * params.sy * params.cz - params.cx * params.sz;
      h_ang_c3(2) = 0.0;

      h_ang_d1(0) = -params.cy * params.cz;
      h_ang_d1(1) = params.cy * params.sz;
      h_ang_d1(2) = params.sy;

      h_ang_d2(0) = -params.sx * params.sy * params.cz;
      h_ang_d2(1) = params.sx * params.sy * params.sz;
      h_ang_d2(2) = params.sx * params.cy;

      h_ang_d3(0) = params.cx * params.sy * params.cz;
      h_ang_d3(1) = -params.cx * params.sy * params.sz;
      h_ang_d3(2) = -params.cx * params.cy;

      h_ang_e1(0) = params.sy * params.sz;
      h_ang_e1(1) = params.sy * params.cz;
      h_ang_e1(2) = 0.0;

      h_ang_e2(0) = -params.sx * params.cy * params.sz;
      h_ang_e2(1) = -params.sx * params.cy * params.cz;
      h_ang_e2(2) = 0.0;

      h_ang_e3(0) = params.cx * params.cy * params.sz;
      h_ang_e3(1) = params.cx * params.cy * params.cz;
      h_ang_e3(2) = 0.0;

      h_ang_f1(0) = -params.cy * params.cz;
      h_ang_f1(1) = params.cy * params.sz;
      h_ang_f1(2) = 0.0;

      h_ang_f2(0) = -params.cx * params.sz - params.sx * params.sy * params.cz;
      h_ang_f2(1) = -params.cx * params.cz + params.sx * params.sy * params.sz;
      h_ang_f2(2) = 0.0;

      h_ang_f3(0) = -params.sx * params.sz + params.cx * params.sy * params.cz;
      h_ang_f3(1) = -params.cx * params.sy * params.sz - params.sx * params.cz;
      h_ang_f3(2) = 0.0;
    }

    Point h_ang_a2, h_ang_a3,
      h_ang_b2, h_ang_b3,
      h_ang_c2, h_ang_c3,
      h_ang_d1, h_ang_d2, h_ang_d3,
      h_ang_e1, h_ang_e2, h_ang_e3,
      h_ang_f1, h_ang_f2, h_ang_f3;
  };

  void compute_point_gradients(
    const GradientAngleParameters & params,
    const Point & x,
    PointGrad & point_gradient)
  {
    point_gradient(1, 3) = x.dot(params.j_ang_a);
    point_gradient(2, 3) = x.dot(params.j_ang_b);
    point_gradient(0, 4) = x.dot(params.j_ang_c);
    point_gradient(1, 4) = x.dot(params.j_ang_d);
    point_gradient(2, 4) = x.dot(params.j_ang_e);
    point_gradient(0, 5) = x.dot(params.j_ang_f);
    point_gradient(1, 5) = x.dot(params.j_ang_g);
    point_gradient(2, 5) = x.dot(params.j_ang_h);
  }

  void compute_point_hessians(
    const HessianAngleParameters & params,
    const Point & x,
    PointHessian & point_hessian)
  {
    const Point a{0.0, x.dot(params.h_ang_a2), x.dot(params.h_ang_a3)};
    const Point b{0.0, x.dot(params.h_ang_b2), x.dot(params.h_ang_b3)};
    const Point c{0.0, x.dot(params.h_ang_c2), x.dot(params.h_ang_c3)};
    const Point d{x.dot(params.h_ang_d1), x.dot(params.h_ang_d2),
      x.dot(params.h_ang_d3)};
    const Point e{x.dot(params.h_ang_e1), x.dot(params.h_ang_e2),
      x.dot(params.h_ang_e3)};
    const Point f{x.dot(params.h_ang_f1), x.dot(params.h_ang_f2),
      x.dot(params.h_ang_f3)};

    point_hessian.block<3, 1>(9, 3) = a;
    point_hessian.block<3, 1>(12, 3) = b;
    point_hessian.block<3, 1>(15, 3) = c;
    point_hessian.block<3, 1>(9, 4) = b;
    point_hessian.block<3, 1>(12, 4) = d;
    point_hessian.block<3, 1>(15, 4) = e;
    point_hessian.block<3, 1>(9, 5) = c;
    point_hessian.block<3, 1>(12, 5) = e;
    point_hessian.block<3, 1>(15, 5) = f;
  }

  /// Initializes the guassian fitting parameters (eq. 6.8) [Magnusson 2009]
  /// \param outlier_ratio Outlier ratio to be used in the gaussian distribution variation
  /// used in (eq. 6.7) [Magnusson 2009]
  void init(Real outlier_ratio)
  {
    if (!is_valid_probability(outlier_ratio)) {
      throw std::domain_error("Outlier ratio must be between 0 and 1");
    }
    const auto c_size = m_map_ref.cell_size();
    // The gaussian fitting parameters below are taken from the PCL implementation.
    // 10.0 seems to be a magic number. For details on the gaussian
    // approximation of the mixture probability in see [Biber et al, 2004] and [Magnusson 2009].
    const auto gauss_c1 = 10.0 * (1.0 - outlier_ratio);
    const auto gauss_c2 = outlier_ratio / static_cast<Real>(c_size.x * c_size.y * c_size.z);
    const auto gauss_d3 = -std::log(gauss_c2);
    m_gauss_d1 = -std::log(gauss_c1 + gauss_c2) - gauss_d3;
    m_gauss_d2 = -2 *
      std::log((-std::log(gauss_c1 * std::exp(-0.5) + gauss_c2) - gauss_d3) / m_gauss_d1);
  }

  // references as class members to be initialized at constructor.
  const Scan & m_scan_ref;
  const Map & m_map_ref;
  // States:
  Real m_gauss_d1{0.0};
  Real m_gauss_d2{0.0};
};

template<typename MapT>
using P2DNDTOptimizationProblem =
  common::optimization::UnconstrainedOptimizationProblem<P2DNDTObjective<MapT>, EigenPose<Real>,
    6U>;
}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // NDT__NDT_OPTIMIZATION_PROBLEM_HPP_
