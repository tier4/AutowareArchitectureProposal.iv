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

#ifndef NDT__NDT_LOCALIZER_HPP_
#define NDT__NDT_LOCALIZER_HPP_

#include <helper_functions/template_utils.hpp>
#include <localization_common/optimized_registration_summary.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <ndt/ndt_common.hpp>
#include <ndt/ndt_optimization_problem.hpp>
#include <ndt/constraints.hpp>
#include <optimization/optimizer_options.hpp>
#include <experimental/optional>
#include <utility>
#include <string>

namespace autoware
{
namespace localization
{
namespace ndt
{
using CloudT = sensor_msgs::msg::PointCloud2;

/// Base class for NDT based localizers. Implementations must implement the validation logic.
/// \tparam ScanT Type of ndt scan.
/// \tparam NDTOptimizationProblemT Type of ndt optimization problem.
/// \tparam OptimizerT Type of optimizer.
/// \tparam ConfigT Type of localization configuration.
template<
  typename ScanT,
  typename NDTOptimizationProblemT,
  typename OptimizationProblemConfigT,
  typename OptimizerT>
class NDT_PUBLIC NDTLocalizerBase
{
public:
  using Transform = geometry_msgs::msg::TransformStamped;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Summary = localization_common::OptimizedRegistrationSummary;

  /// Constructor
  ///
  /// @param      config                       NDT localizer config
  /// @param      optimization_problem_config  The optimization problem config
  /// @param      optimizer                    Optimizer to use during optimization.
  /// @param      scan                         Initial value of the ndt scan. This element is
  ///                                          expected to be constructed in the implementation
  ///                                          class and moved to the base class.
  ///
  explicit NDTLocalizerBase(
    const NDTLocalizerConfigBase & config,
    const OptimizationProblemConfigT & optimization_problem_config,
    const OptimizerT & optimizer,
    ScanT && scan)
  : m_config{config},
    m_optimization_problem_config{optimization_problem_config},
    m_optimizer{optimizer},
    m_scan{std::forward<ScanT>(scan)} {}

  /// Register a measurement to the current map and return the transformation from map to the
  /// measurement.
  /// \tparam MapT Map type to be used. This map should conform the interface specied in
  /// `LocalizationMapConstraint`
  /// \param[in] msg Measurement message to register.
  /// \param[in] transform_initial Initial guess of the pose to initialize the localizer with
  /// in iterative processes like solving optimization problems.
  /// \param[in] map Map to register.
  /// \param[out] summary (Optional) Reference to the registration summary.
  /// \return Pose estimate after registration.
  /// \throws std::logic_error on measurements older than the map.
  /// \throws std::domain_error on pose estimates that are not within the configured duration
  /// range from the measurement.
  /// \throws std::runtime_error on numerical errors in the optimizer.
  template<typename MapT,
    Requires = traits::LocalizationMapConstraint<MapT>::value>
  PoseWithCovarianceStamped register_measurement(
    const CloudT & msg,
    const Transform & transform_initial,
    const MapT & map,
    Summary * const summary = nullptr)
  {
    PoseWithCovarianceStamped pose_out{};
    validate_msg(msg, map);
    validate_guess(msg, transform_initial);
    // Initial checks passed, proceed with initialization
    // Eigen representations to be used for internal computations.
    EigenPose<Real> eig_pose_initial, eig_pose_result;
    eig_pose_initial.setZero();
    eig_pose_result.setZero();
    // Convert the ros transform/pose to eigen pose vector
    transform_adapters::transform_to_pose(transform_initial.transform, eig_pose_initial);

    // Set the scan
    m_scan.clear();
    m_scan.insert(msg);

    // Define and solve the problem.
    NDTOptimizationProblemT problem(m_scan, map, m_optimization_problem_config);
    const auto opt_summary = m_optimizer.solve(problem, eig_pose_initial, eig_pose_result);

    if (opt_summary.termination_type() == common::optimization::TerminationType::FAILURE) {
      throw std::runtime_error(
              "NDT localizer has likely encountered a numerical "
              "error during optimization.");
    }

    // Convert eigen pose back to ros pose/transform
    transform_adapters::pose_to_transform(
      eig_pose_result,
      pose_out.pose.pose);

    pose_out.header.stamp = msg.header.stamp;
    pose_out.header.frame_id = map.frame_id();

    // Populate covariance information. It is implementation defined.
    set_covariance(problem, eig_pose_initial, eig_pose_result, pose_out);
    if (summary != nullptr) {
      *summary = localization_common::OptimizedRegistrationSummary{opt_summary};
    }
    pose_out.pose.covariance[0] = 0.025;
    pose_out.pose.covariance[7] = 0.025;
    pose_out.pose.covariance[14] = 0.025;
    pose_out.pose.covariance[21] = 0.000625;
    pose_out.pose.covariance[28] = 0.000625;
    pose_out.pose.covariance[35] = 0.000625;

    return pose_out;
  }


  /// Get the last used scan.
  const ScanT & scan() const noexcept
  {
    return m_scan;
  }
  /// Get the optimizer.
  const OptimizerT & optimizer() const noexcept
  {
    return m_optimizer;
  }
  /// Get the localizer configuration.
  const NDTLocalizerConfigBase & config() const noexcept
  {
    return m_config;
  }
  /// Get the optimization problem configuration.
  const OptimizationProblemConfigT & optimization_problem_config() const noexcept
  {
    return m_optimization_problem_config;
  }

protected:
  /// Populate the covariance information of an ndt estimate using the information using existing
  /// information regarding scan, map and the optimization problem.
  /// \param[in] problem Optimization problem.
  /// \param[in] initial_guess Initial transformation guess as a pose.
  /// \param[in] pose_result Estimated transformation as a pose.
  /// \param[out] solution Estimated transform message.
  virtual void set_covariance(
    const NDTOptimizationProblemT & problem,
    const EigenPose<Real> & initial_guess,
    const EigenPose<Real> & pose_result,
    PoseWithCovarianceStamped & solution) const
  {
    (void) problem;
    (void) initial_guess;
    (void) pose_result;
    (void) solution;
    // For now, do nothing.
  }

  /// Check if the received message is valid to be registered. Following checks are made:
  /// * Message timestamp is not older than the map timestamp.
  /// \param msg Message to register.
  /// \param map The map to be registered on.
  /// \throws std::logic_error on old data.
  template<typename MapT>
  void validate_msg(const CloudT & msg, const MapT & map) const
  {
    const auto message_time = ::time_utils::from_message(msg.header.stamp);
    // Map shouldn't be newer than a measurement.
    if (message_time < map.stamp()) {
      throw std::logic_error(
              "Lidar scan should not have a timestamp older than the timestamp of"
              "the current map.");
    }
  }

  /// Check if the initial guess is valid. Following checks are made:
  /// * pose guess timestamp is within a tolerated range from the scan timestamp.
  /// \param msg Message to register
  /// \param transform_initial Initial pose estimate
  /// \throws std::domain_error on untimely initial pose.
  virtual void validate_guess(const CloudT & msg, const Transform & transform_initial) const
  {
    const auto message_time = ::time_utils::from_message(msg.header.stamp);

    const auto guess_scan_diff = ::time_utils::from_message(transform_initial.header.stamp) -
      message_time;
    const auto stamp_tol = m_config.guess_time_tolerance();
    // An initial estimate should be comparable in time to the measurement's time stamp
    if (std::abs(
        std::chrono::duration_cast<std::decay_t<decltype(stamp_tol)>>(guess_scan_diff).
        count()) >
      std::abs(stamp_tol.count()))
    {
      throw std::domain_error(
              "Initial guess is not within: " + std::to_string(stamp_tol.count()) +
              "ns range of the scan's time stamp. Either increase the tolerance range or"
              "make sure the localizer takes in timely initial pose guesses.");
    }
  }

private:
  NDTLocalizerConfigBase m_config;
  OptimizationProblemConfigT m_optimization_problem_config;
  OptimizerT m_optimizer;
  ScanT m_scan;
};

/// P2D localizer implementation.
/// This implementation specifies a covariance computation method for the P2D optimization problem.
/// \tparam OptimizerT Optimizer type.
/// \tparam OptimizerOptionsT Optimizer options type.
/// \tparam MapT Type of map to be used. By default it is StaticNDTMap.
template<typename OptimizerT, typename MapT = StaticNDTMap>
class NDT_PUBLIC P2DNDTLocalizer : public NDTLocalizerBase<
    P2DNDTScan, P2DNDTOptimizationProblem<MapT>, P2DNDTOptimizationConfig, OptimizerT>
{
public:
  using CloudT = sensor_msgs::msg::PointCloud2;
  using ParentT = NDTLocalizerBase<
    P2DNDTScan, P2DNDTOptimizationProblem<MapT>, P2DNDTOptimizationConfig, OptimizerT>;
  using Transform = typename ParentT::Transform;
  using PoseWithCovarianceStamped = typename ParentT::PoseWithCovarianceStamped;
  using ScanT = P2DNDTScan;

  explicit P2DNDTLocalizer(
    const P2DNDTLocalizerConfig & config,
    const OptimizerT & optimizer,
    const Real outlier_ratio)
  : ParentT{
      config,
      P2DNDTOptimizationConfig{outlier_ratio},
      optimizer,
      ScanT{config.scan_capacity()}} {}

protected:
  void set_covariance(
    const P2DNDTOptimizationProblem<MapT> &,
    const EigenPose<Real> &,
    const EigenPose<Real> &,
    PoseWithCovarianceStamped &) const override
  {
    // For now, do nothing.
  }
};

}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // NDT__NDT_LOCALIZER_HPP_
