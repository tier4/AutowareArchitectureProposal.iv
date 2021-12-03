// Copyright 2020 the Autoware Foundation
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

#ifndef NDT__NDT_CONFIG_HPP_
#define NDT__NDT_CONFIG_HPP_

#include <ndt/ndt_common.hpp>
#include <voxel_grid/config.hpp>
#include <utility>

namespace autoware
{
namespace localization
{
namespace ndt
{
/// Base config class for all ndt localizers
/// \tparam OptimizationConfigT Configuration class type for the used optimization problem.
class NDTLocalizerConfigBase
{
public:
  /// Constructor
  /// \param guess_time_tolerance
  NDTLocalizerConfigBase(
    std::chrono::nanoseconds guess_time_tolerance)
  : m_guess_time_tol{guess_time_tolerance} {}

  /// Get optimizer config.
  /// \return optimizer config
  const std::chrono::nanoseconds & guess_time_tolerance() const noexcept
  {
    return m_guess_time_tol;
  }

private:
  std::chrono::nanoseconds m_guess_time_tol;
};


/// Config class for p2d optimziation problem
class NDT_PUBLIC P2DNDTOptimizationConfig
{
public:
  /// Constructor
  /// \param outlier_ratio Outlier ratio to be used in the gaussian distribution variation used
  /// in (eq. 6.7) [Magnusson 2009]
  explicit P2DNDTOptimizationConfig(Real outlier_ratio)
  : m_outlier_ratio{outlier_ratio} {}

  /// Get outlier ratio.
  /// \return outlier ratio.
  Real outlier_ratio() const noexcept {return m_outlier_ratio;}

private:
  Real m_outlier_ratio;
};


/// config class for p2d ndt localizer
class NDT_PUBLIC P2DNDTLocalizerConfig : public NDTLocalizerConfigBase
{
public:
  /// Constructor
  /// \param scan_capacity Capacity of the ndt scan. This corresponds to the maximum number of
  /// points expected in a single lidar scan.
  /// \param guess_time_tolerance Time difference tolerance between the initial guess timestamp
  /// and the timestamp of the scan.
  P2DNDTLocalizerConfig(
    const uint32_t scan_capacity,
    std::chrono::nanoseconds guess_time_tolerance)
  : NDTLocalizerConfigBase{guess_time_tolerance},
    m_scan_capacity(scan_capacity) {}

  /// Get scan capacity.
  /// \return scan capacity.
  uint32_t scan_capacity() const noexcept
  {
    return m_scan_capacity;
  }

private:
  uint32_t m_scan_capacity;
};

}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // NDT__NDT_CONFIG_HPP_
