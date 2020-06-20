// Copyright 2017-2019 Apex.AI, Inc.
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

/// \file
/// \brief This file covers simple constants and misc functions

#ifndef HELPER_FUNCTIONS__HELPER_FUNCTIONS_HPP_
#define HELPER_FUNCTIONS__HELPER_FUNCTIONS_HPP_

#include <common/types.hpp>

#include <cmath>

namespace autoware
{
namespace common
{
/// \brief Common constants and functions that may be used
///        throughout the codebase.
namespace helper_functions
{

using autoware::common::types::float32_t;

/// \brief th_deg - phi_deg, normalized to +/- 180 deg
/// \param[in] th_deg the reference angle
/// \param[in] phi_deg the test angle
/// \return angle from reference angle to test angle
inline float32_t angle_distance_deg(const float32_t th_deg, const float32_t phi_deg)
{
  return fmodf((th_deg - phi_deg) + 540.0F, 360.0F) - 180.0F;
}

/// \brief converts a radian value to a degree value
/// \param[in] rad_val the radian value to convert
/// \return the radian value in degrees
inline float32_t rad2deg(const float32_t rad_val)
{
  return rad_val * 57.2958F;
}
}  // namespace helper_functions
}  // namespace common
}  // namespace autoware


#endif  // HELPER_FUNCTIONS__HELPER_FUNCTIONS_HPP_
