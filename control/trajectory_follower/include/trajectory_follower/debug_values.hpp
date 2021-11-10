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

#ifndef TRAJECTORY_FOLLOWER__DEBUG_VALUES_HPP_
#define TRAJECTORY_FOLLOWER__DEBUG_VALUES_HPP_

#include <array>

#include "common/types.hpp"
#include "trajectory_follower/visibility_control.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
using autoware::common::types::float64_t;
/// Debug Values used for debugging or controller tuning
class TRAJECTORY_FOLLOWER_PUBLIC DebugValues
{
public:
  /// Types of debug values
  enum class TYPE
  {
    DT = 0,
    CURRENT_VEL = 1,
    TARGET_VEL = 2,
    TARGET_ACC = 3,
    NEAREST_VEL = 4,
    NEAREST_ACC = 5,
    SHIFT = 6,
    PITCH_LPF_RAD = 7,
    PITCH_RAW_RAD = 8,
    PITCH_LPF_DEG = 9,
    PITCH_RAW_DEG = 10,
    ERROR_VEL = 11,
    ERROR_VEL_FILTERED = 12,
    CONTROL_STATE = 13,
    ACC_CMD_PID_APPLIED = 14,
    ACC_CMD_ACC_LIMITED = 15,
    ACC_CMD_JERK_LIMITED = 16,
    ACC_CMD_SLOPE_APPLIED = 17,
    ACC_CMD_PUBLISHED = 18,
    ACC_CMD_FB_P_CONTRIBUTION = 19,
    ACC_CMD_FB_I_CONTRIBUTION = 20,
    ACC_CMD_FB_D_CONTRIBUTION = 21,
    FLAG_STOPPING = 22,
    FLAG_EMERGENCY_STOP = 23,
    PREDICTED_VEL = 24,
    CALCULATED_ACC = 25,
    PITCH_RAW_TRAJ_RAD = 26,
    PITCH_RAW_TRAJ_DEG = 27,
    STOP_DIST = 28,
    SIZE  // this is the number of enum elements
  };

  /**
   * @brief get the index corresponding to the given value TYPE
   * @param [in] type the TYPE enum for which to get the index
   * @return index of the type
   */
  size_t getValuesIdx(const TYPE type) const {return static_cast<size_t>(type);}
  /**
   * @brief get all the debug values as an std::array
   * @return array of all debug values
   */
  std::array<float64_t, static_cast<size_t>(TYPE::SIZE)> getValues() const {return m_values;}
  /**
   * @brief set the given type to the given value
   * @param [in] type TYPE of the value
   * @param [in] value value to set
   */
  void setValues(const TYPE type, const float64_t value)
  {
    m_values.at(static_cast<size_t>(type)) = value;
  }
  /**
   * @brief set the given type to the given value
   * @param [in] type index of the type
   * @param [in] value value to set
   */
  void setValues(const size_t type, const float64_t value) {m_values.at(type) = value;}

private:
  std::array<float64_t, static_cast<size_t>(TYPE::SIZE)> m_values;
};
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER__DEBUG_VALUES_HPP_
