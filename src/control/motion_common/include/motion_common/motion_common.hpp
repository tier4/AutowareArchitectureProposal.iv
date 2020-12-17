// Copyright 2019 Christopher Ho
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
#ifndef MOTION_COMMON__MOTION_COMMON_HPP_
#define MOTION_COMMON__MOTION_COMMON_HPP_

#include <motion_common/visibility_control.hpp>
#include <autoware_auto_msgs/msg/control_diagnostic.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <time_utils/time_utils.hpp>

#include <algorithm>
#include <cmath>

namespace motion
{
namespace motion_common
{
// Use same representation as message type
using Real = decltype(autoware_auto_msgs::msg::TrajectoryPoint::x);
using Command = autoware_auto_msgs::msg::VehicleControlCommand;
using Diagnostic = autoware_auto_msgs::msg::ControlDiagnostic;
using State = autoware_auto_msgs::msg::VehicleKinematicState;
using Trajectory = autoware_auto_msgs::msg::Trajectory;
using Heading = decltype(decltype(State::state)::heading);
using Index = decltype(Trajectory::points)::size_type;
using Point = decltype(Trajectory::points)::value_type;

/// Check if a state is past a given trajectory point, assuming heading is correct
MOTION_COMMON_PUBLIC bool is_past_point(const Point & state, const Point & pt) noexcept;
/// Check if a state is past a given trajectory point given the last (aka current)
/// trajectory point
MOTION_COMMON_PUBLIC bool is_past_point(
  const Point & state, const Point & current_pt, const Point & next_pt) noexcept;
/// Given a normal, determine if state is past a point
MOTION_COMMON_PUBLIC
bool is_past_point(const Point & state, const Point & pt, Real nx, Real ny) noexcept;

/// Check if cosine angle is less than some dot product threshold
MOTION_COMMON_PUBLIC
bool is_aligned(Heading a, Heading b, Real dot_threshold);

/// Advance to the first trajectory point past state according to criterion is_past_point
template<typename IsPastPointF>
Index update_reference_index(
  const Trajectory & traj,
  const State & state,
  Index start_idx,
  IsPastPointF is_past_point)
{
  // Invariant: m_reference_trajectory.points.size > 0U
  if (traj.points.empty()) {
    return 0U;
  }
  auto next_idx = start_idx;
  for (auto idx = start_idx; idx < traj.points.size() - 1U; ++idx) {
    const auto current_pt = traj.points[idx];
    const auto next_pt = traj.points[idx + 1U];

    // If I'm not past the next point, then I'm done
    if (!is_past_point(state, current_pt, next_pt, traj)) {
      break;
    }
    // Otherwise, update
    next_idx = idx + 1U;
  }
  return next_idx;
}

/// Check that all heading values in a trajectory are normalized 2D quaternions
MOTION_COMMON_PUBLIC bool heading_ok(const Trajectory & traj);

////////////////////////////////////////////////////////////////////////////////
// Operations relating to algebraic manipulation of VehicleKinematicState and TrajectoryPoint

/// Apply transform to TrajectoryPoint
MOTION_COMMON_PUBLIC void doTransform(
  const Point & t_in,
  Point & t_out,
  const geometry_msgs::msg::TransformStamped & transform) noexcept;
/// Apply transform to VehicleKinematicState
MOTION_COMMON_PUBLIC void doTransform(
  const State & t_in,
  State & t_out,
  const geometry_msgs::msg::TransformStamped & transform) noexcept;

/// Converts 2D quaternion to simple heading representation
MOTION_COMMON_PUBLIC Real to_angle(Heading heading) noexcept;

/// Basic conversion
template<typename RealT>
Heading from_angle(RealT angle) noexcept
{
  static_assert(std::is_floating_point<RealT>::value, "angle must be floating point");
  Heading ret{};
  ret.real = std::cos(angle * RealT{0.5});
  ret.imag = std::sin(angle * RealT{0.5});
  return ret;
}

/// Standard clamp implementation
template<typename T>
T clamp(T val, T min, T max)
{
  if (max < min) {
    throw std::domain_error{"max < min"};
  }
  return std::min(max, std::max(min, val));
}

/// Standard linear interpolation
template<typename T, typename RealT = double>
T interpolate(T a, T b, RealT t)
{
  static_assert(std::is_floating_point<RealT>::value, "t must be floating point");
  t = clamp(t, RealT{0.0}, RealT{1.0});
  const auto del = b - a;
  return static_cast<T>(t * static_cast<RealT>(del)) + a;
}

/// 2D nlerp (linear approximation of slerp):
/// http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
MOTION_COMMON_PUBLIC Heading nlerp(Heading a, Heading b, Real t);
// TODO(c.ho) proper slerp implementation

/// Trajectory point interpolation
template<typename SlerpF>
Point interpolate(Point a, Point b, Real t, SlerpF slerp_fn)
{
#ifdef ROS_DISTRO_DASHING
  Point ret{rosidl_generator_cpp::MessageInitialization::ALL};
#elif defined ROS_DISTRO_FOXY
  Point ret{rosidl_runtime_cpp::MessageInitialization::ALL};
#endif
  {
    const auto dt0 = time_utils::from_message(a.time_from_start);
    const auto dt1 = time_utils::from_message(b.time_from_start);
    ret.time_from_start = time_utils::to_message(time_utils::interpolate(dt0, dt1, t));
  }
  ret.x = interpolate(a.x, b.x, t);
  ret.y = interpolate(a.y, b.y, t);
  ret.heading = slerp_fn(a.heading, b.heading, t);
  ret.longitudinal_velocity_mps =
    interpolate(a.longitudinal_velocity_mps, b.longitudinal_velocity_mps, t);
  ret.lateral_velocity_mps = interpolate(a.lateral_velocity_mps, b.lateral_velocity_mps, t);
  ret.acceleration_mps2 = interpolate(a.acceleration_mps2, b.acceleration_mps2, t);
  ret.heading_rate_rps = interpolate(a.heading_rate_rps, b.heading_rate_rps, t);
  ret.front_wheel_angle_rad = interpolate(a.front_wheel_angle_rad, b.front_wheel_angle_rad, t);
  ret.rear_wheel_angle_rad = interpolate(a.rear_wheel_angle_rad, b.rear_wheel_angle_rad, t);

  return ret;
}

/// Default point interpolation, currently uses nlerp
MOTION_COMMON_PUBLIC Point interpolate(Point a, Point b, Real t);

/// Sample a trajectory using interpolation; does not extrapolate
template<typename SlerpF>
void sample(
  const Trajectory & in,
  Trajectory & out,
  std::chrono::nanoseconds period,
  SlerpF slerp_fn)
{
  out.points.clear();
  out.header = in.header;
  if (in.points.empty()) {
    return;
  }
  // Determine number of points
  using time_utils::from_message;
  const auto last_time = from_message(in.points.back().time_from_start);
  auto count_ = last_time / period;
  // should round down
  using SizeT = typename decltype(in.points)::size_type;
  const auto count =
    static_cast<SizeT>(std::min(count_, static_cast<decltype(count_)>(in.CAPACITY)));
  out.points.reserve(count);
  // Insert first point
  out.points.push_back(in.points.front());
  // Add remaining points
  auto ref_duration = period;
  auto after_current_ref_idx = SizeT{1};
  for (auto idx = SizeT{1}; idx < count; ++idx) {
    // Determine next reference index
    for (auto jdx = after_current_ref_idx; jdx < in.points.size(); ++jdx) {
      const auto & pt = in.points[jdx];
      if (from_message(pt.time_from_start) > ref_duration) {
        after_current_ref_idx = jdx;
        break;
      }
    }
    // Interpolate
    {
      const auto & curr_pt = in.points[after_current_ref_idx - 1U];
      const auto & next_pt = in.points[after_current_ref_idx];
      const auto dt = std::chrono::duration_cast<std::chrono::duration<Real>>(
        from_message(next_pt.time_from_start) - from_message(curr_pt.time_from_start));
      const auto dt_ = std::chrono::duration_cast<std::chrono::duration<Real>>(
        ref_duration - from_message(curr_pt.time_from_start));
      const auto t = dt_.count() / dt.count();
      out.points.push_back(interpolate(curr_pt, next_pt, t, slerp_fn));
    }
    ref_duration += period;
  }
}

/// Trajectory sampling with default interpolation (of nlerp)
MOTION_COMMON_PUBLIC void sample(
  const Trajectory & in,
  Trajectory & out,
  std::chrono::nanoseconds period);

/// Diagnostic header stuff
MOTION_COMMON_PUBLIC
void error(const Point & state, const Point & ref, Diagnostic & out) noexcept;
}  // namespace motion_common
}  // namespace motion

namespace autoware_auto_msgs
{
namespace msg
{
// TODO(c.ho) these should go into some autoware_auto_msgs package
/// Addition operator
MOTION_COMMON_PUBLIC Complex32 operator+(Complex32 a, Complex32 b) noexcept;
/// Unary minus
MOTION_COMMON_PUBLIC Complex32 operator-(Complex32 a) noexcept;
/// Difference operator
MOTION_COMMON_PUBLIC Complex32 operator-(Complex32 a, Complex32 b) noexcept;
}  // namespace msg
}  // namespace autoware_auto_msgs
#endif  // MOTION_COMMON__MOTION_COMMON_HPP_
