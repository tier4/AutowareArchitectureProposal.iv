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
#include "motion_testing/motion_testing.hpp"

#include <time_utils/time_utils.hpp>

#include <limits>

namespace motion
{
namespace motion_testing
{
State make_state(
  float x0,
  float y0,
  float heading,
  float v0,
  float a0,
  float turn_rate,
  std::chrono::system_clock::time_point t)
{
  State start_state{rosidl_runtime_cpp::MessageInitialization::ALL};
  start_state.state.x = x0;
  start_state.state.y = y0;
  start_state.state.heading.real = std::cos(heading / 2.0F);
  start_state.state.heading.imag = std::sin(heading / 2.0F);
  start_state.state.longitudinal_velocity_mps = v0;
  start_state.state.acceleration_mps2 = a0;
  start_state.state.heading_rate_rps = turn_rate;
  start_state.state.lateral_velocity_mps = 0.0F;

  start_state.header.stamp = time_utils::to_message(t);

  return start_state;
}

////////////////////////////////////////////////////////////////////////////////
State generate_state(Generator & gen)
{
  State ret{rosidl_runtime_cpp::MessageInitialization::ALL};
  // Parameters with positive and negative supports
  std::normal_distribution<decltype(ret.state.x)> normal{0.0F, 1.0F};
  ret.state.x = 10.0F * normal(gen);
  ret.state.y = 10.0F * normal(gen);
  ret.state.lateral_velocity_mps = 0.5F * normal(gen);
  ret.state.acceleration_mps2 = normal(gen);
  ret.state.heading_rate_rps = 0.1F * normal(gen);
  ret.state.heading.real = normal(gen);
  ret.state.heading.imag = normal(gen);
  // Parameters with only positive supports
  std::exponential_distribution<decltype(ret.state.longitudinal_velocity_mps)>
  exponential{1.0F};
  ret.state.longitudinal_velocity_mps = 5.0F * exponential(gen);

  return ret;
}

////////////////////////////////////////////////////////////////////////////////
Trajectory generate_trajectory(const State & start_state, Generator & gen)
{
  Trajectory ret{rosidl_runtime_cpp::MessageInitialization::ALL};
  (void)start_state;
  (void)gen;
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
Trajectory constant_trajectory(const State & start_state, const std::chrono::nanoseconds dt)
{
  Trajectory ret{rosidl_runtime_cpp::MessageInitialization::ALL};
  const auto capacity = 100LL;  // TEMP
  ret.points.reserve(capacity);
  const auto dt_s = std::chrono::duration_cast<std::chrono::duration<float>>(dt).count();
  ret.points.push_back(start_state.state);
  ret.points.back().time_from_start = time_utils::to_message(std::chrono::nanoseconds::zero());
  // quaternion increments
  const auto dw = std::cos(start_state.state.heading_rate_rps * dt_s / 2.0F);
  const auto dz = std::sin(start_state.state.heading_rate_rps * dt_s / 2.0F);
  // fill out trajectory
  for (auto i = 1LL; i < capacity; ++i) {
    const auto & last_state = ret.points.back();
    decltype(ret.points)::value_type next_state{last_state};
    // longitudinal velocity update; lateral assumed fixed
    next_state.longitudinal_velocity_mps += dt_s * next_state.acceleration_mps2;
    // heading update
    const auto w = (last_state.heading.real * dw) - (last_state.heading.imag * dz);
    const auto z = (last_state.heading.real * dz) + (last_state.heading.imag * dw);
    next_state.heading.real = w;
    next_state.heading.imag = z;
    // compute heading angles via double angle formulas
    const auto c = (w + z) * (w - z);
    const auto s = 2 * w * z;
    // position update: simplified heading effects
    const auto ds =
      dt_s * (last_state.longitudinal_velocity_mps + (0.5F * dt_s * last_state.acceleration_mps2));
    next_state.x += c * ds;
    next_state.y += s * ds;

    next_state.time_from_start = time_utils::to_message(dt * i);

    ret.points.push_back(next_state);
  }
  ret.header.stamp = time_utils::to_message(std::chrono::system_clock::now());
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
Trajectory bad_heading_trajectory(const State & start_state, const std::chrono::nanoseconds dt)
{
  Trajectory ret{rosidl_runtime_cpp::MessageInitialization::ALL};
  const auto capacity = 100LL;  // TEMP
  ret.points.reserve(capacity);
  const auto dt_s = std::chrono::duration_cast<std::chrono::duration<float>>(dt).count();
  ret.points.push_back(start_state.state);
  ret.points.back().heading.real = 0.0F;
  ret.points.back().heading.imag = 0.0F;
  ret.points.back().heading_rate_rps = 0.0F;

  ret.points.back().time_from_start = time_utils::to_message(std::chrono::nanoseconds::zero());

  // fill out trajectory
  for (auto i = 1LL; i < capacity; ++i) {
    const auto & last_state = ret.points.back();
    decltype(ret.points)::value_type next_state{last_state};

    next_state.x += dt_s * last_state.longitudinal_velocity_mps;
    next_state.y += 0.0F;
    next_state.time_from_start = time_utils::to_message(dt * i);

    ret.points.push_back(next_state);
  }
  ret.header.stamp = time_utils::to_message(std::chrono::system_clock::now());
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
Trajectory constant_velocity_trajectory(
  const float x0,
  const float y0,
  const float heading,
  const float v0,
  const std::chrono::nanoseconds dt)
{
  return constant_acceleration_turn_rate_trajectory(x0, y0, heading, v0, 0.0F, 0.0F, dt);
}

////////////////////////////////////////////////////////////////////////////////
Trajectory constant_acceleration_trajectory(
  const float x0,
  const float y0,
  const float heading,
  const float v0,
  const float a0,
  const std::chrono::nanoseconds dt)
{
  return constant_acceleration_turn_rate_trajectory(x0, y0, heading, v0, a0, 0.0F, dt);
}

////////////////////////////////////////////////////////////////////////////////
Trajectory constant_velocity_turn_rate_trajectory(
  const float x0,
  const float y0,
  const float heading,
  const float v0,
  const float turn_rate,
  const std::chrono::nanoseconds dt)
{
  return constant_acceleration_turn_rate_trajectory(x0, y0, heading, v0, 0.0F, turn_rate, dt);
}

////////////////////////////////////////////////////////////////////////////////
Trajectory constant_acceleration_turn_rate_trajectory(
  const float x0,
  const float y0,
  const float heading,
  const float v0,
  const float a0,
  const float turn_rate,
  const std::chrono::nanoseconds dt)
{
  State start_state =
    make_state(x0, y0, heading, v0, a0, turn_rate, std::chrono::system_clock::now());

  return constant_trajectory(start_state, dt);
}

////////////////////////////////////////////////////////////////////////////////
void next_state(
  const Trajectory & trajectory,
  State & state,
  const uint32_t hint,
  Generator * const gen)
{
  (void)trajectory;
  (void)state;
  (void)gen;
  (void)hint;
}

////////////////////////////////////////////////////////////////////////////////
Index progresses_towards_target(
  const Trajectory & trajectory,
  const Point & target,
  const Real heading_tolerance)
{
  auto last_err = std::numeric_limits<Real>::max();
  auto last_heading_err = -std::numeric_limits<Real>::max();
  for (auto idx = Index{}; idx < trajectory.points.size(); ++idx) {
    const auto & pt = trajectory.points[idx];
    // Pose
    const auto dx = pt.x - target.x;
    const auto dy = pt.y - target.y;
    const auto err = (dx * dx) + (dy * dy);
    if (err > last_err) {
      return idx;
    }
    last_err = err;
    // Heading: dot product should tend towards 1
    const auto dot =
      (pt.heading.real * target.heading.real) + (pt.heading.imag * target.heading.imag);
    if (dot < last_heading_err - heading_tolerance) {  // Allow for some error
      return idx;
    }
    last_heading_err = dot;
  }
  return trajectory.points.size();
}

////////////////////////////////////////////////////////////////////////////////
Index dynamically_feasible(const Trajectory & trajectory, const Real tolerance)
{
  if (trajectory.points.empty()) {
    return trajectory.points.size();
  }
  auto last_pt = trajectory.points.front();
  for (auto idx = Index{1}; idx < trajectory.points.size(); ++idx) {
    const auto & pt = trajectory.points[idx];
    const auto dt_ = time_utils::from_message(pt.time_from_start) -
      time_utils::from_message(last_pt.time_from_start);
    const auto dt = std::chrono::duration_cast<std::chrono::duration<Real>>(dt_).count();
    const auto dv = last_pt.acceleration_mps2 * dt;
    const auto ds =
      (Real{0.5} *dv * dt) + (last_pt.longitudinal_velocity_mps * dt);
    const auto dn = last_pt.lateral_velocity_mps * dt;
    const auto dth = last_pt.heading_rate_rps * dt;
    const auto check_fn = [tolerance](auto expect, auto val, auto str) -> bool {
        bool success = true;
        if (std::fabs(expect) < Real{1}) {
          success = std::fabs(expect - val) < tolerance;
        } else {
          success = (std::fabs(expect - val) / expect) < tolerance;
        }
        (void)str;
        return success;
      };
    bool ok = true;
    {
      const auto v = last_pt.longitudinal_velocity_mps + dv;
      ok = check_fn(v, pt.longitudinal_velocity_mps, "vel") && ok;
    }
    {
      const auto th = last_pt.heading;
      {
        // Dot product between angles to "check" magnitude of rotation
        const auto dot = (last_pt.heading.real * pt.heading.real) +
          (last_pt.heading.imag * pt.heading.imag);
        ok = check_fn(dot, std::cos(dth), "th_mag") && ok;
        // cross product between angles to check for consistent sign of change
        if (std::fabs(dth) > Real{0.001F}) {
          const auto cross = (last_pt.heading.real * pt.heading.imag) -
            (last_pt.heading.imag * pt.heading.real);
          // Negative product -> not pointing in same direction
          ok = (dth * cross > Real{}) && ok;
        }
      }
      // Check changes either from current or next heading
      const auto c = (th.real - th.imag) * (th.real + th.imag);
      const auto s = Real{2} *th.real * th.imag;
      const auto c2 = (pt.heading.real - pt.heading.imag) * (pt.heading.real + pt.heading.imag);
      const auto s2 = Real{2} *pt.heading.real * pt.heading.imag;
      const auto dx = (ds * c) - (dn * s);
      const auto dx2 = (ds * c2) - (dn * s2);
      ok = (check_fn(last_pt.x + dx, pt.x, "x") || check_fn(last_pt.x + dx2, pt.x, "x2")) && ok;
      const auto dy = (s * ds) + (dn * c);
      const auto dy2 = (s2 * ds) + (dn * c2);
      ok = (check_fn(last_pt.y + dy, pt.y, "y") || check_fn(last_pt.y + dy2, pt.y, "y2")) && ok;
    }
    if (!ok) {
      return idx;
    }
    last_pt = pt;
  }
  return trajectory.points.size();
}
}  // namespace motion_testing
}  // namespace motion
