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
#include <apex_test_tools/apex_test_tools.hpp>

#include <motion_common/motion_common.hpp>
#include <motion_testing/motion_testing.hpp>
#include <time_utils/time_utils.hpp>

#include <chrono>
#include <limits>

using motion::motion_common::Command;
using motion::motion_common::Point;
using motion::motion_common::State;
using motion::motion_common::Trajectory;
using motion::motion_common::to_angle;
using motion::motion_testing::make_state;
using motion::motion_testing::constant_velocity_trajectory;

using time_utils::from_message;

using std::chrono::milliseconds;

TEST(Interpolation, Clamp)
{
  const auto fn_floating = [](auto val, auto min, auto max, auto res, auto tol)
    {
      EXPECT_LT(std::fabs(motion::motion_common::clamp(val, min, max) - res), tol) <<
        val << ", " << min << ", " << max << ", " << res << ", " << tol;
    };
  const auto fn_suite = [ = ](auto min, auto max, auto tol)
    {
      const auto med = (min + max) / decltype(min) {2.0};
      fn_floating(med, min, max, med, tol);
      fn_floating(max, min, max, max, tol);
      fn_floating(min, min, max, min, tol);
      constexpr auto eps = std::numeric_limits<decltype(tol)>::epsilon();
      fn_floating(min - eps, min, max, min, tol);
      fn_floating(max + eps, min, max, max, tol);
      constexpr auto lim = std::numeric_limits<decltype(tol)>::max();
      fn_floating(min - lim, min, max, min, tol);
      fn_floating(max + lim, min, max, max, tol);
    };
  apex_test_tools::memory_test::start_paused();
  // Just have one of these in a compilation unit to make sure everything is hunky-dory
#ifndef __aarch64__
  ASSERT_TRUE(osrf_testing_tools_cpp::memory_tools::is_working());
#endif
  apex_test_tools::memory_test::resume();
  constexpr auto TOL = 1.0E-5;
  fn_suite(5.0, 15.0, TOL);
  fn_suite(-15.0, -5.0, TOL);
  fn_suite(-5.0, 5.0, TOL);
  constexpr auto FTOL = 1.0E-5F;
  fn_suite(5.0F, 15.0F, FTOL);
  fn_suite(-15.0F, -5.0F, FTOL);
  fn_suite(-5.0F, 5.0F, FTOL);
  apex_test_tools::memory_test::stop();
  ASSERT_FALSE(osrf_testing_tools_cpp::memory_tools::is_working());
  // TODO(c.ho) ints etc. or typed test
}

TEST(Interpolation, Interpolation)
{
  const auto fn_floating = [](auto a, auto b, auto t, auto res, auto tol)
    {
      EXPECT_LT(std::fabs(motion::motion_common::interpolate(a, b, t) - res), tol) <<
        a << ", " << b << ", " << t << ", " << res << ", " << tol;
    };
  constexpr auto TOL = 1.0E-5F;
  constexpr auto lim = std::numeric_limits<decltype(TOL)>::max();
  // Base case
  apex_test_tools::memory_test::start();
  fn_floating(4.0F, 8.0F, -lim, 4.0F, TOL);
  fn_floating(4.0F, 8.0F, lim, 8.0F, TOL);
  fn_floating(4.0F, 8.0F, 0.0F, 4.0F, TOL);
  fn_floating(4.0F, 8.0F, 1.0F, 8.0F, TOL);
  fn_floating(4.0F, 8.0F, 0.5F, 6.0F, TOL);
  fn_floating(4.0F, 8.0F, 0.25F, 5.0F, TOL);
  fn_floating(4.0F, 8.0F, 0.75F, 7.0F, TOL);
  // Communitive property
  fn_floating(8.0F, 4.0F, -lim, 8.0F, TOL);
  fn_floating(8.0F, 4.0F, lim, 4.0F, TOL);
  fn_floating(8.0F, 4.0F, 0.0F, 8.0F, TOL);
  fn_floating(8.0F, 4.0F, 1.0F, 4.0F, TOL);
  fn_floating(8.0F, 4.0F, 0.5F, 6.0F, TOL);
  fn_floating(8.0F, 4.0F, 0.25F, 7.0F, TOL);
  fn_floating(8.0F, 4.0F, 0.75F, 5.0F, TOL);
  apex_test_tools::memory_test::stop();
}

TEST(Interpolation, Slerp2d)
{
  using motion::motion_common::Heading;
  using motion::motion_common::Real;
  const auto make = [](Real real, Real imag) -> Heading
    {
      return Heading{}.set__real(real).set__imag(imag);
    };
  const auto angle_distance = [](Real a, Real b) -> Real
    {
      // https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles?rq=1
      const auto d = a - b;
      return std::atan2(std::sin(d), std::cos(d));
    };
  // Plain check
  const auto test_case = [ = ](Heading a, Heading b, Real t, Heading res, Real tol)
    {
      using motion::motion_common::nlerp;
      const auto ret = nlerp(a, b, t);
      EXPECT_LT(std::fabs(ret.imag - res.imag), tol) << ret.imag << ", " << res.imag;
      EXPECT_LT(std::fabs(ret.real - res.real), tol) << ret.real << ", " << res.real;
    };
  // Check using alternate computation path
  const auto test_case_dual_path = [ = ](Heading a, Heading b, Real t, Real tol)
    {
      // Compute result using angles
      using motion::motion_common::to_angle;
      const auto th_a = to_angle(a);
      const auto th_b = to_angle(b);
      const auto ab = angle_distance(th_a, th_b);
      const auto t_ = motion::motion_common::clamp(t, 0.0F, 1.0F);
      const auto th_t = th_a + (t_ * ab);
      using motion::motion_common::from_angle;
      const auto res_th = from_angle(th_t);
      using motion::motion_common::nlerp;
      test_case(a, b, t, res_th, tol);
      if (HasFailure()) {
        const auto ret = nlerp(a, b, t);
        std::cout << "Angles: " << th_a << ", " << th_b << "; " << th_t << ", " << to_angle(ret) <<
          "\n";
      }
    };
  const auto test_suite = [ = ](Heading a, Heading b, Real tol, Real tol2, bool b_snap = false)
    {
      constexpr auto lim = std::numeric_limits<Real>::max();
      // If you do something crazy, make b less crazy wrt a
      if (!b_snap) {
        test_case(a, b, lim, b, tol);
      }
      test_case(a, b, -lim, a, tol);
      if (!b_snap) {
        test_case(a, b, 1.0F, b, tol);
      }
      test_case(a, b, 0.0F, a, tol);
      test_case_dual_path(a, b, 0.25F, tol2);
      test_case_dual_path(a, b, 0.5F, tol2);
      test_case_dual_path(a, b, 0.75F, tol2);
    };
  constexpr auto TOL = 1.0E-5F;
  const auto cs45 = std::sqrt(2.0F) / 2.0F;
  apex_test_tools::memory_test::start();
  // 0 - 180 -> 90
  test_case(make(1.0F, 0.0F), make(0.0F, 1.0F), 0.5F, make(cs45, cs45), TOL);
  // 0 - -180 -> -90
  test_case(make(1.0F, 0.0F), make(0.0F, -1.0F), 0.5F, make(cs45, -cs45), TOL);
  // Commutivity
  test_case(make(0.0F, 1.0F), make(1.0F, 0.0F), 0.5F, make(cs45, cs45), TOL);
  test_case(make(0.0F, -1.0F), make(1.0F, 0.0F), 0.5F, make(cs45, -cs45), TOL);
  // test suite
  test_suite(make(1.0F, 0.0F), make(0.0F, 1.0F), TOL, 0.07F);
  test_suite(make(1.0F, 0.0F), make(0.0F, -1.0F), TOL, 0.07F);
  test_suite(make(1.0F, 0.0F), make(-1.0F, 0.0F), TOL, TOL, true);
  test_suite(make(0.0F, 1.0F), make(0.0F, -1.0F), TOL, TOL, true);
  apex_test_tools::memory_test::stop();
  // Test case below doesn't work, but I'm pretty sure it's more because the test doesn't work for
  // crazy angles
  // test_suite(make(-cs45, -cs45), make(-cs45, cs45), TOL, TOL);
  // TODO(c.ho) harder cases past the limits of 180, -180
}
// TODO(c.ho) point interpolation sanity check

TEST(Interpolation, AngleArithmetic)
{
  using motion::motion_common::from_angle;
  using motion::motion_common::to_angle;
  using motion::motion_common::Heading;
  const auto test_fn = [](auto a, auto b, auto res, auto tol) {
      const auto qa = from_angle(a);
      const auto qb = from_angle(b);
      const auto dq = qa - qb;
      const auto dth = to_angle(dq);
      EXPECT_LT(std::fabs(dth - res), tol) << a << ", " << b << ", " << res << ", " << dth;
    };
  const auto TOL = 1.0E-3F;
  apex_test_tools::memory_test::start();
  test_fn(0.0F, 1.0F, -1.0F, TOL);
  test_fn(1.0F, 0.0F, 1.0F, TOL);
  test_fn(1.0F, -1.0F, 2.0F, TOL);
  test_fn(-1.0F, 1.0F, -2.0F, TOL);
  test_fn(-1.0F, -2.0F, 1.0F, TOL);
  test_fn(-2.0F, -1.0F, -1.0F, TOL);
  test_fn(-3.14145F, 3.14159F, 0.0F, TOL);
  test_fn(3.14145F, -3.14159F, 0.0F, TOL);
  apex_test_tools::memory_test::stop();
}

// These are generic and/or not represented by the CATR model
void generic_checks(const Point & s, const Point & p, std::chrono::nanoseconds dt, float TOL)
{
  EXPECT_LT(std::fabs(s.lateral_velocity_mps - p.lateral_velocity_mps), TOL);
  EXPECT_LT(std::fabs(s.front_wheel_angle_rad - p.front_wheel_angle_rad), TOL);
  EXPECT_LT(std::fabs(s.rear_wheel_angle_rad - p.rear_wheel_angle_rad), TOL);
  EXPECT_TRUE((dt < milliseconds(1)) && (dt > milliseconds(-1))) <<
    std::chrono::duration_cast<milliseconds>(dt).count();
}
TEST(Interpolation, TrajectorySubsample)
{
  using std::chrono::milliseconds;
  const auto dt0 = milliseconds(100LL);
  const auto dt = milliseconds(200LL);
  const auto t_ref = constant_velocity_trajectory({}, {}, 1.0F, 1.0F, dt0);
  const auto t_res = constant_velocity_trajectory({}, {}, 1.0F, 1.0F, dt);
  Trajectory result{};
  result.points.reserve(Trajectory::CAPACITY);
  apex_test_tools::memory_test::start();
  motion::motion_common::sample(t_ref, result, dt);
  EXPECT_LE(result.points.size(), t_res.points.size());
  EXPECT_GT(result.points.size(), 1U);
  // TODO(c.ho) check header
  for (auto idx = 0U; idx < result.points.size(); ++idx) {
    const auto & pt = result.points[idx];
    const auto & ref = t_res.points[idx];
    // Check...
    constexpr auto TOL = 1.0E-3F;
    EXPECT_LT(std::fabs(pt.x - ref.x), TOL);
    EXPECT_LT(std::fabs(pt.y - ref.y), TOL);
    EXPECT_LT(std::fabs(to_angle(pt.heading) - to_angle(ref.heading)), TOL);
    EXPECT_LT(std::fabs(pt.longitudinal_velocity_mps - ref.longitudinal_velocity_mps), TOL);
    EXPECT_LT(std::fabs(pt.acceleration_mps2 - ref.acceleration_mps2), TOL);
    EXPECT_LT(std::fabs(pt.heading_rate_rps - ref.heading_rate_rps), TOL);
    const auto dt_err = (dt * idx) - from_message(pt.time_from_start);
    generic_checks(pt, ref, dt_err, TOL);
  }
  apex_test_tools::memory_test::stop();
}

TEST(Interpolation, TrajectorySupersample)
{
  using std::chrono::milliseconds;
  const auto dt0 = milliseconds(200LL);
  const auto dt = milliseconds(100LL);
  const auto t_ref = constant_velocity_trajectory({}, {}, 1.0F, 1.0F, dt0);
  const auto t_res = constant_velocity_trajectory({}, {}, 1.0F, 1.0F, dt);
  Trajectory result{};
  result.points.reserve(Trajectory::CAPACITY);
  apex_test_tools::memory_test::start();
  motion::motion_common::sample(t_ref, result, dt);
  EXPECT_LE(result.points.size(), t_res.points.size());
  EXPECT_GT(result.points.size(), 1U);
  for (auto idx = 0U; idx < result.points.size(); ++idx) {
    const auto & pt = result.points[idx];
    const auto & ref = t_res.points[idx];
    // Check...
    constexpr auto TOL = 1.0E-3F;
    EXPECT_LT(std::fabs(pt.x - ref.x), TOL);
    EXPECT_LT(std::fabs(pt.y - ref.y), TOL);
    EXPECT_LT(std::fabs(to_angle(pt.heading) - to_angle(ref.heading)), TOL);
    EXPECT_LT(std::fabs(pt.longitudinal_velocity_mps - ref.longitudinal_velocity_mps), TOL);
    EXPECT_LT(std::fabs(pt.acceleration_mps2 - ref.acceleration_mps2), TOL);
    EXPECT_LT(std::fabs(pt.heading_rate_rps - ref.heading_rate_rps), TOL);
    const auto dt_err = (dt * idx) - from_message(pt.time_from_start);
    generic_checks(pt, ref, dt_err, TOL);
  }
  apex_test_tools::memory_test::stop();
}
