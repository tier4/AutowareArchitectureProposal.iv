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

#include "velocity_controller/velocity_controller_utils.hpp"

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <limits>
#include <vector>

namespace vcu = velocity_controller_utils;

TEST(test_velocity_controller_utils, isValidTrajectory)
{
  using autoware_planning_msgs::msg::Trajectory;
  using autoware_planning_msgs::msg::TrajectoryPoint;
  Trajectory traj;
  TrajectoryPoint point;
  EXPECT_FALSE(vcu::isValidTrajectory(traj));
  traj.points.push_back(point);
  EXPECT_TRUE(vcu::isValidTrajectory(traj));
  point.pose.position.x = std::numeric_limits<double>::infinity();
  traj.points.push_back(point);
  EXPECT_FALSE(vcu::isValidTrajectory(traj));
}

TEST(test_velocity_controller_utils, calcStopDistance)
{
  using autoware_planning_msgs::msg::Trajectory;
  using autoware_planning_msgs::msg::TrajectoryPoint;
  using geometry_msgs::msg::Point;
  Point current_pos;
  current_pos.x = 0.0;
  current_pos.y = 0.0;
  Trajectory traj;
  // empty trajectory : exception
  EXPECT_THROW(vcu::calcStopDistance(current_pos, traj), std::invalid_argument);
  // one point trajectory : exception
  TrajectoryPoint point;
  point.pose.position.x = 0.0;
  point.pose.position.y = 0.0;
  point.twist.linear.x = 0.0;
  traj.points.push_back(point);
  EXPECT_THROW(vcu::calcStopDistance(current_pos, traj), std::out_of_range);
  traj.points.clear();
  // non stopping trajectory: stop distance = trajectory length
  point.pose.position.x = 0.0;
  point.pose.position.y = 0.0;
  point.twist.linear.x = 1.0;
  traj.points.push_back(point);
  point.pose.position.x = 1.0;
  point.pose.position.y = 0.0;
  point.twist.linear.x = 1.0;
  traj.points.push_back(point);
  point.pose.position.x = 2.0;
  point.pose.position.y = 0.0;
  point.twist.linear.x = 1.0;
  traj.points.push_back(point);
  EXPECT_EQ(vcu::calcStopDistance(current_pos, traj), 2.0);
  // stopping trajectory: stop distance = length until stopping point
  point.pose.position.x = 3.0;
  point.pose.position.y = 0.0;
  point.twist.linear.x = 0.0;
  traj.points.push_back(point);
  point.pose.position.x = 4.0;
  point.pose.position.y = 0.0;
  point.twist.linear.x = 1.0;
  traj.points.push_back(point);
  point.pose.position.x = 5.0;
  point.pose.position.y = 0.0;
  point.twist.linear.x = 0.0;
  traj.points.push_back(point);
  EXPECT_EQ(vcu::calcStopDistance(current_pos, traj), 3.0);
}

TEST(test_velocity_controller_utils, getPitchByPose)
{
  tf2::Quaternion quaternion_tf;
  quaternion_tf.setRPY(0.0, 0.0, 0.0);
  EXPECT_EQ(vcu::getPitchByPose(tf2::toMsg(quaternion_tf)), 0.0);
  quaternion_tf.setRPY(0.0, 1.0, 0.0);
  EXPECT_EQ(vcu::getPitchByPose(tf2::toMsg(quaternion_tf)), 1.0);
}

TEST(test_velocity_controller_utils, getPitchByTraj)
{
  using autoware_planning_msgs::msg::Trajectory;
  using autoware_planning_msgs::msg::TrajectoryPoint;
  const double wheel_base = 0.9;
  /**
   * Trajectory:
   * 1    X
   *            X
   * 0 X     X
   *   0  1  2  3
   */
  Trajectory traj;
  TrajectoryPoint point;
  point.pose.position.x = 0.0;
  point.pose.position.y = 0.0;
  point.pose.position.z = 0.0;
  traj.points.push_back(point);
  // non stopping trajectory: stop distance = trajectory length
  point.pose.position.x = 1.0;
  point.pose.position.y = 0.0;
  point.pose.position.z = 1.0;
  traj.points.push_back(point);
  point.pose.position.x = 2.0;
  point.pose.position.y = 0.0;
  point.pose.position.z = 0.0;
  traj.points.push_back(point);
  point.pose.position.x = 3.0;
  point.pose.position.y = 0.0;
  point.pose.position.z = 0.5;
  traj.points.push_back(point);
  size_t closest_idx = 0;
  EXPECT_DOUBLE_EQ(std::abs(vcu::getPitchByTraj(traj, closest_idx, wheel_base)), M_PI_4);
  closest_idx = 1;
  EXPECT_DOUBLE_EQ(std::abs(vcu::getPitchByTraj(traj, closest_idx, wheel_base)), M_PI_4);
  closest_idx = 2;
  EXPECT_DOUBLE_EQ(
    std::abs(vcu::getPitchByTraj(traj, closest_idx, wheel_base)), std::atan2(0.5, 1));
  closest_idx = 3;
  EXPECT_DOUBLE_EQ(
    std::abs(vcu::getPitchByTraj(traj, closest_idx, wheel_base)), std::atan2(0.5, 1));
}

TEST(test_velocity_controller_utils, calcElevationAngle)
{
  using geometry_msgs::msg::Point;
  Point p_from;
  p_from.x = 0.0;
  p_from.y = 0.0;
  p_from.z = 0.0;
  Point p_to;
  p_to.x = 1.0;
  p_to.y = 0.0;
  p_to.z = 0.0;
  EXPECT_DOUBLE_EQ(vcu::calcElevationAngle(p_from, p_to), 0.0);
  p_to.x = 1.0;
  p_to.z = 1.0;
  EXPECT_DOUBLE_EQ(vcu::calcElevationAngle(p_from, p_to), -M_PI_4);
  p_to.x = -1.0;
  p_to.z = 1.0;
  EXPECT_DOUBLE_EQ(vcu::calcElevationAngle(p_from, p_to), -M_PI_4);
  p_to.x = 0.0;
  p_to.z = 1.0;
  EXPECT_DOUBLE_EQ(vcu::calcElevationAngle(p_from, p_to), -M_PI_2);
  p_to.x = 1.0;
  p_to.z = -1.0;
  EXPECT_DOUBLE_EQ(vcu::calcElevationAngle(p_from, p_to), M_PI_4);
  p_to.x = -1.0;
  p_to.z = -1.0;
  EXPECT_DOUBLE_EQ(vcu::calcElevationAngle(p_from, p_to), M_PI_4);
}

TEST(test_velocity_controller_utils, calcPoseAfterTimeDelay)
{
  using geometry_msgs::msg::Pose;
  const double abs_err = 1e-15;
  Pose current_pose;
  current_pose.position.x = 0.0;
  current_pose.position.y = 0.0;
  current_pose.position.z = 0.0;
  tf2::Quaternion quaternion_tf;
  quaternion_tf.setRPY(0.0, 0.0, 0.0);
  current_pose.orientation = tf2::toMsg(quaternion_tf);

  // With a delay and/or a velocity of 0.0 there is no change of position
  double delay_time = 0.0;
  double current_vel = 0.0;
  Pose delayed_pose = vcu::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x, abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  delay_time = 1.0;
  current_vel = 0.0;
  delayed_pose = vcu::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x, abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  delay_time = 0.0;
  current_vel = 1.0;
  delayed_pose = vcu::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x, abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  // With both delay and velocity: change of position
  delay_time = 1.0;
  current_vel = 1.0;
  delayed_pose = vcu::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x + current_vel * delay_time, abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  // Vary the yaw
  quaternion_tf.setRPY(0.0, 0.0, M_PI);
  current_pose.orientation = tf2::toMsg(quaternion_tf);
  delayed_pose = vcu::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x - current_vel * delay_time, abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  quaternion_tf.setRPY(0.0, 0.0, M_PI_2);
  current_pose.orientation = tf2::toMsg(quaternion_tf);
  delayed_pose = vcu::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x, abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y + current_vel * delay_time, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  quaternion_tf.setRPY(0.0, 0.0, -M_PI_2);
  current_pose.orientation = tf2::toMsg(quaternion_tf);
  delayed_pose = vcu::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x, abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y - current_vel * delay_time, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  // Vary the pitch : no effect /!\ NOTE: bug with roll of +-PI/2 which rotates the yaw by PI
  quaternion_tf.setRPY(0.0, M_PI_4, 0.0);
  current_pose.orientation = tf2::toMsg(quaternion_tf);
  delayed_pose = vcu::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x + current_vel * delay_time, abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  // Vary the roll : no effect
  quaternion_tf.setRPY(M_PI_2, 0.0, 0.0);
  current_pose.orientation = tf2::toMsg(quaternion_tf);
  delayed_pose = vcu::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x + current_vel * delay_time, abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);
}

TEST(test_velocity_controller_utils, lerp)
{
  EXPECT_EQ(vcu::lerp(0.0, 1.0, 0.5), 0.5);
  EXPECT_EQ(vcu::lerp(0.0, 1.0, 0.0), 0.0);
  EXPECT_EQ(vcu::lerp(0.0, 1.0, 1.0), 1.0);
  EXPECT_EQ(vcu::lerp(0.0, 1.0, 4.5), 4.5);
  EXPECT_EQ(vcu::lerp(1.0, 2.0, 0.5), 1.5);
  EXPECT_EQ(vcu::lerp(1.0, 2.0, 0.75), 1.75);
  EXPECT_EQ(vcu::lerp(0.0, -2.0, 0.75), -1.5);
  EXPECT_EQ(vcu::lerp(-10.0, -5.0, 0.5), -7.5);
}

TEST(test_velocity_controller_utils, lerpOrientation)
{
  geometry_msgs::msg::Quaternion result;
  tf2::Quaternion o_from;
  tf2::Quaternion o_to;
  tf2::Quaternion o_result;
  double roll;
  double pitch;
  double yaw;
  double ratio;

  o_from.setRPY(0.0, 0.0, 0.0);
  o_to.setRPY(M_PI_4, M_PI_4, M_PI_4);

  ratio = 0.0;
  result = vcu::lerpOrientation(tf2::toMsg(o_from), tf2::toMsg(o_to), ratio);
  tf2::convert(result, o_result);
  tf2::Matrix3x3(o_result).getRPY(roll, pitch, yaw);
  EXPECT_DOUBLE_EQ(roll, 0.0);
  EXPECT_DOUBLE_EQ(pitch, 0.0);
  EXPECT_DOUBLE_EQ(yaw, 0.0);

  ratio = 1.0;
  result = vcu::lerpOrientation(tf2::toMsg(o_from), tf2::toMsg(o_to), ratio);
  tf2::convert(result, o_result);
  tf2::Matrix3x3(o_result).getRPY(roll, pitch, yaw);
  EXPECT_DOUBLE_EQ(roll, M_PI_4);
  EXPECT_DOUBLE_EQ(pitch, M_PI_4);
  EXPECT_DOUBLE_EQ(yaw, M_PI_4);

  ratio = 0.5;
  o_to.setRPY(M_PI_4, 0.0, 0.0);
  result = vcu::lerpOrientation(tf2::toMsg(o_from), tf2::toMsg(o_to), ratio);
  tf2::convert(result, o_result);
  tf2::Matrix3x3(o_result).getRPY(roll, pitch, yaw);
  EXPECT_DOUBLE_EQ(roll, M_PI_4 / 2);
  EXPECT_DOUBLE_EQ(pitch, 0.0);
  EXPECT_DOUBLE_EQ(yaw, 0.0);

  o_to.setRPY(0.0, M_PI_4, 0.0);
  result = vcu::lerpOrientation(tf2::toMsg(o_from), tf2::toMsg(o_to), ratio);
  tf2::convert(result, o_result);
  tf2::Matrix3x3(o_result).getRPY(roll, pitch, yaw);
  EXPECT_DOUBLE_EQ(roll, 0.0);
  EXPECT_DOUBLE_EQ(pitch, M_PI_4 / 2);
  EXPECT_DOUBLE_EQ(yaw, 0.0);

  o_to.setRPY(0.0, 0.0, M_PI_4);
  result = vcu::lerpOrientation(tf2::toMsg(o_from), tf2::toMsg(o_to), ratio);
  tf2::convert(result, o_result);
  tf2::Matrix3x3(o_result).getRPY(roll, pitch, yaw);
  EXPECT_DOUBLE_EQ(roll, 0.0);
  EXPECT_DOUBLE_EQ(pitch, 0.0);
  EXPECT_DOUBLE_EQ(yaw, M_PI_4 / 2);
}

TEST(test_velocity_controller_utils, lerpTrajectoryPoint)
{
  using autoware_planning_msgs::msg::TrajectoryPoint;
  using geometry_msgs::msg::Point;
  const double abs_err = 1e-15;
  std::vector<TrajectoryPoint> points;
  TrajectoryPoint p;
  p.pose.position.x = 0.0;
  p.pose.position.y = 0.0;
  p.twist.linear.x = 10.0;
  p.accel.linear.x = 10.0;
  points.push_back(p);
  p.pose.position.x = 1.0;
  p.pose.position.y = 0.0;
  p.twist.linear.x = 20.0;
  p.accel.linear.x = 20.0;
  points.push_back(p);
  p.pose.position.x = 1.0;
  p.pose.position.y = 1.0;
  p.twist.linear.x = 30.0;
  p.accel.linear.x = 30.0;
  points.push_back(p);
  p.pose.position.x = 2.0;
  p.pose.position.y = 1.0;
  p.twist.linear.x = 40.0;
  p.accel.linear.x = 40.0;
  points.push_back(p);
  TrajectoryPoint result;
  Point point;

  // Points on the trajectory gives back the original trajectory points values
  point.x = 0.0;
  point.y = 0.0;
  result = vcu::lerpTrajectoryPoint(points, point);
  EXPECT_NEAR(result.pose.position.x, point.x, abs_err);
  EXPECT_NEAR(result.pose.position.y, point.y, abs_err);
  EXPECT_NEAR(result.twist.linear.x, 10.0, abs_err);
  EXPECT_NEAR(result.accel.linear.x, 10.0, abs_err);

  point.x = 1.0;
  point.y = 0.0;
  result = vcu::lerpTrajectoryPoint(points, point);
  EXPECT_NEAR(result.pose.position.x, point.x, abs_err);
  EXPECT_NEAR(result.pose.position.y, point.y, abs_err);
  EXPECT_NEAR(result.twist.linear.x, 20.0, abs_err);
  EXPECT_NEAR(result.accel.linear.x, 20.0, abs_err);

  point.x = 1.0;
  point.y = 1.0;
  result = vcu::lerpTrajectoryPoint(points, point);
  EXPECT_NEAR(result.pose.position.x, point.x, abs_err);
  EXPECT_NEAR(result.pose.position.y, point.y, abs_err);
  EXPECT_NEAR(result.twist.linear.x, 30.0, abs_err);
  EXPECT_NEAR(result.accel.linear.x, 30.0, abs_err);

  point.x = 2.0;
  point.y = 1.0;
  result = vcu::lerpTrajectoryPoint(points, point);
  EXPECT_NEAR(result.pose.position.x, point.x, abs_err);
  EXPECT_NEAR(result.pose.position.y, point.y, abs_err);
  EXPECT_NEAR(result.twist.linear.x, 40.0, abs_err);
  EXPECT_NEAR(result.accel.linear.x, 40.0, abs_err);

  // Interpolate between trajectory points
  point.x = 0.5;
  point.y = 0.0;
  result = vcu::lerpTrajectoryPoint(points, point);
  EXPECT_NEAR(result.pose.position.x, point.x, abs_err);
  EXPECT_NEAR(result.pose.position.y, point.y, abs_err);
  EXPECT_NEAR(result.twist.linear.x, 15.0, abs_err);
  EXPECT_NEAR(result.accel.linear.x, 15.0, abs_err);
  point.x = 0.75;
  point.y = 0.0;
  result = vcu::lerpTrajectoryPoint(points, point);

  EXPECT_NEAR(result.pose.position.x, point.x, abs_err);
  EXPECT_NEAR(result.pose.position.y, point.y, abs_err);
  EXPECT_NEAR(result.twist.linear.x, 17.5, abs_err);
  EXPECT_NEAR(result.accel.linear.x, 17.5, abs_err);

  // Interpolate away from the trajectory (interpolated point is projected)
  point.x = 0.5;
  point.y = -1.0;
  result = vcu::lerpTrajectoryPoint(points, point);
  EXPECT_NEAR(result.pose.position.x, point.x, abs_err);
  EXPECT_NEAR(result.pose.position.y, 0.0, abs_err);
  EXPECT_NEAR(result.twist.linear.x, 15.0, abs_err);
  EXPECT_NEAR(result.accel.linear.x, 15.0, abs_err);

  // Ambiguous projections: possibility with the lowest index is used
  point.x = 0.5;
  point.y = 0.5;
  result = vcu::lerpTrajectoryPoint(points, point);
  EXPECT_NEAR(result.pose.position.x, point.x, abs_err);
  EXPECT_NEAR(result.pose.position.y, 0.0, abs_err);
  EXPECT_NEAR(result.twist.linear.x, 15.0, abs_err);
  EXPECT_NEAR(result.accel.linear.x, 15.0, abs_err);
}

TEST(test_velocity_controller, applyDiffLimitFilter)
{
  double dt = 1.0;
  double max_val = 0.0;  // cannot increase
  double min_val = 0.0;  // cannot decrease
  double prev_val = 0.0;

  double input_val = 10.0;
  EXPECT_DOUBLE_EQ(vcu::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), 0.0);

  max_val = 1.0;  // can only increase by up to 1.0 at a time
  EXPECT_DOUBLE_EQ(vcu::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), 1.0);

  input_val = -10.0;
  EXPECT_DOUBLE_EQ(vcu::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), 0.0);

  min_val = -1.0;  // can decrease by up to -1.0 at a time
  EXPECT_DOUBLE_EQ(vcu::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), -1.0);

  dt = 5.0;  // can now increase/decrease 5 times more
  input_val = 10.0;
  EXPECT_DOUBLE_EQ(vcu::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), 5.0);
  input_val = -10.0;
  EXPECT_DOUBLE_EQ(vcu::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), -5.0);

  dt = 1.0;
  input_val = 100.0;
  for (double prev = 0.0; prev < 100.0; ++prev) {
    const double new_val = vcu::applyDiffLimitFilter(input_val, prev, dt, max_val, min_val);
    EXPECT_DOUBLE_EQ(new_val, prev + max_val);
    prev = new_val;
  }
}
