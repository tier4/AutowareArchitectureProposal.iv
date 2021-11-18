// Copyright 2021 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#include <gtest/gtest.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory>

#include "astar_search/astar_search.hpp"


using autoware::planning::astar_search::AstarSearch;
using autoware::planning::astar_search::AstarParam;
using autoware::planning::astar_search::RobotShape;
using autoware::planning::astar_search::SearchStatus;

using nav_msgs::msg::OccupancyGrid;


OccupancyGrid createOccupancyGridWithFrame()
{
  auto out = OccupancyGrid();

  // 100 x 100 cells, 20 m x 20 m
  out.info.height = 100;
  out.info.width = 100;
  out.info.resolution = 0.2f;

  // initialize with empty cells
  out.data.resize(out.info.height * out.info.width, 0);

  // vertical segments
  for (unsigned int i = 0; i < out.info.height; ++i) {
    out.data[i * out.info.width] = 100;  // left segment
    out.data[i * out.info.width + (out.info.width - 1)] = 100;  // right segment
  }

  // horizontal segments
  for (unsigned int i = 0; i < out.info.width; ++i) {
    out.data[i] = 100;  // upper segment
    out.data[(out.info.width - 1) * out.info.height + i] = 100;  // bottom segment
  }

  return out;
}

double
lateralError(const geometry_msgs::msg::Pose & actual, const geometry_msgs::msg::Pose & desired)
{
  // compute heading normal of desired point
  const auto nx = (desired.orientation.w * desired.orientation.w) -
    (desired.orientation.z * desired.orientation.z);
  const auto ny = 2.0 * desired.orientation.z * desired.orientation.w;

  // project actual onto desired basis
  const auto dx = actual.position.x - desired.position.x;
  const auto dy = actual.position.y - desired.position.y;

  // normals rotated +90 deg
  auto lateral_error_m = (dx * (-ny)) + (dy * nx);

  return lateral_error_m;
}

double longitudinalError(
  const geometry_msgs::msg::Pose & actual,
  const geometry_msgs::msg::Pose & desired)
{
  // compute heading normal of desired point
  const auto nx = (desired.orientation.w * desired.orientation.w) -
    (desired.orientation.z * desired.orientation.z);
  const auto ny = 2.0 * desired.orientation.z * desired.orientation.w;

  // project actual onto desired basis
  const auto dx = actual.position.x - desired.position.x;
  const auto dy = actual.position.y - desired.position.y;

  // normals rotated +90 deg
  auto longitudinal_error_m = (dx * nx) + (dy * ny);

  return longitudinal_error_m;
}

double angularError(
  const geometry_msgs::msg::Quaternion & actual,
  const geometry_msgs::msg::Quaternion & desired)
{
  tf2::Quaternion actual_tf, desired_tf;

  tf2::fromMsg(actual, actual_tf);
  tf2::fromMsg(desired, desired_tf);

  return actual_tf.angleShortestPath(desired_tf);
}

void
testPoseEquality(const geometry_msgs::msg::Pose & actual, const geometry_msgs::msg::Pose & desired)
{
  EXPECT_DOUBLE_EQ(actual.position.x, desired.position.x);
  EXPECT_DOUBLE_EQ(actual.position.y, desired.position.y);
  EXPECT_DOUBLE_EQ(actual.position.z, desired.position.z);

  EXPECT_DOUBLE_EQ(actual.orientation.x, desired.orientation.x);
  EXPECT_DOUBLE_EQ(actual.orientation.y, desired.orientation.y);
  EXPECT_DOUBLE_EQ(actual.orientation.z, desired.orientation.z);
  EXPECT_DOUBLE_EQ(actual.orientation.w, desired.orientation.w);
}

class AstarSearchTest : public ::testing::Test
{
public:
  AstarSearchTest()
  {
    astar_param = std::make_unique<AstarParam>(generateExampleParameters());
    astar_search = std::make_unique<AstarSearch>(*(astar_param.get()));
  }

protected:
  AstarParam generateExampleParameters()
  {
    auto params = AstarParam();

    auto robot_shape = RobotShape();
    robot_shape.length = 4.0;
    robot_shape.width = 2.0;
    robot_shape.cg2back = 2.0;

    params.use_back = true;
    params.only_behind_solutions = false;
    params.time_limit = 300000;

    params.robot_shape = robot_shape;
    params.minimum_turning_radius = 9.0;
    params.maximum_turning_radius = 20.0;
    params.turning_radius_size = 5;

    params.theta_size = 48;
    params.reverse_weight = 2.0;
    params.distance_heuristic_weight = 1.0;
    params.goal_lateral_tolerance = 0.25;
    params.goal_longitudinal_tolerance = 1.0;
    params.goal_angular_tolerance = 0.05236;  // in radians

    params.obstacle_threshold = 100;

    return params;
  }

  AstarParam generateParametersWithUnmeetableTimeLimit()
  {
    auto params = generateExampleParameters();

    params.time_limit = 1;

    return params;
  }

  std::unique_ptr<AstarParam> astar_param;
  std::unique_ptr<AstarSearch> astar_search;
};


TEST_F(AstarSearchTest, ObstacleOnStartPose) {
  astar_search->setOccupancyGrid(createOccupancyGridWithFrame());

  // start pose touches frame
  auto start_pose = geometry_msgs::msg::Pose();
  start_pose.position.x = 1.5;
  start_pose.position.y = 1.5;

  auto goal_pose = geometry_msgs::msg::Pose();
  goal_pose.position.x = 10.0;
  goal_pose.position.y = 10.0;

  auto status = astar_search->makePlan(start_pose, goal_pose);

  EXPECT_EQ(status, SearchStatus::FAILURE_COLLISION_AT_START);
  EXPECT_EQ(astar_search->getWaypoints().waypoints.size(), 0U);
}

TEST_F(AstarSearchTest, StartPoseOutOfCostmap) {
  astar_search->setOccupancyGrid(createOccupancyGridWithFrame());

  // start pose out of costmap bound
  auto start_pose = geometry_msgs::msg::Pose();
  start_pose.position.x = -5.0;
  start_pose.position.y = -5.0;

  auto goal_pose = geometry_msgs::msg::Pose();
  goal_pose.position.x = 10.0;
  goal_pose.position.y = 10.0;

  auto status = astar_search->makePlan(start_pose, goal_pose);

  EXPECT_EQ(status, SearchStatus::FAILURE_COLLISION_AT_START);
  EXPECT_EQ(astar_search->getWaypoints().waypoints.size(), 0U);
}

TEST_F(AstarSearchTest, ObstacleOnGoalPose) {
  astar_search->setOccupancyGrid(createOccupancyGridWithFrame());

  auto start_pose = geometry_msgs::msg::Pose();
  start_pose.position.x = 3.0;
  start_pose.position.y = 3.0;

  // goal pose touches frame
  auto goal_pose = geometry_msgs::msg::Pose();
  goal_pose.position.x = 10.0;
  goal_pose.position.y = 19.3;

  auto status = astar_search->makePlan(start_pose, goal_pose);

  EXPECT_EQ(status, SearchStatus::FAILURE_COLLISION_AT_GOAL);
  EXPECT_EQ(astar_search->getWaypoints().waypoints.size(), 0U);
}

TEST_F(AstarSearchTest, GoalPoseOutOfCostmap) {
  astar_search->setOccupancyGrid(createOccupancyGridWithFrame());

  auto start_pose = geometry_msgs::msg::Pose();
  start_pose.position.x = 3.0;
  start_pose.position.y = 3.0;

  // goal pose out of costmap bound
  auto goal_pose = geometry_msgs::msg::Pose();
  goal_pose.position.x = 25.0;
  goal_pose.position.y = 11.0;

  auto status = astar_search->makePlan(start_pose, goal_pose);

  EXPECT_EQ(status, SearchStatus::FAILURE_COLLISION_AT_GOAL);
  EXPECT_EQ(astar_search->getWaypoints().waypoints.size(), 0U);
}

TEST_F(AstarSearchTest, TimeLimitUnmeetable) {
  astar_search = std::make_unique<AstarSearch>(generateParametersWithUnmeetableTimeLimit());

  astar_search->setOccupancyGrid(createOccupancyGridWithFrame());

  auto start_pose = geometry_msgs::msg::Pose();
  start_pose.position.x = 3.0;
  start_pose.position.y = 3.0;

  auto goal_pose = geometry_msgs::msg::Pose();
  goal_pose.position.x = 17.0;
  goal_pose.position.y = 17.0;

  auto status = astar_search->makePlan(start_pose, goal_pose);

  EXPECT_EQ(status, SearchStatus::FAILURE_TIMEOUT_EXCEEDED);
  EXPECT_EQ(astar_search->getWaypoints().waypoints.size(), 0U);
}

TEST_F(AstarSearchTest, GoalUnreachable) {
  auto occupancy_grid = createOccupancyGridWithFrame();

  // create horizontal wall of obstacles
  for (unsigned int i = 0; i < occupancy_grid.info.width; ++i) {
    occupancy_grid.data[occupancy_grid.data.size() / 2 + i] = 100;
  }

  astar_search->setOccupancyGrid(occupancy_grid);

  auto start_pose = geometry_msgs::msg::Pose();
  start_pose.position.x = 4.0;
  start_pose.position.y = 4.0;

  auto goal_pose = geometry_msgs::msg::Pose();
  goal_pose.position.x = 16.0;
  goal_pose.position.y = 16.0;

  auto status = astar_search->makePlan(start_pose, goal_pose);

  EXPECT_EQ(status, SearchStatus::FAILURE_NO_PATH_FOUND);
  EXPECT_EQ(astar_search->getWaypoints().waypoints.size(), 0U);
}

TEST_F(AstarSearchTest, PlanningSuccessfulOnEmptyCostmap) {
  astar_search->setOccupancyGrid(createOccupancyGridWithFrame());

  auto start_pose = geometry_msgs::msg::Pose();
  start_pose.position.x = 4.0;
  start_pose.position.y = 4.0;

  auto goal_pose = geometry_msgs::msg::Pose();
  goal_pose.position.x = 16.0;
  goal_pose.position.y = 16.0;

  auto status = astar_search->makePlan(start_pose, goal_pose);

  EXPECT_EQ(status, SearchStatus::SUCCESS);
  ASSERT_GE(astar_search->getWaypoints().waypoints.size(), 2U);

  // check start pose
  testPoseEquality(astar_search->getWaypoints().waypoints.front().pose.pose, start_pose);

  // calculate errors
  auto longitudinal_error = longitudinalError(
    astar_search->getWaypoints().waypoints.back().pose.pose, goal_pose);
  auto lateral_error = lateralError(
    astar_search->getWaypoints().waypoints.back().pose.pose, goal_pose);
  auto angular_error = angularError(
    astar_search->getWaypoints().waypoints.back().pose.pose.orientation, goal_pose.orientation);

  // check goal pose
  EXPECT_LE(longitudinal_error, astar_param->goal_longitudinal_tolerance);
  EXPECT_LE(lateral_error, astar_param->goal_lateral_tolerance);
  EXPECT_LE(angular_error, astar_param->goal_angular_tolerance);
}

TEST_F(AstarSearchTest, PlanningSuccessfulOnCostmapWithObstacles) {
  auto occupancy_grid = createOccupancyGridWithFrame();

  // create horizontal wall of obstacles
  for (unsigned int i = 0; i < occupancy_grid.info.width / 2; ++i) {
    occupancy_grid.data[occupancy_grid.data.size() / 2 + i] = 100;
  }

  astar_search->setOccupancyGrid(occupancy_grid);

  auto start_pose = geometry_msgs::msg::Pose();
  start_pose.position.x = 4.0;
  start_pose.position.y = 4.0;

  auto goal_pose = geometry_msgs::msg::Pose();
  goal_pose.position.x = 16.0;
  goal_pose.position.y = 16.0;

  auto status = astar_search->makePlan(start_pose, goal_pose);

  EXPECT_EQ(status, SearchStatus::SUCCESS);
  ASSERT_GE(astar_search->getWaypoints().waypoints.size(), 2U);

  // check start pose
  testPoseEquality(astar_search->getWaypoints().waypoints.front().pose.pose, start_pose);

  // calculate errors
  auto longitudinal_error = longitudinalError(
    astar_search->getWaypoints().waypoints.back().pose.pose, goal_pose);
  auto lateral_error = lateralError(
    astar_search->getWaypoints().waypoints.back().pose.pose, goal_pose);
  auto angular_error = angularError(
    astar_search->getWaypoints().waypoints.back().pose.pose.orientation, goal_pose.orientation);

  // check goal pose
  EXPECT_LE(longitudinal_error, astar_param->goal_longitudinal_tolerance);
  EXPECT_LE(lateral_error, astar_param->goal_lateral_tolerance);
  EXPECT_LE(angular_error, astar_param->goal_angular_tolerance);
}

TEST_F(AstarSearchTest, BackwardsPlanningSuccessfulOnEmptyCostmap) {
  astar_search->setOccupancyGrid(createOccupancyGridWithFrame());

  tf2::Quaternion backwards_quaternion;
  backwards_quaternion.setRPY(0, 0, M_PI);

  auto start_pose = geometry_msgs::msg::Pose();
  start_pose.position.x = 4.0;
  start_pose.position.y = 4.0;
  start_pose.orientation = tf2::toMsg(backwards_quaternion);

  auto goal_pose = geometry_msgs::msg::Pose();
  goal_pose.position.x = 16.0;
  goal_pose.position.y = 4.0;
  goal_pose.orientation = tf2::toMsg(backwards_quaternion);

  auto status = astar_search->makePlan(start_pose, goal_pose);

  EXPECT_EQ(status, SearchStatus::SUCCESS);
  ASSERT_GE(astar_search->getWaypoints().waypoints.size(), 2U);

  // check start pose
  testPoseEquality(astar_search->getWaypoints().waypoints.front().pose.pose, start_pose);

  // calculate errors
  auto longitudinal_error = longitudinalError(
    astar_search->getWaypoints().waypoints.back().pose.pose, goal_pose);
  auto lateral_error = lateralError(
    astar_search->getWaypoints().waypoints.back().pose.pose, goal_pose);
  auto angular_error = angularError(
    astar_search->getWaypoints().waypoints.back().pose.pose.orientation, goal_pose.orientation);

  // check goal pose
  EXPECT_LE(longitudinal_error, astar_param->goal_longitudinal_tolerance);
  EXPECT_LE(lateral_error, astar_param->goal_lateral_tolerance);
  EXPECT_LE(angular_error, astar_param->goal_angular_tolerance);
}
