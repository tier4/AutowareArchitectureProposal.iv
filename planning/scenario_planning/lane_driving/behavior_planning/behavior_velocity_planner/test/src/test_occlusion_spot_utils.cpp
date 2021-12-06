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

#include "autoware_utils/autoware_utils.hpp"
#include "gtest/gtest.h"
#include "scene_module/occlusion_spot/occlusion_spot_utils.hpp"
#include "utilization/path_utilization.hpp"
#include "utilization/util.hpp"
#include "utils.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using Point = geometry_msgs::msg::Point;
using Vector3 = geometry_msgs::msg::Vector3;
using DynamicObjects = autoware_auto_perception_msgs::msg::PredictedObjects;
using DynamicObject = autoware_auto_perception_msgs::msg::PredictedObject;
using Semantic = autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathPoint;
using autoware_auto_planning_msgs::msg::PathWithLaneId;

autoware_auto_planning_msgs::msg::Path toPath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path_with_id)
{
  autoware_auto_planning_msgs::msg::Path path;
  for (const auto & p : path_with_id.points) {
    path.points.push_back(p.point);
  }
  return path;
}

Point setPoint(const double x, const double y, const double z)
{
  Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

Vector3 setVector3(const double x, const double y, const double z)
{
  Vector3 p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

TEST(spline, splineInterpolate)
{
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;
  autoware_auto_planning_msgs::msg::PathWithLaneId path = test::generatePath(0, 0.0, 6.0, 0.0, 7);
  const auto path_interp = behavior_velocity_planner::interpolatePath(toPath(path), 100, 0.5);
  for (const auto & p : path_interp.points) {
    std::cout << "interp" << p.pose.position.x << std::endl;
  }
  ASSERT_EQ(path_interp.points.size(), path.points.size() * 2 - +1);
}

TEST(buildPathLanelet, nominal)
{
  using behavior_velocity_planner::occlusion_spot_utils::buildPathLanelet;
  lanelet::ConstLanelet path_lanelet;
  /* straight diagonal path
      0 1 2 3 4
    0 x
    1   x
    2     x
    3       x
    4         x
  */
  autoware_auto_planning_msgs::msg::PathWithLaneId path = test::generatePath(0, 0, 4, 4, 5);
  path_lanelet = buildPathLanelet(path);
  ASSERT_EQ(path_lanelet.centerline2d().front().x(), 0.0);
  ASSERT_EQ(path_lanelet.centerline2d().front().y(), 0.0);
  ASSERT_NE(path_lanelet.centerline2d().back().x(), 4.0);
  ASSERT_NE(path_lanelet.centerline2d().back().y(), 4.0);
  std::cout << "path lanelet size: " << path_lanelet.centerline2d().size() << std::endl;
}

TEST(calcSlowDownPointsForPossibleCollision, TooManyPossibleCollisions)
{
  using behavior_velocity_planner::occlusion_spot_utils::calcSlowDownPointsForPossibleCollision;
  using behavior_velocity_planner::occlusion_spot_utils::PossibleCollisionInfo;
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;
  std::vector<PossibleCollisionInfo> possible_collisions;
  // make a path with 2000 points from x=0 to x=4
  autoware_auto_planning_msgs::msg::PathWithLaneId path =
    test::generatePath(0.0, 3.0, 4.0, 3.0, 2000);
  // make 2000 possible collision from x=0 to x=10
  test::generatePossibleCollisions(possible_collisions, 0.0, 3.0, 4.0, 3.0, 2000);

  /**
   * @brief too many possible collisions on path
   *
   * Ego -col-col-col-col-col-col-col--col-col-col-col-col-col-col-col-col-> path
   *
   */

  auto start_naive = high_resolution_clock::now();
  calcSlowDownPointsForPossibleCollision(0, path, 0, possible_collisions);

  auto end_naive = high_resolution_clock::now();
  // 2000 path * 2000 possible collisions
  EXPECT_EQ(possible_collisions.size(), size_t{2000});
  EXPECT_EQ(path.points.size(), size_t{2000});
  EXPECT_TRUE(duration_cast<microseconds>(end_naive - start_naive).count() < 2000);
  std::cout << " runtime (microsec) "
            << duration_cast<microseconds>(end_naive - start_naive).count() << std::endl;
}

TEST(calcSlowDownPointsForPossibleCollision, Nominal)
{
  using behavior_velocity_planner::occlusion_spot_utils::calcSlowDownPointsForPossibleCollision;
  using behavior_velocity_planner::occlusion_spot_utils::PossibleCollisionInfo;
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;
  std::vector<PossibleCollisionInfo> pcs;
  // for public
  {
    const double offset_from_start_to_ego = 0;
    PathWithLaneId path = test::generatePath(0.0, 3.0, 6.0, 3.0, 7);
    for (size_t i = 0; i < path.points.size(); i++) {
      path.points[i].point.longitudinal_velocity_mps = static_cast<double>(i);
    }
    test::generatePossibleCollisions(pcs, 3.0, 3.0, 6.0, 3.0, 3);
    /**
     * @brief generated path and possible collisions : path start from 2 to 6
     *    0 1 2 3 4 5 6
     * x: e-p-p-p-p-p-p-> path
     * v: 0-1-2-3-4-5-6-> velocity
     * c: N-N-N-c-NcN-c-> collision
     *    --->| longitudinal offset
     *    e : ego
     *    p : path
     *    c : collision
     */
    calcSlowDownPointsForPossibleCollision(0, path, -offset_from_start_to_ego, pcs);
    if (pcs[0].collision_path_point.longitudinal_velocity_mps - 3.0 > 1e-3) {
      for (size_t i = 0; i < path.points.size(); i++) {
        std::cout << "v : " << path.points[i].point.longitudinal_velocity_mps << "\t";
      }
      std::cout << std::endl;
      for (const auto pc : pcs) {
        std::cout << "len : " << pc.arc_lane_dist_at_collision.length << "\t";
      }
      std::cout << std::endl;
    }
    EXPECT_DOUBLE_EQ(pcs[0].collision_path_point.longitudinal_velocity_mps, 3);
    EXPECT_DOUBLE_EQ(pcs[1].collision_path_point.longitudinal_velocity_mps, 4.5);
    EXPECT_DOUBLE_EQ(pcs[2].collision_path_point.longitudinal_velocity_mps, 6);
  }

  pcs.clear();
}

TEST(generatePossibleCollisionBehindParkedVehicle, TargetVehicle)
{
  using behavior_velocity_planner::occlusion_spot_utils::createPossibleCollisionBehindParkedVehicle;
  using behavior_velocity_planner::occlusion_spot_utils::PossibleCollisionInfo;
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;

  // Set parameters: enable sidewalk obstacles
  behavior_velocity_planner::occlusion_spot_utils::PlannerParam param;
  param.detection_area_length = 100;
  param.lateral_distance_thr = 2.5;

  // make a path lanelet with 2 points from x=0 to x=6
  lanelet::ConstLanelet ll = test::toPathLanelet(test::generatePath(0.0, 0.0, 6.0, 0.0, 2));
  // There is a parked bus,car,truck along with ego path.
  std::cout << "\n"
            << " 4 -   |CAR|   |   |   |  -> ignored     \n"
            << " 3 -   |   |   |   |   |                 \n"
            << " 2 -   |TRU|   |   |   |  -> considered  \n"
            << " 1 Ego-|---|RAC|-path->|  -> ignored     \n"
            << " 0 -   |   |   |SUB|   |  -> considered  \n"
            << "-1 -   |   |   |   |   |                 \n"
            << "-2 | 0 | 1 | 2 | 3 | 4 | \n";

  std::vector<autoware_auto_perception_msgs::msg::PredictedObject> objects;
  autoware_auto_perception_msgs::msg::PredictedObject obj;
  const double RIGHT = 0.0;
  const double LEFT = M_PI;
  auto & tmp_obj_pose = obj.kinematics.initial_pose_with_covariance.pose;
  auto & tmp_obj_type = obj.classification.at(0).label;
  //! assume all vehicle is stopping
  obj.kinematics.initial_twist_with_covariance.twist.linear.x = 0.0;
  const double eps = 1e-6;
  const double dim = 1e-7;
  const double dim_h = 0.5 * dim;
  const double truck_position_x = 1.5;
  const double bus_position_x = 3.5;
  //! Ignore vehicle dimensions to simplify test
  obj.shape.dimensions = setVector3(dim, dim, dim);
  param.vehicle_info.vehicle_width = dim + eps;
  param.vehicle_info.baselink_to_front = 0.0;

  //! case within lateral distance
  tmp_obj_pose.position = setPoint(truck_position_x, 1.0, 0.0);
  tmp_obj_type = Semantic::TRUCK;
  tmp_obj_pose.orientation = autoware_utils::createQuaternionFromYaw(RIGHT);
  objects.emplace_back(obj);

  tmp_obj_pose.position = setPoint(bus_position_x, -1.0, 0.0);
  tmp_obj_pose.orientation = autoware_utils::createQuaternionFromYaw(LEFT);
  tmp_obj_type = Semantic::BUS;
  objects.emplace_back(obj);

  //! case farther than lateral distance
  tmp_obj_pose.position = setPoint(1.5, 3.0, 0.0);
  tmp_obj_type = Semantic::CAR;
  tmp_obj_pose.orientation = autoware_utils::createQuaternionFromYaw(RIGHT);
  objects.emplace_back(obj);

  //! case farther than detection area max length
  tmp_obj_pose.position = setPoint(10, 2.0, 0.0);
  tmp_obj_type = Semantic::CAR;
  tmp_obj_pose.orientation = autoware_utils::createQuaternionFromYaw(RIGHT);
  objects.emplace_back(obj);

  //! case too close
  tmp_obj_pose.position = setPoint(2.0, 0.0, 0.0);
  tmp_obj_type = Semantic::CAR;
  tmp_obj_pose.orientation = autoware_utils::createQuaternionFromYaw(LEFT);
  objects.emplace_back(obj);

  auto start_naive = high_resolution_clock::now();
  // ! TRUCK and BUS is considered as possible collision
  std::vector<PossibleCollisionInfo> pcs =
    generatePossibleCollisionBehindParkedVehicle(ll, param, 0.0, objects);
  auto end_naive = high_resolution_clock::now();

  // ! print all factor if test fails
  if (pcs.size() != static_cast<size_t>(2)) {
    // print position
    for (size_t i = 0; i < objects.size(); i++) {
      const auto p = objects.at(i).kinematics.initial_pose_with_covariance.pose.position;
      std::cout << "obj x: " << p.x << " y: " << p.y << std::endl;
    }
    // print lanelet
    const auto ll2 = ll.centerline2d().basicLineString();
    for (size_t i = 0; i < ll2.size(); i++) {
      std::cout << "ll2 x: " << ll2.at(i).x() << " y: " << ll2.at(i).y() << std::endl;
    }
    // print occlusion spot
    for (size_t i = 0; i < pcs.size(); i++) {
      const auto o = pcs.at(i).obstacle_info.position;
      std::cout << "idx: " << i << std::endl;
      std::cout << "occ x: "
                << " " << o.x << " y: " << o.y << std::endl;
      const auto arc = pcs.at(i).arc_lane_dist_at_collision;
      std::cout << "arc l: "
                << " " << arc.length << " d: " << arc.distance << std::endl;
    }
  }
  ASSERT_EQ(pcs.size(), static_cast<size_t>(2));
  EXPECT_DOUBLE_EQ(pcs.at(0).obstacle_info.position.x, truck_position_x + dim_h);
  EXPECT_DOUBLE_EQ(pcs.at(1).obstacle_info.position.x, bus_position_x + dim_h);
  std::cout << "success :  runtime (microsec) "
            << duration_cast<microseconds>(end_naive - start_naive).count() << std::endl;
}
