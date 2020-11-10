/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <boost/optional.hpp>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/regulatory_elements/detection_area.h>

#include <scene_module/scene_module_interface.h>
#include <utilization/boost_geometry_helper.h>

using PathIndexWithPose = std::pair<size_t, geometry_msgs::Pose>;  // front index, pose
using PathIndexWithPoint2d = std::pair<size_t, Point2d>;           // front index, point2d
using PathIndexWithOffset = std::pair<size_t, double>;             // front index, offset

class DetectionAreaModule : public SceneModuleInterface
{
public:
  enum class State { GO, STOP };

  struct DebugData
  {
    double base_link2front;
    std::vector<geometry_msgs::Pose> stop_poses;
    std::vector<geometry_msgs::Pose> dead_line_poses;
    geometry_msgs::Pose first_stop_pose;
    std::vector<geometry_msgs::Point> obstacle_points;
  };

  struct PlannerParam
  {
    double stop_margin;
    bool use_dead_line;
    double dead_line_margin;
    bool use_pass_judge_line;
    double state_clear_time;
  };

public:
  DetectionAreaModule(
    const int64_t module_id, const lanelet::autoware::DetectionArea & detection_area_reg_elem,
    const PlannerParam & planner_param);

  bool modifyPathVelocity(
    autoware_planning_msgs::PathWithLaneId * path,
    autoware_planning_msgs::StopReason * stop_reason) override;

  visualization_msgs::MarkerArray createDebugMarkerArray() override;

private:
  LineString2d getStopLineGeometry2d() const;

  std::vector<geometry_msgs::Point> getObstaclePoints() const;

  bool canClearStopState() const;

  bool isOverLine(
    const autoware_planning_msgs::PathWithLaneId & path, const geometry_msgs::Pose & self_pose,
    const geometry_msgs::Pose & line_pose) const;

  bool hasEnoughBrakingDistance(
    const geometry_msgs::Pose & self_pose, const geometry_msgs::Pose & line_pose) const;

  autoware_planning_msgs::PathWithLaneId insertStopPoint(
    const autoware_planning_msgs::PathWithLaneId & path,
    const PathIndexWithPose & stop_point) const;

  boost::optional<PathIndexWithPose> createTargetPoint(
    const autoware_planning_msgs::PathWithLaneId & path, const LineString2d & stop_line,
    const double margin) const;

  // Key Feature
  const lanelet::autoware::DetectionArea & detection_area_reg_elem_;

  // State
  State state_;
  std::shared_ptr<const ros::Time> last_obstacle_found_time_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;
};
