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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SIDE_SHIFT__SIDE_SHIFT_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SIDE_SHIFT__SIDE_SHIFT_MODULE_HPP_

#include "behavior_path_planner/path_shifter/path_shifter.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <route_handler/route_handler.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/lateral_offset.hpp>

#include <memory>
#include <string>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_planning_msgs::msg::LateralOffset;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::OccupancyGrid;

struct SideShiftParameters
{
  double time_to_start_shifting;
  double min_distance_to_start_shifting;
  double shifting_lateral_jerk;
  double min_shifting_distance;
  double min_shifting_speed;
  double drivable_area_resolution;
  double drivable_area_width;
  double drivable_area_height;
};

class SideShiftModule : public SceneModuleInterface
{
public:
  SideShiftModule(
    const std::string & name, rclcpp::Node & node, const SideShiftParameters & parameters);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BT::NodeStatus updateState() override;
  void updateData() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  PathWithLaneId planCandidate() const override;
  void onEntry() override;
  void onExit() override;

  void setParameters(const SideShiftParameters & parameters);

private:
  rclcpp::Subscription<LateralOffset>::SharedPtr lateral_offset_subscriber_;

  void initVariables();

  void onLateralOffset(const LateralOffset::ConstSharedPtr lateral_offset_msg);

  // non-const methods
  void adjustDrivableArea(ShiftedPath * path) const;

  ShiftPoint calcShiftPoint() const;

  bool addShiftPoint();

  // const methods
  void publishPath(const PathWithLaneId & path) const;

  double getClosestShiftLength() const;

  // member
  PathWithLaneId refined_path_{};
  std::shared_ptr<PathWithLaneId> reference_path_{std::make_shared<PathWithLaneId>()};
  lanelet::ConstLanelets current_lanelets_;
  SideShiftParameters parameters_;

  // Current lateral offset to shift the reference path.
  double lateral_offset_{0.0};

  // Flag to check lateral offset change is requested
  bool lateral_offset_change_request_{false};

  // Triggered when offset is changed, released when start pose is refound.
  bool start_pose_reset_request_{false};

  PathShifter path_shifter_;

  ShiftedPath prev_output_;

  // NOTE: this function is ported from avoidance.
  PoseStamped getUnshiftedEgoPose(const ShiftedPath & prev_path) const;
  inline PoseStamped getEgoPose() const { return *(planner_data_->self_pose); }
  PathWithLaneId calcCenterLinePath(
    const std::shared_ptr<const PlannerData> & planner_data, const PoseStamped & pose) const;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SIDE_SHIFT__SIDE_SHIFT_MODULE_HPP_
