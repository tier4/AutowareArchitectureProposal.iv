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

#ifndef BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_
#define BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_

#include "behavior_path_planner/behavior_tree_manager.hpp"
#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/scene_module/avoidance/avoidance_module.hpp"
#include "behavior_path_planner/scene_module/lane_change/lane_change_module.hpp"
#include "behavior_path_planner/scene_module/lane_following/lane_following_module.hpp"
#include "behavior_path_planner/scene_module/pull_out/pull_out_module.hpp"
#include "behavior_path_planner/scene_module/pull_over/pull_over_module.hpp"
#include "behavior_path_planner/scene_module/side_shift/side_shift_module.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"

#include <autoware_utils/ros/self_pose_listener.hpp>
#include <route_handler/route_handler.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_planning_msgs/msg/approval.hpp>
#include <autoware_planning_msgs/msg/path_change_module.hpp>
#include <autoware_planning_msgs/msg/path_change_module_array.hpp>
#include <autoware_planning_msgs/msg/path_change_module_id.hpp>
#include <autoware_planning_msgs/msg/stop_reason_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{
using ApprovalMsg = autoware_planning_msgs::msg::Approval;
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::HADMapRoute;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_planning_msgs::msg::PathChangeModule;
using autoware_planning_msgs::msg::PathChangeModuleArray;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::OccupancyGrid;
using route_handler::RouteHandler;
using visualization_msgs::msg::MarkerArray;

class BehaviorPathPlannerNode : public rclcpp::Node
{
public:
  explicit BehaviorPathPlannerNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<HADMapRoute>::SharedPtr route_subscriber_;
  rclcpp::Subscription<HADMapBin>::SharedPtr vector_map_subscriber_;
  rclcpp::Subscription<TwistStamped>::SharedPtr velocity_subscriber_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr perception_subscriber_;
  rclcpp::Subscription<ApprovalMsg>::SharedPtr external_approval_subscriber_;
  rclcpp::Subscription<PathChangeModule>::SharedPtr force_approval_subscriber_;
  rclcpp::Publisher<PathWithLaneId>::SharedPtr path_publisher_;
  rclcpp::Publisher<Path>::SharedPtr path_candidate_publisher_;
  rclcpp::Publisher<PathChangeModuleArray>::SharedPtr force_available_publisher_;
  rclcpp::Publisher<PathChangeModule>::SharedPtr plan_ready_publisher_;
  rclcpp::Publisher<PathChangeModuleArray>::SharedPtr plan_running_publisher_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr turn_signal_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<PlannerData> planner_data_;
  std::shared_ptr<BehaviorTreeManager> bt_manager_;
  autoware_utils::SelfPoseListener self_pose_listener_{this};

  std::string prev_ready_module_name_ = "NONE";

  TurnSignalDecider turn_signal_decider_;

  // setup
  void waitForData();

  // parameters
  BehaviorPathPlannerParameters getCommonParam();
  BehaviorTreeManagerParam getBehaviorTreeManagerParam();
  SideShiftParameters getSideShiftParam();
  AvoidanceParameters getAvoidanceParam();
  LaneFollowingParameters getLaneFollowingParam();
  LaneChangeParameters getLaneChangeParam();
  PullOverParameters getPullOverParam();
  PullOutParameters getPullOutParam();

  // callback
  void onVelocity(const TwistStamped::ConstSharedPtr msg);
  void onPerception(const PredictedObjects::ConstSharedPtr msg);
  void onExternalApproval(const ApprovalMsg::ConstSharedPtr msg);
  void onForceApproval(const PathChangeModule::ConstSharedPtr msg);
  void onMap(const HADMapBin::ConstSharedPtr map_msg);
  void onRoute(const HADMapRoute::ConstSharedPtr route_msg);

  /**
   * @brief Modify the path points near the goal to smoothly connect the lanelet and the goal point.
   */
  PathWithLaneId modifyPathForSmoothGoalConnection(
    const PathWithLaneId & path) const;  // (TODO) move to util

  void clipPathLength(PathWithLaneId & path) const;  // (TODO) move to util

  /**
   * @brief Execute behavior tree and publish planned data.
   */
  void run();

  /**
   * @brief extract path from behavior tree output
   */
  PathWithLaneId::SharedPtr getPath(const BehaviorModuleOutput & bt_out);

  /**
   * @brief extract path candidate from behavior tree output
   */
  PathWithLaneId::SharedPtr getPathCandidate(const BehaviorModuleOutput & bt_out);

  /**
   * @brief publish behavior module status mainly for the user interface
   */
  void publishModuleStatus(const std::vector<std::shared_ptr<SceneModuleStatus>> & statuses);

  /**
   * @brief update current pose on the planner_data_
   */
  void updateCurrentPose();

  // debug

private:
  rclcpp::Publisher<OccupancyGrid>::SharedPtr debug_drivable_area_publisher_;
  rclcpp::Publisher<Path>::SharedPtr debug_path_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_marker_publisher_;
  void publishDebugMarker(const std::vector<MarkerArray> & debug_markers);
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_
