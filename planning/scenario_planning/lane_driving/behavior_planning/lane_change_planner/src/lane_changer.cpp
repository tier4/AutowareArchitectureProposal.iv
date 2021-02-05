/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#include <lane_change_planner/lane_changer.h>
#include <lane_change_planner/utilities.h>
#include <visualization_msgs/MarkerArray.h>

std_msgs::ColorRGBA toRainbow(double ratio);
visualization_msgs::Marker convertToMarker(
  const autoware_perception_msgs::PredictedPath & path, const int id, const std::string & ns,
  const double radius);

visualization_msgs::Marker convertToMarker(
  const autoware_planning_msgs::PathWithLaneId & path, const int id, const std::string & ns,
  const std_msgs::ColorRGBA & color);

visualization_msgs::MarkerArray createVirtualWall(
  const geometry_msgs::Pose & pose, const int id, const std::string & factor_text);

namespace lane_change_planner
{
LaneChanger::LaneChanger() : pnh_("~") { init(); }

void LaneChanger::init()
{
  // data_manager
  data_manager_ptr_ = std::make_shared<DataManager>();
  route_handler_ptr_ = std::make_shared<RouteHandler>();
  velocity_subscriber_ =
    pnh_.subscribe("input/velocity", 1, &DataManager::velocityCallback, &(*data_manager_ptr_));
  perception_subscriber_ =
    pnh_.subscribe("input/perception", 1, &DataManager::perceptionCallback, &(*data_manager_ptr_));
  lane_change_approval_subscriber_ = pnh_.subscribe(
    "input/lane_change_approval", 1, &DataManager::laneChangeApprovalCallback,
    &(*data_manager_ptr_));
  force_lane_change_subscriber_ = pnh_.subscribe(
    "input/force_lane_change", 1, &DataManager::forceLaneChangeSignalCallback,
    &(*data_manager_ptr_));

  // ROS parameters
  LaneChangerParameters parameters;
  pnh_.param("min_stop_distance", parameters.min_stop_distance, 5.0);
  pnh_.param("stop_time", parameters.stop_time, 2.0);
  pnh_.param("hysteresis_buffer_distance", parameters.hysteresis_buffer_distance, 2.0);
  pnh_.param("backward_path_length", parameters.backward_path_length, 5.0);
  pnh_.param("forward_path_length", parameters.forward_path_length, 100.0);
  pnh_.param("lane_change_prepare_duration", parameters.lane_change_prepare_duration, 2.0);
  pnh_.param("lane_changing_duration", parameters.lane_changing_duration, 4.0);
  pnh_.param(
    "backward_length_buffer_for_end_of_lane", parameters.backward_length_buffer_for_end_of_lane,
    5.0);
  pnh_.param("lane_change_finish_judge_buffer", parameters.lane_change_finish_judge_buffer, 3.0);
  pnh_.param("minimum_lane_change_length", parameters.minimum_lane_change_length, 8.0);
  pnh_.param("minimum_lane_change_velocity", parameters.minimum_lane_change_velocity, 8.3);
  pnh_.param("prediction_duration", parameters.prediction_duration, 8.0);
  pnh_.param("prediction_time_resolution", parameters.prediction_time_resolution, 0.5);
  pnh_.param("drivable_area_resolution", parameters.drivable_area_resolution, 0.1);
  pnh_.param("drivable_area_width", parameters.drivable_area_width, 100.0);
  pnh_.param("drivable_area_height", parameters.drivable_area_height, 50.0);
  pnh_.param("static_obstacle_velocity_thresh", parameters.static_obstacle_velocity_thresh, 0.1);
  pnh_.param("maximum_deceleration", parameters.maximum_deceleration, 1.0);
  pnh_.param("lane_change_sampling_num", parameters.lane_change_sampling_num, 10);
  pnh_.param("enable_abort_lane_change", parameters.enable_abort_lane_change, true);
  pnh_.param(
    "enable_collision_check_at_prepare_phase", parameters.enable_collision_check_at_prepare_phase,
    true);
  pnh_.param(
    "use_predicted_path_outside_lanelet", parameters.use_predicted_path_outside_lanelet, true);
  pnh_.param("use_all_predicted_path", parameters.use_all_predicted_path, false);
  pnh_.param("/vehicle_info/vehicle_width", parameters.vehicle_width, 2.8);
  pnh_.param("/vehicle_info/vehicle_length", parameters.vehicle_length, 5.0);
  pnh_.param("/vehicle_info/max_longitudinal_offset", parameters.base_link2front, 3.74);
  pnh_.param(
    "abort_lane_change_velocity_thresh", parameters.abort_lane_change_velocity_thresh, 0.5);
  pnh_.param(
    "abort_lane_change_angle_thresh", parameters.abort_lane_change_angle_thresh,
    0.174533);  // 10 deg
  pnh_.param(
    "abort_lane_change_distance_thresh", parameters.abort_lane_change_distance_thresh, 0.3);
  pnh_.param("refine_goal_search_radius_range", parameters.refine_goal_search_radius_range, 7.5);
  pnh_.param("enable_blocked_by_obstacle", parameters.enable_blocked_by_obstacle, false);
  // validation of parameters
  if (parameters.lane_change_sampling_num < 1) {
    ROS_FATAL_STREAM(
      "lane_change_sampling_num must be positive integer. Given parameter: "
      << parameters.lane_change_sampling_num << std::endl
      << "Terminating the program...");
    exit(EXIT_FAILURE);
  }
  if (parameters.maximum_deceleration < 0.0) {
    ROS_FATAL_STREAM(
      "maximum_deceleration cannot be negative value. Given parameter: "
      << parameters.maximum_deceleration << std::endl
      << "Terminating the program...");
    exit(EXIT_FAILURE);
  }
  data_manager_ptr_->setLaneChangerParameters(parameters);

  // route_handler
  vector_map_subscriber_ =
    pnh_.subscribe("input/vector_map", 1, &RouteHandler::mapCallback, &(*route_handler_ptr_));
  route_subscriber_ =
    pnh_.subscribe("input/route", 1, &RouteHandler::routeCallback, &(*route_handler_ptr_));

  // publisher
  path_publisher_ =
    pnh_.advertise<autoware_planning_msgs::PathWithLaneId>("output/lane_change_path", 1);
  candidate_path_publisher_ =
    pnh_.advertise<autoware_planning_msgs::Path>("debug/lane_change_candidate_path", 1);
  path_marker_publisher_ =
    pnh_.advertise<visualization_msgs::MarkerArray>("debug/predicted_path_markers", 1);
  stop_reason_publisher_ =
    pnh_.advertise<autoware_planning_msgs::StopReasonArray>("output/stop_reasons", 1);
  drivable_area_publisher_ = pnh_.advertise<nav_msgs::OccupancyGrid>("debug/drivable_area", 1);
  lane_change_ready_publisher_ = pnh_.advertise<std_msgs::Bool>("output/lane_change_ready", 1);
  lane_change_available_publisher_ =
    pnh_.advertise<std_msgs::Bool>("output/lane_change_available", 1);

  waitForData();

  // set state_machine
  state_machine_ptr_ = std::make_shared<StateMachine>(data_manager_ptr_, route_handler_ptr_);
  state_machine_ptr_->init();
  route_init_subscriber_ =
    pnh_.subscribe("input/route", 1, &StateMachine::init, &(*state_machine_ptr_));
  // Start timer. This must be done after all data (e.g. vehicle pose, velocity) are ready.
  timer_ = pnh_.createTimer(ros::Duration(0.1), &LaneChanger::run, this);
}

void LaneChanger::waitForData()
{
  // wait until mandatory data is ready
  while (!route_handler_ptr_->isHandlerReady() && ros::ok()) {
    ROS_WARN_THROTTLE(5, "waiting for route to be ready");
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  while (!data_manager_ptr_->isDataReady() && ros::ok()) {
    ROS_WARN_THROTTLE(5, "waiting for vehicle pose, vehicle_velocity, and obstacles");
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
}

void LaneChanger::run(const ros::TimerEvent & event)
{
  state_machine_ptr_->updateState();
  const auto path = state_machine_ptr_->getPath();

  const auto goal = route_handler_ptr_->getGoalPose();
  const auto goal_lane_id = route_handler_ptr_->getGoalLaneId();

  geometry_msgs::Pose refined_goal;
  {
    lanelet::ConstLanelet goal_lanelet;
    if (route_handler_ptr_->getGoalLanelet(&goal_lanelet)) {
      refined_goal = util::refineGoal(goal, goal_lanelet);
    } else {
      refined_goal = goal;
    }
  }

  const auto ros_parameters = data_manager_ptr_->getLaneChangerParameters();
  auto refined_path = util::refinePath(
    ros_parameters.refine_goal_search_radius_range, M_PI * 0.5, path, refined_goal, goal_lane_id);
  refined_path.header.frame_id = "map";
  refined_path.header.stamp = ros::Time::now();

  if (!path.points.empty()) {
    path_publisher_.publish(refined_path);
  }

  publishDebugMarkers();
  publishDrivableArea(refined_path);

  // publish lane change status
  const auto lane_change_status = state_machine_ptr_->getStatus();
  std_msgs::Bool lane_change_ready_msg;
  lane_change_ready_msg.data = lane_change_status.lane_change_ready;
  lane_change_ready_publisher_.publish(lane_change_ready_msg);

  std_msgs::Bool lane_change_available_msg;
  lane_change_available_msg.data = lane_change_status.lane_change_available;
  lane_change_available_publisher_.publish(lane_change_available_msg);

  auto candidate_path =
    util::convertToPathFromPathWithLaneId(lane_change_status.lane_change_path.path);
  candidate_path.header.frame_id = "map";
  candidate_path.header.stamp = ros::Time::now();
  candidate_path_publisher_.publish(candidate_path);
}

void LaneChanger::publishDrivableArea(const autoware_planning_msgs::PathWithLaneId & path)
{
  drivable_area_publisher_.publish(path.drivable_area);
}

void LaneChanger::publishDebugMarkers()
{
  const auto current_pose = data_manager_ptr_->getCurrentSelfPose();
  const auto current_twist = data_manager_ptr_->getCurrentSelfVelocity();
  const auto dynamic_objects = data_manager_ptr_->getDynamicObjects();
  const auto ros_parameters = data_manager_ptr_->getLaneChangerParameters();

  const double min_radius = ros_parameters.min_stop_distance;
  const double stop_time = ros_parameters.stop_time;
  const double time_resolution = ros_parameters.prediction_time_resolution;
  const double prediction_duration = ros_parameters.prediction_duration;

  visualization_msgs::MarkerArray debug_markers;
  autoware_planning_msgs::StopReasonArray stop_reason_array;
  // get ego vehicle path marker
  const auto & status = state_machine_ptr_->getStatus();
  if (!status.lane_change_path.path.points.empty()) {
    const auto & vehicle_predicted_path = util::convertToPredictedPath(
      status.lane_change_path.path, current_twist->twist, current_pose.pose, prediction_duration,
      time_resolution, 0);
    const auto & resampled_path =
      util::resamplePredictedPath(vehicle_predicted_path, time_resolution, prediction_duration);

    double radius = util::l2Norm(current_twist->twist.linear) * stop_time;
    radius = std::max(radius, min_radius);
    const auto & marker = convertToMarker(resampled_path, 1, "ego_lane_change_path", radius);
    debug_markers.markers.push_back(marker);
  }

  if (!status.lane_follow_path.points.empty()) {
    const auto & vehicle_predicted_path = util::convertToPredictedPath(
      status.lane_follow_path, current_twist->twist, current_pose.pose, prediction_duration,
      time_resolution, 0.0);
    const auto & resampled_path =
      util::resamplePredictedPath(vehicle_predicted_path, time_resolution, prediction_duration);

    double radius = util::l2Norm(current_twist->twist.linear) * stop_time;
    radius = std::max(radius, min_radius);
    const auto & marker = convertToMarker(resampled_path, 1, "ego_lane_follow_path", radius);
    debug_markers.markers.push_back(marker);
  }

  // get obstacle path marker
  {
    const auto & target_lanes = route_handler_ptr_->getLaneletsFromIds(status.lane_change_lane_ids);
    constexpr double check_distance = 100.0;
    // get lanes used for detection
    const auto & check_lanes = route_handler_ptr_->getCheckTargetLanesFromPath(
      status.lane_change_path.path, target_lanes, check_distance);

    const auto object_indices = util::filterObjectsByLanelets(*dynamic_objects, check_lanes);
    for (const auto & i : object_indices) {
      const auto & obj = dynamic_objects->objects.at(i);
      std::vector<autoware_perception_msgs::PredictedPath> predicted_paths;
      if (ros_parameters.use_all_predicted_path) {
        predicted_paths = obj.state.predicted_paths;
      } else {
        auto & max_confidence_path = *(std::max_element(
          obj.state.predicted_paths.begin(), obj.state.predicted_paths.end(),
          [](const auto & path1, const auto & path2) {
            return path1.confidence > path2.confidence;
          }));
        predicted_paths.push_back(max_confidence_path);
      }
      for (const auto & obj_path : predicted_paths) {
        const auto & resampled_path =
          util::resamplePredictedPath(obj_path, time_resolution, prediction_duration);
        double radius = util::l2Norm(obj.state.twist_covariance.twist.linear) * stop_time;
        radius = std::max(radius, min_radius);
        const auto & marker = convertToMarker(resampled_path, i, "object_predicted_path", radius);
        debug_markers.markers.push_back(marker);
      }
    }
  }

  const auto debug_data = state_machine_ptr_->getDebugData();
  // candidate paths
  {
    int i = 0;
    std_msgs::ColorRGBA color;
    color.r = 1;
    color.g = 1;
    color.b = 1;
    color.a = 0.6;
    for (const auto & path : debug_data.lane_change_candidate_paths) {
      const auto marker = convertToMarker(path.path, i++, "candidate_lane_change_path", color);
      debug_markers.markers.push_back(marker);
    }
    color.r = 1;
    color.g = 0;
    color.b = 0;
    color.a = 0.9;
    const auto marker = convertToMarker(debug_data.selected_path, 1, "selected_path", color);
    debug_markers.markers.push_back(marker);
  }

  // stop point
  {
    if (state_machine_ptr_->getState() == State::BLOCKED_BY_OBSTACLE) {
      // calculate virtual wall pose
      geometry_msgs::Pose wall_pose;
      tf2::Transform tf_map2base_link;
      tf2::fromMsg(debug_data.stop_point.point.pose, tf_map2base_link);
      tf2::Transform tf_base_link2front(
        tf2::Quaternion(0.0, 0.0, 0.0, 1.0),
        tf2::Vector3(ros_parameters.base_link2front, 0.0, 0.0));
      tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
      tf2::toMsg(tf_map2front, wall_pose);
      const auto virtual_wall_markers = createVirtualWall(wall_pose, 1, "blockedByObstacle");
      debug_markers.markers.insert(
        debug_markers.markers.end(), virtual_wall_markers.markers.begin(),
        virtual_wall_markers.markers.end());
    }
  }

  // stop point
  {
    if (state_machine_ptr_->getState() == State::STOPPING_LANE_CHANGE) {
      // calculate virtual wall pose
      geometry_msgs::Pose wall_pose;
      tf2::Transform tf_map2base_link;
      tf2::fromMsg(debug_data.stop_point.point.pose, tf_map2base_link);
      tf2::Transform tf_base_link2front(
        tf2::Quaternion(0.0, 0.0, 0.0, 1.0),
        tf2::Vector3(ros_parameters.base_link2front, 0.0, 0.0));
      tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
      tf2::toMsg(tf_map2front, wall_pose);
      const auto virtual_wall_markers = createVirtualWall(wall_pose, 1, "Stopping");
      debug_markers.markers.insert(
        debug_markers.markers.end(), virtual_wall_markers.markers.begin(),
        virtual_wall_markers.markers.end());
    }
  }

  //create stop reason array from debug_data and state
  stop_reason_array = makeStopReasonArray(debug_data, state_machine_ptr_->getState());

  path_marker_publisher_.publish(debug_markers);
  stop_reason_publisher_.publish(stop_reason_array);
}

autoware_planning_msgs::StopReasonArray LaneChanger::makeStopReasonArray(
  const DebugData & debug_data, const State & state)
{
  //create header
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time::now();

  //create stop reason array
  autoware_planning_msgs::StopReasonArray stop_reason_array;
  stop_reason_array.header = header;

  //create stop reason stamped
  autoware_planning_msgs::StopReason stop_reason_msg;
  autoware_planning_msgs::StopFactor stop_factor;

  if (state == lane_change_planner::State::BLOCKED_BY_OBSTACLE) {
    stop_reason_msg.reason = autoware_planning_msgs::StopReason::BLOCKED_BY_OBSTACLE;
  } else if (state == lane_change_planner::State::STOPPING_LANE_CHANGE) {
    stop_reason_msg.reason = autoware_planning_msgs::StopReason::STOPPING_LANE_CHANGE;
  } else {
    //not stop. return empty reason_point
    stop_reason_array.stop_reasons = makeEmptyStopReasons();
    return stop_reason_array;
  }

  stop_factor.stop_pose = debug_data.stop_point.point.pose;
  if (state == lane_change_planner::State::BLOCKED_BY_OBSTACLE) {
    stop_factor.stop_factor_points.emplace_back(debug_data.stop_factor_point);
  }
  // STOPPING_LANE_CHANGE: no stop factor points
  stop_reason_msg.stop_factors.emplace_back(stop_factor);

  stop_reason_array.stop_reasons.emplace_back(stop_reason_msg);
  return stop_reason_array;
}

std::vector<autoware_planning_msgs::StopReason> LaneChanger::makeEmptyStopReasons()
{
  autoware_planning_msgs::StopReason stop_reason_blocked;
  stop_reason_blocked.reason = autoware_planning_msgs::StopReason::BLOCKED_BY_OBSTACLE;
  autoware_planning_msgs::StopReason stop_reason_stopping;
  stop_reason_stopping.reason = autoware_planning_msgs::StopReason::STOPPING_LANE_CHANGE;

  std::vector<autoware_planning_msgs::StopReason> stop_reasons;
  stop_reasons.emplace_back(stop_reason_blocked);
  stop_reasons.emplace_back(stop_reason_stopping);
  return stop_reasons;
}

}  // namespace lane_change_planner

std_msgs::ColorRGBA toRainbow(double ratio)
{
  // we want to normalize ratio so that it fits in to 6 regions
  // where each region is 256 units long
  int normalized = int(ratio * 256 * 6);

  // find the distance to the start of the closest region
  int x = normalized % 256;

  int red = 0, grn = 0, blu = 0;
  switch (normalized / 256) {
    case 0:
      red = 255;
      grn = x;
      blu = 0;
      break;  // red
    case 1:
      red = 255 - x;
      grn = 255;
      blu = 0;
      break;  // yellow
    case 2:
      red = 0;
      grn = 255;
      blu = x;
      break;  // green
    case 3:
      red = 0;
      grn = 255 - x;
      blu = 255;
      break;  // cyan
    case 4:
      red = x;
      grn = 0;
      blu = 255;
      break;  // blue
    case 5:
      red = 255;
      grn = 0;
      blu = 255 - x;
      break;  // magenta
  }
  std_msgs::ColorRGBA color;
  color.r = static_cast<double>(red) / 255;
  color.g = static_cast<double>(grn) / 255;
  color.b = static_cast<double>(blu) / 255;
  color.a = 0.3;

  return color;
}

visualization_msgs::Marker convertToMarker(
  const autoware_perception_msgs::PredictedPath & path, const int id, const std::string & ns,
  const double radius)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = radius * 2;
  marker.scale.y = radius * 2;
  marker.scale.z = radius * 2;

  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 0.3;

  marker.lifetime = ros::Duration(1);
  marker.frame_locked = true;

  for (std::size_t i = 0; i < path.path.size(); i++) {
    marker.points.push_back(path.path.at(i).pose.pose.position);
    marker.colors.push_back(toRainbow(static_cast<double>(i) / path.path.size()));
  }

  return marker;
}

visualization_msgs::Marker convertToMarker(
  const autoware_planning_msgs::PathWithLaneId & path, const int id, const std::string & ns,
  const std_msgs::ColorRGBA & color)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.2;

  marker.color = color;

  marker.lifetime = ros::Duration(1);
  marker.frame_locked = true;

  for (std::size_t i = 0; i < path.points.size(); i++) {
    marker.points.push_back(path.points.at(i).point.pose.position);
  }

  return marker;
}

visualization_msgs::MarkerArray createVirtualWall(
  const geometry_msgs::Pose & pose, const int id, const std::string & factor_text)
{
  visualization_msgs::MarkerArray msg;
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "stop_virtual_wall";
    marker.id = id;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.pose.position.z += 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 5.0;
    marker.scale.z = 2.0;
    marker.color.a = 0.5;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    msg.markers.push_back(marker);
  }
  // Factor Text
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "factor_text";
    marker.id = id;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.pose.position.z += 2.0;
    marker.scale.x = 0.0;
    marker.scale.y = 0.0;
    marker.scale.z = 1.0;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.text = factor_text;
    msg.markers.push_back(marker);
  }

  return msg;
}
