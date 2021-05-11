// Copyright 2019 Autoware Foundation. All rights reserved.
// Copyright 2020 Tier IV, Inc.
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

#include "lane_change_planner/lane_changer.hpp"
#include <vector>
#include <string>
#include <memory>
#include <algorithm>
#include <utility>
#include "lane_change_planner/utilities.hpp"

std_msgs::msg::ColorRGBA toRainbow(double ratio);
visualization_msgs::msg::Marker convertToMarker(
  const autoware_perception_msgs::msg::PredictedPath & path, const int id, const std::string & ns,
  const double radius, const rclcpp::Time & stamp);

visualization_msgs::msg::Marker convertToMarker(
  const autoware_planning_msgs::msg::PathWithLaneId & path, const int id, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color, const rclcpp::Time & stamp);

visualization_msgs::msg::MarkerArray createVirtualWall(
  const geometry_msgs::msg::Pose & pose, const int id, const std::string & factor_text,
  const rclcpp::Time & stamp);

namespace lane_change_planner
{
LaneChanger::LaneChanger(const rclcpp::NodeOptions & node_options)
: Node("lane_change_planner_node", node_options)
{
  init();
}

void LaneChanger::init()
{
  // data_manager
  data_manager_ptr_ = std::make_shared<DataManager>(get_logger(), get_clock());
  route_handler_ptr_ = std::make_shared<RouteHandler>(get_logger());

  // subscription
  velocity_subscriber_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/input/velocity", rclcpp::QoS{1},
    std::bind(&DataManager::velocityCallback, data_manager_ptr_, std::placeholders::_1));
  perception_subscriber_ = create_subscription<autoware_perception_msgs::msg::DynamicObjectArray>(
    "~/input/perception", rclcpp::QoS{1},
    std::bind(&DataManager::perceptionCallback, data_manager_ptr_, std::placeholders::_1));
  lane_change_approval_subscriber_ =
    create_subscription<autoware_planning_msgs::msg::LaneChangeCommand>(
    "~/input/lane_change_approval", rclcpp::QoS{1},
    std::bind(
      &DataManager::laneChangeApprovalCallback, data_manager_ptr_,
      std::placeholders::_1));
  force_lane_change_subscriber_ =
    create_subscription<autoware_planning_msgs::msg::LaneChangeCommand>(
    "~/input/force_lane_change", rclcpp::QoS{1},
    std::bind(
      &DataManager::forceLaneChangeSignalCallback, data_manager_ptr_,
      std::placeholders::_1));

  // ROS parameters
  LaneChangerParameters parameters;
  parameters.min_stop_distance = declare_parameter("min_stop_distance", 5.0);
  parameters.stop_time = declare_parameter("stop_time", 2.0);
  parameters.hysteresis_buffer_distance = declare_parameter("hysteresis_buffer_distance", 2.0);
  parameters.backward_path_length = declare_parameter("backward_path_length", 5.0);
  parameters.forward_path_length = declare_parameter("forward_path_length", 100.0);
  parameters.lane_change_prepare_duration = declare_parameter("lane_change_prepare_duration", 2.0);
  parameters.lane_changing_duration = declare_parameter("lane_changing_duration", 4.0);
  parameters.backward_length_buffer_for_end_of_lane = declare_parameter(
    "backward_length_buffer_for_end_of_lane", 5.0);
  parameters.lane_change_finish_judge_buffer = declare_parameter(
    "lane_change_finish_judge_buffer",
    3.0);
  parameters.minimum_lane_change_length = declare_parameter("minimum_lane_change_length", 8.0);
  parameters.minimum_lane_change_velocity = declare_parameter("minimum_lane_change_velocity", 8.3);
  parameters.prediction_duration = declare_parameter("prediction_duration", 8.0);
  parameters.prediction_time_resolution = declare_parameter("prediction_time_resolution", 0.5);
  parameters.drivable_area_resolution = declare_parameter("drivable_area_resolution", 0.1);
  parameters.drivable_area_width = declare_parameter("drivable_area_width", 100.0);
  parameters.drivable_area_height = declare_parameter("drivable_area_height", 50.0);
  parameters.static_obstacle_velocity_thresh = declare_parameter(
    "static_obstacle_velocity_thresh",
    0.1);
  parameters.maximum_deceleration = declare_parameter("maximum_deceleration", 1.0);
  parameters.lane_change_sampling_num = declare_parameter("lane_change_sampling_num", 10);
  parameters.enable_abort_lane_change = declare_parameter("enable_abort_lane_change", true);
  parameters.enable_collision_check_at_prepare_phase = declare_parameter(
    "enable_collision_check_at_prepare_phase", true);
  parameters.use_predicted_path_outside_lanelet = declare_parameter(
    "use_predicted_path_outside_lanelet", true);
  parameters.use_all_predicted_path = declare_parameter("use_all_predicted_path", false);

  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  parameters.vehicle_width = vehicle_info.vehicle_width_m;
  parameters.vehicle_length = vehicle_info.vehicle_length_m;
  parameters.base_link2front = vehicle_info.max_longitudinal_offset_m;
  parameters.abort_lane_change_velocity_thresh = declare_parameter(
    "abort_lane_change_velocity_thresh", 0.5);
  parameters.abort_lane_change_angle_thresh = declare_parameter(
    "abort_lane_change_angle_thresh", 0.174533 /* 10 deg */);
  parameters.abort_lane_change_distance_thresh = declare_parameter(
    "abort_lane_change_distance_thresh", 0.3);
  parameters.refine_goal_search_radius_range = declare_parameter(
    "refine_goal_search_radius_range", 7.5);
  parameters.enable_blocked_by_obstacle = declare_parameter("enable_blocked_by_obstacle", false);

  // validation of parameters
  if (parameters.lane_change_sampling_num < 1) {
    RCLCPP_FATAL_STREAM(
      get_logger(),
      "lane_change_sampling_num must be positive integer. Given parameter: " <<
        parameters.lane_change_sampling_num << std::endl <<
        "Terminating the program...");
    exit(EXIT_FAILURE);
  }
  if (parameters.maximum_deceleration < 0.0) {
    RCLCPP_FATAL_STREAM(
      get_logger(),
      "maximum_deceleration cannot be negative value. Given parameter: " <<
        parameters.maximum_deceleration << std::endl <<
        "Terminating the program...");
    exit(EXIT_FAILURE);
  }
  data_manager_ptr_->setLaneChangerParameters(parameters);

  // route_handler
  vector_map_subscriber_ = create_subscription<autoware_lanelet2_msgs::msg::MapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&RouteHandler::mapCallback, route_handler_ptr_, std::placeholders::_1));
  route_subscriber_ = create_subscription<autoware_planning_msgs::msg::Route>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&RouteHandler::routeCallback, route_handler_ptr_, std::placeholders::_1));

  // publisher
  path_publisher_ = create_publisher<autoware_planning_msgs::msg::PathWithLaneId>(
    "~/output/lane_change_path", rclcpp::QoS{1});
  candidate_path_publisher_ = create_publisher<autoware_planning_msgs::msg::Path>(
    "~/debug/lane_change_candidate_path", rclcpp::QoS{1});
  path_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/debug/predicted_path_markers", rclcpp::QoS{1});
  stop_reason_publisher_ = create_publisher<autoware_planning_msgs::msg::StopReasonArray>(
    "~/output/stop_reasons", rclcpp::QoS{1});
  drivable_area_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "~/debug/drivable_area",
    rclcpp::QoS{1});
  lane_change_ready_publisher_ = create_publisher<autoware_planning_msgs::msg::LaneChangeStatus>(
    "~/output/lane_change_ready",
    rclcpp::QoS{1});
  lane_change_available_publisher_ =
    create_publisher<autoware_planning_msgs::msg::LaneChangeStatus>(
    "~/output/lane_change_available", rclcpp::QoS{1});

  // set state_machine
  state_machine_ptr_ = std::make_shared<StateMachine>(
    data_manager_ptr_, route_handler_ptr_,
    get_logger(), get_clock());
  state_machine_ptr_->init();
  route_init_subscriber_ = create_subscription<autoware_planning_msgs::msg::Route>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&StateMachine::initCallback, state_machine_ptr_, std::placeholders::_1));

  // Start timer. This must be done after all data (e.g. vehicle pose, velocity) are ready.
  auto timer_callback = std::bind(&LaneChanger::run, this);
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period_ns, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void LaneChanger::run()
{
  // wait until mandatory data is ready
  if (!route_handler_ptr_->isHandlerReady()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(),
      5000, "waiting for route to be ready");
    return;
  }
  if (!data_manager_ptr_->isDataReady()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(),
      5000, "waiting for vehicle pose, vehicle_velocity, and obstacles");
    return;
  }

  state_machine_ptr_->updateState();
  const auto path = state_machine_ptr_->getPath();

  const auto goal = route_handler_ptr_->getGoalPose();
  const auto goal_lane_id = route_handler_ptr_->getGoalLaneId();

  geometry_msgs::msg::Pose refined_goal;
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
    ros_parameters.refine_goal_search_radius_range, M_PI * 0.5, path, refined_goal, goal_lane_id,
    get_logger());
  refined_path.header.frame_id = "map";
  refined_path.header.stamp = this->now();

  if (!path.points.empty()) {
    path_publisher_->publish(refined_path);
  }

  publishDebugMarkers();
  publishDrivableArea(refined_path);

  // publish lane change status
  const auto lane_change_status = state_machine_ptr_->getStatus();
  autoware_planning_msgs::msg::LaneChangeStatus lane_change_ready_msg;
  lane_change_ready_msg.stamp = this->now();
  lane_change_ready_msg.status = lane_change_status.lane_change_ready;
  lane_change_ready_publisher_->publish(lane_change_ready_msg);

  autoware_planning_msgs::msg::LaneChangeStatus lane_change_available_msg;
  lane_change_available_msg.stamp = this->now();
  lane_change_available_msg.status = lane_change_status.lane_change_available;
  lane_change_available_publisher_->publish(lane_change_available_msg);

  auto candidate_path =
    util::convertToPathFromPathWithLaneId(lane_change_status.lane_change_path.path);
  candidate_path.header.frame_id = "map";
  candidate_path.header.stamp = this->now();
  candidate_path_publisher_->publish(candidate_path);
}

void LaneChanger::publishDrivableArea(const autoware_planning_msgs::msg::PathWithLaneId & path)
{
  drivable_area_publisher_->publish(path.drivable_area);
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

  visualization_msgs::msg::MarkerArray debug_markers;
  autoware_planning_msgs::msg::StopReasonArray stop_reason_array;
  // get ego vehicle path marker
  const auto & status = state_machine_ptr_->getStatus();
  if (!status.lane_change_path.path.points.empty()) {
    const auto & vehicle_predicted_path = util::convertToPredictedPath(
      status.lane_change_path.path, current_twist->twist, current_pose.pose, prediction_duration,
      time_resolution, 0, get_logger(), get_clock());
    const auto & resampled_path =
      util::resamplePredictedPath(
      vehicle_predicted_path, time_resolution, prediction_duration,
      get_logger(), get_clock());

    double radius = util::l2Norm(current_twist->twist.linear) * stop_time;
    radius = std::max(radius, min_radius);
    const auto & marker = convertToMarker(
      resampled_path, 1, "ego_lane_change_path", radius,
      this->now());
    debug_markers.markers.push_back(marker);
  }

  if (!status.lane_follow_path.points.empty()) {
    const auto & vehicle_predicted_path = util::convertToPredictedPath(
      status.lane_follow_path, current_twist->twist, current_pose.pose, prediction_duration,
      time_resolution, 0.0, get_logger(), get_clock());
    const auto & resampled_path =
      util::resamplePredictedPath(
      vehicle_predicted_path, time_resolution, prediction_duration,
      get_logger(), get_clock());

    double radius = util::l2Norm(current_twist->twist.linear) * stop_time;
    radius = std::max(radius, min_radius);
    const auto & marker = convertToMarker(
      resampled_path, 1, "ego_lane_follow_path", radius,
      this->now());
    debug_markers.markers.push_back(marker);
  }

  // get obstacle path marker
  {
    const auto & target_lanes = route_handler_ptr_->getLaneletsFromIds(status.lane_change_lane_ids);
    constexpr double check_distance = 100.0;
    // get lanes used for detection
    const auto & check_lanes = route_handler_ptr_->getCheckTargetLanesFromPath(
      status.lane_change_path.path, target_lanes, check_distance);

    const auto object_indices = util::filterObjectsByLanelets(
      *dynamic_objects, check_lanes,
      get_logger());
    for (const auto & i : object_indices) {
      const auto & obj = dynamic_objects->objects.at(i);
      std::vector<autoware_perception_msgs::msg::PredictedPath> predicted_paths;
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
          util::resamplePredictedPath(
          obj_path, time_resolution, prediction_duration,
          get_logger(), get_clock());
        double radius = util::l2Norm(obj.state.twist_covariance.twist.linear) * stop_time;
        radius = std::max(radius, min_radius);
        const auto & marker = convertToMarker(
          resampled_path, i, "object_predicted_path", radius,
          this->now());
        debug_markers.markers.push_back(marker);
      }
    }
  }

  const auto debug_data = state_machine_ptr_->getDebugData();
  // candidate paths
  {
    int i = 0;
    std_msgs::msg::ColorRGBA color;
    color.r = 1;
    color.g = 1;
    color.b = 1;
    color.a = 0.6;
    for (const auto & path : debug_data.lane_change_candidate_paths) {
      const auto marker = convertToMarker(
        path.path, i++, "candidate_lane_change_path", color,
        this->now());
      debug_markers.markers.push_back(marker);
    }
    color.r = 1;
    color.g = 0;
    color.b = 0;
    color.a = 0.9;
    const auto marker = convertToMarker(
      debug_data.selected_path, 1, "selected_path", color,
      this->now());
    debug_markers.markers.push_back(marker);
  }

  // stop point
  {
    if (state_machine_ptr_->getState() == State::BLOCKED_BY_OBSTACLE) {
      // calculate virtual wall pose
      geometry_msgs::msg::Pose wall_pose;
      tf2::Transform tf_map2base_link;
      tf2::fromMsg(debug_data.stop_point.point.pose, tf_map2base_link);
      tf2::Transform tf_base_link2front(
        tf2::Quaternion(0.0, 0.0, 0.0, 1.0),
        tf2::Vector3(ros_parameters.base_link2front, 0.0, 0.0));
      tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
      tf2::toMsg(tf_map2front, wall_pose);
      const auto virtual_wall_markers = createVirtualWall(
        wall_pose, 1, "blockedByObstacle",
        this->now());
      debug_markers.markers.insert(
        debug_markers.markers.end(), virtual_wall_markers.markers.begin(),
        virtual_wall_markers.markers.end());
    }
  }

  // stop point
  {
    if (state_machine_ptr_->getState() == State::STOPPING_LANE_CHANGE) {
      // calculate virtual wall pose
      geometry_msgs::msg::Pose wall_pose;
      tf2::Transform tf_map2base_link;
      tf2::fromMsg(debug_data.stop_point.point.pose, tf_map2base_link);
      tf2::Transform tf_base_link2front(
        tf2::Quaternion(0.0, 0.0, 0.0, 1.0),
        tf2::Vector3(ros_parameters.base_link2front, 0.0, 0.0));
      tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
      tf2::toMsg(tf_map2front, wall_pose);
      const auto virtual_wall_markers = createVirtualWall(wall_pose, 1, "Stopping", this->now());
      debug_markers.markers.insert(
        debug_markers.markers.end(), virtual_wall_markers.markers.begin(),
        virtual_wall_markers.markers.end());
    }
  }

  // create stop reason array from debug_data and state
  stop_reason_array = makeStopReasonArray(debug_data, state_machine_ptr_->getState());

  path_marker_publisher_->publish(debug_markers);
  stop_reason_publisher_->publish(stop_reason_array);
}

autoware_planning_msgs::msg::StopReasonArray LaneChanger::makeStopReasonArray(
  const DebugData & debug_data, const State & state)
{
  // create header
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = this->now();

  // create stop reason array
  autoware_planning_msgs::msg::StopReasonArray stop_reason_array;
  stop_reason_array.header = header;

  // create stop reason stamped
  autoware_planning_msgs::msg::StopReason stop_reason_msg;
  autoware_planning_msgs::msg::StopFactor stop_factor;

  if (state == lane_change_planner::State::BLOCKED_BY_OBSTACLE) {
    stop_reason_msg.reason = autoware_planning_msgs::msg::StopReason::BLOCKED_BY_OBSTACLE;
  } else if (state == lane_change_planner::State::STOPPING_LANE_CHANGE) {
    stop_reason_msg.reason = autoware_planning_msgs::msg::StopReason::STOPPING_LANE_CHANGE;
  } else {
    // not stop. return empty reason_point
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

std::vector<autoware_planning_msgs::msg::StopReason> LaneChanger::makeEmptyStopReasons()
{
  autoware_planning_msgs::msg::StopReason stop_reason_blocked;
  stop_reason_blocked.reason = autoware_planning_msgs::msg::StopReason::BLOCKED_BY_OBSTACLE;
  autoware_planning_msgs::msg::StopReason stop_reason_stopping;
  stop_reason_stopping.reason = autoware_planning_msgs::msg::StopReason::STOPPING_LANE_CHANGE;

  std::vector<autoware_planning_msgs::msg::StopReason> stop_reasons;
  stop_reasons.emplace_back(stop_reason_blocked);
  stop_reasons.emplace_back(stop_reason_stopping);
  return stop_reasons;
}

}  // namespace lane_change_planner

std_msgs::msg::ColorRGBA toRainbow(double ratio)
{
  // we want to normalize ratio so that it fits in to 6 regions
  // where each region is 256 units long
  int normalized = static_cast<int>(ratio * 256 * 6);

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
  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<double>(red) / 255;
  color.g = static_cast<double>(grn) / 255;
  color.b = static_cast<double>(blu) / 255;
  color.a = 0.3;

  return color;
}

visualization_msgs::msg::Marker convertToMarker(
  const autoware_perception_msgs::msg::PredictedPath & path, const int id, const std::string & ns,
  const double radius, const rclcpp::Time & stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;

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

  marker.lifetime = rclcpp::Duration::from_seconds(1);
  marker.frame_locked = true;

  for (std::size_t i = 0; i < path.path.size(); i++) {
    marker.points.push_back(path.path.at(i).pose.pose.position);
    marker.colors.push_back(toRainbow(static_cast<double>(i) / path.path.size()));
  }

  return marker;
}

visualization_msgs::msg::Marker convertToMarker(
  const autoware_planning_msgs::msg::PathWithLaneId & path, const int id, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color, const rclcpp::Time & stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.2;

  marker.color = color;

  marker.lifetime = rclcpp::Duration::from_seconds(1);
  marker.frame_locked = true;

  for (std::size_t i = 0; i < path.points.size(); i++) {
    marker.points.push_back(path.points.at(i).point.pose.position);
  }

  return marker;
}

visualization_msgs::msg::MarkerArray createVirtualWall(
  const geometry_msgs::msg::Pose & pose, const int id, const std::string & factor_text,
  const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray msg;
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = stamp;
    marker.ns = "stop_virtual_wall";
    marker.id = id;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
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
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = stamp;
    marker.ns = "factor_text";
    marker.id = id;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lane_change_planner::LaneChanger)
