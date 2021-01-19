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

#include <lane_departure_checker/lane_departure_checker_node.h>

#include <autoware_utils/math/unit_conversion.h>
#include <autoware_utils/ros/marker_helper.h>
#include <autoware_utils/system/stop_watch.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/visualization/visualization.h>

using autoware_utils::rad2deg;

namespace
{
std::array<geometry_msgs::Point, 3> triangle2points(const geometry_msgs::Polygon & triangle)
{
  std::array<geometry_msgs::Point, 3> points;
  for (size_t i = 0; i < 3; ++i) {
    const auto & p = triangle.points.at(i);

    geometry_msgs::Point point;
    point.x = static_cast<double>(p.x);
    point.y = static_cast<double>(p.y);
    point.z = static_cast<double>(p.z);
    points.at(i) = point;
  }
  return points;
}

lanelet::ConstLanelets getRouteLanelets(
  const lanelet::LaneletMap & lanelet_map, const lanelet::routing::RoutingGraphPtr & routing_graph,
  const std::vector<autoware_planning_msgs::RouteSection> & route_sections,
  const double vehicle_length)
{
  lanelet::ConstLanelets route_lanelets;

  // Add preceding lanes of front route_section to prevent detection errors
  {
    const auto extention_length = 2 * vehicle_length;

    for (const auto & lane_id : route_sections.front().lane_ids) {
      for (const auto & lanelet_sequence : lanelet::utils::query::getPrecedingLaneletSequences(
             routing_graph, lanelet_map.laneletLayer.get(lane_id), extention_length)) {
        for (const auto & preceding_lanelet : lanelet_sequence) {
          route_lanelets.push_back(preceding_lanelet);
        }
      }
    }
  }

  for (const auto & route_section : route_sections) {
    for (const auto & lane_id : route_section.lane_ids) {
      route_lanelets.push_back(lanelet_map.laneletLayer.get(lane_id));
    }
  }

  // Add succeeding lanes of last route_section to prevent detection errors
  {
    const auto extention_length = 2 * vehicle_length;

    for (const auto & lane_id : route_sections.back().lane_ids) {
      for (const auto & lanelet_sequence : lanelet::utils::query::getSucceedingLaneletSequences(
             routing_graph, lanelet_map.laneletLayer.get(lane_id), extention_length)) {
        for (const auto & succeeding_lanelet : lanelet_sequence) {
          route_lanelets.push_back(succeeding_lanelet);
        }
      }
    }
  }

  return route_lanelets;
}
}  // namespace

namespace lane_departure_checker
{
LaneDepartureCheckerNode::LaneDepartureCheckerNode()
{
  // Node Parameter
  private_nh_.param("update_rate", node_param_.update_rate, 10.0);

  // Core Parameter
  param_.vehicle_info = waitForVehicleInfo();
  private_nh_.param("footprint_margin", param_.footprint_margin, 0.0);
  private_nh_.param("resample_interval", param_.resample_interval, 0.3);
  private_nh_.param("max_deceleration", param_.max_deceleration, 3.0);
  private_nh_.param("delay_time", param_.delay_time, 0.3);
  private_nh_.param("max_lateral_deviation", param_.max_lateral_deviation, 1.0);
  private_nh_.param("max_longitudinal_deviation", param_.max_longitudinal_deviation, 1.0);
  private_nh_.param("max_yaw_deviation_deg", param_.max_yaw_deviation_deg, 30.0);

  // Dynamic Reconfigure
  dynamic_reconfigure_.setCallback(boost::bind(&LaneDepartureCheckerNode::onConfig, this, _1, _2));

  // Core
  lane_departure_checker_ = std::make_unique<LaneDepartureChecker>();
  lane_departure_checker_->setParam(param_);

  // Subscriber
  sub_twist_ = private_nh_.subscribe("input/twist", 1, &LaneDepartureCheckerNode::onTwist, this);
  sub_lanelet_map_bin_ = private_nh_.subscribe(
    "input/lanelet_map_bin", 1, &LaneDepartureCheckerNode::onLaneletMapBin, this);
  sub_route_ = private_nh_.subscribe("input/route", 1, &LaneDepartureCheckerNode::onRoute, this);
  sub_reference_trajectory_ = private_nh_.subscribe(
    "input/reference_trajectory", 1, &LaneDepartureCheckerNode::onReferenceTrajectory, this);
  sub_predicted_trajectory_ = private_nh_.subscribe(
    "input/predicted_trajectory", 1, &LaneDepartureCheckerNode::onPredictedTrajectory, this);

  // Publisher
  // Nothing

  // Diagnostic Updater
  updater_.setHardwareID("lane_departure_checker");

  updater_.add(
    "lane_departure", boost::bind(&LaneDepartureCheckerNode::checkLaneDeparture, this, _1));

  updater_.add(
    "trajectory_deviation",
    boost::bind(&LaneDepartureCheckerNode::checkTrajectoryDeviation, this, _1));

  // Wait for first self pose
  self_pose_listener_.waitForFirstPose();

  // Timer
  timer_ = private_nh_.createTimer(
    ros::Rate(node_param_.update_rate), &LaneDepartureCheckerNode::onTimer, this);
}

void LaneDepartureCheckerNode::onTwist(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
  current_twist_ = msg;
}

void LaneDepartureCheckerNode::onLaneletMapBin(const autoware_lanelet2_msgs::MapBin & msg)
{
  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(msg, lanelet_map_, &traffif_rules_, &routing_graph_);
}

void LaneDepartureCheckerNode::onRoute(const autoware_planning_msgs::Route::ConstPtr & msg)
{
  route_ = msg;
}

void LaneDepartureCheckerNode::onReferenceTrajectory(
  const autoware_planning_msgs::Trajectory::ConstPtr & msg)
{
  reference_trajectory_ = msg;
}

void LaneDepartureCheckerNode::onPredictedTrajectory(
  const autoware_planning_msgs::Trajectory::ConstPtr & msg)
{
  predicted_trajectory_ = msg;
}

bool LaneDepartureCheckerNode::isDataReady()
{
  if (!current_pose_) {
    ROS_INFO_THROTTLE(5.0, "waiting for current_pose...");
    return false;
  }

  if (!current_twist_) {
    ROS_INFO_THROTTLE(5.0, "waiting for current_twist msg...");
    return false;
  }

  if (!lanelet_map_) {
    ROS_INFO_THROTTLE(5.0, "waiting for lanelet_map msg...");
    return false;
  }

  if (!route_) {
    ROS_INFO_THROTTLE(5.0, "waiting for route msg...");
    return false;
  }

  if (!reference_trajectory_) {
    ROS_INFO_THROTTLE(5.0, "waiting for reference_trajectory msg...");
    return false;
  }

  if (!predicted_trajectory_) {
    ROS_INFO_THROTTLE(5.0, "waiting for predicted_trajectory msg...");
    return false;
  }

  return true;
}

bool LaneDepartureCheckerNode::isDataTimeout()
{
  const auto now = ros::Time::now();

  constexpr double th_pose_timeout = 1.0;
  const auto pose_time_diff = current_pose_->header.stamp - now;
  if (pose_time_diff.toSec() > th_pose_timeout) {
    ROS_WARN_THROTTLE(1.0, "pose is timeout...");
    return true;
  }

  return false;
}

void LaneDepartureCheckerNode::onTimer(const ros::TimerEvent & event)
{
  std::map<std::string, double> processing_time_map;
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("Total");

  current_pose_ = self_pose_listener_.getCurrentPose();

  if (!isDataReady()) {
    return;
  }

  if (isDataTimeout()) {
    return;
  }

  processing_time_map["Node: checkData"] = stop_watch.toc(true);

  // In order to wait for both of map and route will be ready, write this not in callback but here
  if (last_route_ != route_) {
    route_lanelets_ = getRouteLanelets(
      *lanelet_map_, routing_graph_, route_->route_sections, param_.vehicle_info.vehicle_length);
    last_route_ = route_;
  }
  processing_time_map["Node: getRouteLanelets"] = stop_watch.toc(true);

  input_.current_pose = current_pose_;
  input_.current_twist = current_twist_;
  input_.lanelet_map = lanelet_map_;
  input_.route = route_;
  input_.route_lanelets = route_lanelets_;
  input_.reference_trajectory = reference_trajectory_;
  input_.predicted_trajectory = predicted_trajectory_;
  processing_time_map["Node: setInputData"] = stop_watch.toc(true);

  output_ = lane_departure_checker_->update(input_);
  processing_time_map["Node: update"] = stop_watch.toc(true);

  updater_.force_update();
  processing_time_map["Node: updateDiagnostics"] = stop_watch.toc(true);

  {
    const auto & deviation = output_.trajectory_deviation;
    debug_publisher_.publish<std_msgs::Float64>("deviation/lateral", deviation.lateral);
    debug_publisher_.publish<std_msgs::Float64>("deviation/yaw", deviation.yaw);
    debug_publisher_.publish<std_msgs::Float64>("deviation/yaw_deg", rad2deg(deviation.yaw));
  }
  processing_time_map["Node: publishTrajectoryDeviation"] = stop_watch.toc(true);

  debug_publisher_.publish<visualization_msgs::MarkerArray>("marker_array", createMarkerArray());
  processing_time_map["Node: publishDebugMarker"] = stop_watch.toc(true);

  // Merge processing_time_map
  for (const auto & m : output_.processing_time_map) {
    processing_time_map["Core: " + m.first] = m.second;
  }

  processing_time_map["Total"] = stop_watch.toc("Total");
  processing_time_publisher_.publish(processing_time_map);
}

void LaneDepartureCheckerNode::onConfig(
  const LaneDepartureCheckerConfig & config, const uint32_t level)
{
  // Node
  node_param_.visualize_lanelet = config.visualize_lanelet;

  // Core
  param_.footprint_margin = config.footprint_margin;
  param_.resample_interval = config.resample_interval;
  param_.max_deceleration = config.max_deceleration;
  param_.delay_time = config.delay_time;

  if (lane_departure_checker_) {
    lane_departure_checker_->setParam(param_);
  }
}

void LaneDepartureCheckerNode::checkLaneDeparture(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (output_.will_leave_lane) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "vehicle will leave lane";
  }

  if (output_.is_out_of_lane) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "vehicle is out of lane";
  }

  stat.summary(level, msg);
}

void LaneDepartureCheckerNode::checkTrajectoryDeviation(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;

  if (std::abs(output_.trajectory_deviation.lateral) >= param_.max_lateral_deviation) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
  }

  if (std::abs(output_.trajectory_deviation.longitudinal) >= param_.max_longitudinal_deviation) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
  }

  if (std::abs(rad2deg(output_.trajectory_deviation.yaw)) >= param_.max_yaw_deviation_deg) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
  }

  std::string msg = "OK";
  if (level == diagnostic_msgs::DiagnosticStatus::ERROR) {
    msg = "trajectory deviation is too large";
  }

  stat.addf("max lateral deviation", "%.3f", param_.max_lateral_deviation);
  stat.addf("lateral deviation", "%.3f", output_.trajectory_deviation.lateral);

  stat.addf("max longitudinal deviation", "%.3f", param_.max_longitudinal_deviation);
  stat.addf("longitudinal deviation", "%.3f", output_.trajectory_deviation.longitudinal);

  stat.addf("max yaw deviation", "%.3f", param_.max_yaw_deviation_deg);
  stat.addf("yaw deviation", "%.3f", rad2deg(output_.trajectory_deviation.yaw));

  stat.summary(level, msg);
}

visualization_msgs::MarkerArray LaneDepartureCheckerNode::createMarkerArray() const
{
  using autoware_utils::createDefaultMarker;
  using autoware_utils::createMarkerColor;
  using autoware_utils::createMarkerScale;

  visualization_msgs::MarkerArray marker_array;

  const auto base_link_z = current_pose_->pose.position.z;

  if (node_param_.visualize_lanelet) {
    // Route Lanelets
    {
      auto marker = createDefaultMarker(
        "map", "route_lanelets", 0, visualization_msgs::Marker::TRIANGLE_LIST,
        createMarkerScale(1.0, 1.0, 1.0), createMarkerColor(0.0, 0.5, 0.5, 0.5));

      for (const auto & lanelet : input_.route_lanelets) {
        std::vector<geometry_msgs::Polygon> triangles;
        lanelet::visualization::lanelet2Triangle(lanelet, &triangles);

        for (const auto & triangle : triangles) {
          for (const auto & point : triangle2points(triangle)) {
            marker.points.push_back(point);
            marker.colors.push_back(marker.color);
          }
        }
      }

      marker_array.markers.push_back(marker);
    }

    // Candidate Lanelets
    {
      auto marker = createDefaultMarker(
        "map", "candidate_lanelets", 0, visualization_msgs::Marker::TRIANGLE_LIST,
        createMarkerScale(1.0, 1.0, 1.0), createMarkerColor(1.0, 1.0, 0.0, 0.1));

      for (const auto & lanelet : output_.candidate_lanelets) {
        std::vector<geometry_msgs::Polygon> triangles;
        lanelet::visualization::lanelet2Triangle(lanelet, &triangles);

        for (const auto & triangle : triangles) {
          for (const auto & point : triangle2points(triangle)) {
            marker.points.push_back(point);
            marker.colors.push_back(marker.color);
          }
        }
      }

      marker_array.markers.push_back(marker);
    }
  }

  if (output_.resampled_trajectory.points.size() >= 2) {
    // Line of resampled_trajectory
    {
      auto marker = createDefaultMarker(
        "map", "resampled_trajectory_line", 0, visualization_msgs::Marker::LINE_STRIP,
        createMarkerScale(0.05, 0, 0), createMarkerColor(1.0, 1.0, 1.0, 0.999));

      for (const auto & p : output_.resampled_trajectory.points) {
        marker.points.push_back(p.pose.position);
        marker.colors.push_back(marker.color);
      }

      marker_array.markers.push_back(marker);
    }

    // Points of resampled_trajectory
    {
      auto marker = createDefaultMarker(
        "map", "resampled_trajectory_points", 0, visualization_msgs::Marker::SPHERE_LIST,
        createMarkerScale(0.1, 0.1, 0.1), createMarkerColor(0.0, 1.0, 0.0, 0.999));

      for (const auto & p : output_.resampled_trajectory.points) {
        marker.points.push_back(p.pose.position);
        marker.colors.push_back(marker.color);
      }

      marker_array.markers.push_back(marker);
    }
  }

  // Vehicle Footprints
  {
    const auto color_ok = createMarkerColor(0.0, 1.0, 0.0, 0.5);
    const auto color_will_leave_lane = createMarkerColor(0.5, 0.5, 0.0, 0.5);
    const auto color_is_out_of_lane = createMarkerColor(1.0, 0.0, 0.0, 0.5);

    auto color = color_ok;
    if (output_.will_leave_lane) {
      color = color_will_leave_lane;
    }
    if (output_.is_out_of_lane) {
      color = color_is_out_of_lane;
    }

    auto marker = createDefaultMarker(
      "map", "vehicle_footprints", 0, visualization_msgs::Marker::LINE_LIST,
      createMarkerScale(0.05, 0, 0), color);

    for (const auto & vehicle_footprint : output_.vehicle_footprints) {
      for (size_t i = 0; i < vehicle_footprint.size() - 1; ++i) {
        const auto p1 = vehicle_footprint.at(i);
        const auto p2 = vehicle_footprint.at(i + 1);

        marker.points.push_back(toMsg(p1.to_3d(base_link_z)));
        marker.points.push_back(toMsg(p2.to_3d(base_link_z)));
      }
    }

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}
}  // namespace lane_departure_checker
