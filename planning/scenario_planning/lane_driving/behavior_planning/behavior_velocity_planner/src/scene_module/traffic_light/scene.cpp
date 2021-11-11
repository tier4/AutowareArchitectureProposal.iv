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

#include <scene_module/traffic_light/scene.hpp>
#include <utilization/util.hpp>

#include <boost/optional.hpp>  // To be replaced by std::optional in C++17

#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

namespace
{
std::pair<int, double> findWayPointAndDistance(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path, const Eigen::Vector2d & p)
{
  constexpr double max_lateral_dist = 3.0;
  for (size_t i = 0; i < input_path.points.size() - 1; ++i) {
    const double dx = p.x() - input_path.points.at(i).point.pose.position.x;
    const double dy = p.y() - input_path.points.at(i).point.pose.position.y;
    const double dx_wp = input_path.points.at(i + 1).point.pose.position.x -
                         input_path.points.at(i).point.pose.position.x;
    const double dy_wp = input_path.points.at(i + 1).point.pose.position.y -
                         input_path.points.at(i).point.pose.position.y;

    const double theta = std::atan2(dy, dx) - std::atan2(dy_wp, dx_wp);

    const double dist = std::hypot(dx, dy);
    const double dist_wp = std::hypot(dx_wp, dy_wp);

    // check lateral distance
    if (std::fabs(dist * std::sin(theta)) > max_lateral_dist) {
      continue;
    }

    // if the point p is back of the way point, return negative distance
    if (dist * std::cos(theta) < 0) {
      return std::make_pair(static_cast<int>(i), -1.0 * dist);
    }

    if (dist * std::cos(theta) < dist_wp) {
      return std::make_pair(static_cast<int>(i), dist);
    }
  }

  // if the way point is not found, return negative distance from the way point at 0
  const double dx = p.x() - input_path.points.at(0).point.pose.position.x;
  const double dy = p.y() - input_path.points.at(0).point.pose.position.y;
  return std::make_pair(-1, -1.0 * std::hypot(dx, dy));
}

double calcArcLengthFromWayPoint(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path, const int & src,
  const int & dst)
{
  double length = 0;
  const size_t src_idx = src >= 0 ? static_cast<size_t>(src) : 0;
  const size_t dst_idx = dst >= 0 ? static_cast<size_t>(dst) : 0;
  for (size_t i = src_idx; i < dst_idx; ++i) {
    const double dx_wp = input_path.points.at(i + 1).point.pose.position.x -
                         input_path.points.at(i).point.pose.position.x;
    const double dy_wp = input_path.points.at(i + 1).point.pose.position.y -
                         input_path.points.at(i).point.pose.position.y;
    length += std::hypot(dx_wp, dy_wp);
  }
  return length;
}

double calcSignedArcLength(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path,
  const geometry_msgs::msg::Pose & p1, const Eigen::Vector2d & p2)
{
  std::pair<int, double> src =
    findWayPointAndDistance(input_path, Eigen::Vector2d(p1.position.x, p1.position.y));
  std::pair<int, double> dst = findWayPointAndDistance(input_path, p2);
  if (dst.first == -1) {
    double dx = p1.position.x - p2.x();
    double dy = p1.position.y - p2.y();
    return -1.0 * std::hypot(dx, dy);
  }

  if (src.first < dst.first) {
    return calcArcLengthFromWayPoint(input_path, src.first, dst.first) - src.second + dst.second;
  } else if (src.first > dst.first) {
    return -1.0 *
           (calcArcLengthFromWayPoint(input_path, dst.first, src.first) - dst.second + src.second);
  } else {
    return dst.second - src.second;
  }
}

[[maybe_unused]] double calcSignedDistance(
  const geometry_msgs::msg::Pose & p1, const Eigen::Vector2d & p2)
{
  Eigen::Affine3d map2p1;
  tf2::fromMsg(p1, map2p1);
  auto basecoords_p2 = map2p1.inverse() * Eigen::Vector3d(p2.x(), p2.y(), p1.position.z);
  return basecoords_p2.x() >= 0 ? basecoords_p2.norm() : -basecoords_p2.norm();
}

bool getBackwardPointFromBasePoint(
  const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
  const Eigen::Vector2d & base_point, const double backward_length, Eigen::Vector2d & output_point)
{
  Eigen::Vector2d line_vec = line_point2 - line_point1;
  Eigen::Vector2d backward_vec = backward_length * line_vec.normalized();
  output_point = base_point + backward_vec;
  return true;
}

boost::optional<Point2d> findNearestCollisionPoint(
  const LineString2d & line1, const LineString2d & line2, const Point2d & origin)
{
  std::vector<Point2d> collision_points;
  bg::intersection(line1, line2, collision_points);

  if (collision_points.empty()) {
    return boost::none;
  }

  // check nearest collision point
  Point2d nearest_collision_point;
  double min_dist = 0.0;

  for (size_t i = 0; i < collision_points.size(); ++i) {
    double dist = bg::distance(origin, collision_points.at(i));
    if (i == 0 || dist < min_dist) {
      min_dist = dist;
      nearest_collision_point = collision_points.at(i);
    }
  }
  return nearest_collision_point;
}

bool createTargetPoint(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input, const LineString2d & stop_line,
  const double offset, size_t & target_point_idx, Eigen::Vector2d & target_point)
{
  if (input.points.size() < 2) {
    return false;
  }
  for (size_t i = 0; i < input.points.size() - 1; ++i) {
    Point2d path_line_begin = {
      input.points.at(i).point.pose.position.x, input.points.at(i).point.pose.position.y};
    Point2d path_line_end = {
      input.points.at(i + 1).point.pose.position.x, input.points.at(i + 1).point.pose.position.y};
    LineString2d path_line = {path_line_begin, path_line_end};

    // check nearest collision point
    const auto nearest_collision_point =
      findNearestCollisionPoint(path_line, stop_line, path_line_begin);
    if (!nearest_collision_point) {
      continue;
    }

    // search target point index
    target_point_idx = 0;
    double length_sum = 0;

    Eigen::Vector2d point1, point2;
    if (0 <= offset) {
      point1 << nearest_collision_point->x(), nearest_collision_point->y();
      point2 << path_line_begin.x(), path_line_begin.y();
      length_sum += (point2 - point1).norm();
      for (size_t j = i; 0 < j; --j) {
        if (offset < length_sum) {
          target_point_idx = j + 1;
          break;
        }
        point1 << input.points.at(j).point.pose.position.x,
          input.points.at(j).point.pose.position.y;
        point2 << input.points.at(j - 1).point.pose.position.x,
          input.points.at(j - 1).point.pose.position.y;
        length_sum += (point2 - point1).norm();
      }
    } else {
      point1 << nearest_collision_point->x(), nearest_collision_point->y();
      point2 << path_line_end.x(), path_line_end.y();
      length_sum -= (point2 - point1).norm();
      for (size_t j = i + 1; j < input.points.size() - 1; ++j) {
        if (length_sum < offset) {
          target_point_idx = j;
          break;
        }
        point1 << input.points.at(j).point.pose.position.x,
          input.points.at(j).point.pose.position.y;
        point2 << input.points.at(j + 1).point.pose.position.x,
          input.points.at(j + 1).point.pose.position.y;
        length_sum -= (point2 - point1).norm();
      }
    }
    // create target point
    getBackwardPointFromBasePoint(
      point2, point1, point2, std::fabs(length_sum - offset), target_point);
    return true;
  }
  return false;
}

bool calcStopPointAndInsertIndex(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path,
  const lanelet::ConstLineString3d & lanelet_stop_lines, const double & offset,
  const double & stop_line_extend_length, Eigen::Vector2d & stop_line_point,
  size_t & stop_line_point_idx)
{
  LineString2d stop_line;

  for (size_t i = 0; i < lanelet_stop_lines.size() - 1; ++i) {
    stop_line = planning_utils::extendLine(
      lanelet_stop_lines[i], lanelet_stop_lines[i + 1], stop_line_extend_length);

    // Calculate stop pose and insert index,
    // if there is a collision point between path and stop line
    if (createTargetPoint(input_path, stop_line, offset, stop_line_point_idx, stop_line_point)) {
      return true;
    }
  }
  return false;
}

geometry_msgs::msg::Point getTrafficLightPosition(
  const lanelet::ConstLineStringOrPolygon3d & traffic_light)
{
  if (!traffic_light.lineString()) {
    throw std::invalid_argument{"Traffic light is not LineString"};
  }
  geometry_msgs::msg::Point tl_center;
  auto traffic_light_ls = traffic_light.lineString().value();
  for (const auto & tl_point : traffic_light_ls) {
    tl_center.x += tl_point.x() / traffic_light_ls.size();
    tl_center.y += tl_point.y() / traffic_light_ls.size();
    tl_center.z += tl_point.z() / traffic_light_ls.size();
  }
  return tl_center;
}
autoware_perception_msgs::msg::LookingTrafficLightState initializeTrafficLightState(
  const rclcpp::Time stamp)
{
  autoware_perception_msgs::msg::LookingTrafficLightState state;
  state.header.stamp = stamp;
  state.is_module_running = true;
  state.perception.has_state = false;
  state.external.has_state = false;
  state.final.has_state = false;
  return state;
}
}  // namespace

TrafficLightModule::TrafficLightModule(
  const int64_t module_id, const lanelet::TrafficLight & traffic_light_reg_elem,
  lanelet::ConstLanelet lane, const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  traffic_light_reg_elem_(traffic_light_reg_elem),
  lane_(lane),
  state_(State::APPROACH),
  input_(Input::PERCEPTION),
  is_prev_state_stop_(false)
{
  planner_param_ = planner_param;
}

bool TrafficLightModule::modifyPathVelocity(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  autoware_planning_msgs::msg::StopReason * stop_reason)
{
  looking_tl_state_ = initializeTrafficLightState(path->header.stamp);
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  first_stop_path_point_index_ = static_cast<int>(path->points.size()) - 1;
  first_ref_stop_path_point_index_ = static_cast<int>(path->points.size()) - 1;
  *stop_reason =
    planning_utils::initializeStopReason(autoware_planning_msgs::msg::StopReason::TRAFFIC_LIGHT);

  const auto input_path = *path;

  const auto & self_pose = planner_data_->current_pose;

  // Get lanelet2 traffic lights and stop lines.
  lanelet::ConstLineString3d lanelet_stop_lines = *(traffic_light_reg_elem_.stopLine());
  lanelet::ConstLineStringsOrPolygons3d traffic_lights = traffic_light_reg_elem_.trafficLights();

  // Calculate stop pose and insert index
  Eigen::Vector2d stop_line_point{};
  size_t stop_line_point_idx{};
  if (!calcStopPointAndInsertIndex(
        input_path, lanelet_stop_lines,
        planner_param_.stop_margin + planner_data_->vehicle_info_.max_longitudinal_offset_m,
        planner_data_->stop_line_extend_length, stop_line_point, stop_line_point_idx)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "Failed to calculate stop point and insert index");
    return false;
  }

  // Calculate dist to stop pose
  const double signed_arc_length_to_stop_point =
    calcSignedArcLength(input_path, self_pose.pose, stop_line_point);

  // Check state
  if (state_ == State::APPROACH) {
    // Move to go out state if ego vehicle over deadline.
    constexpr double signed_deadline_length = -2.0;
    if (signed_arc_length_to_stop_point < signed_deadline_length) {
      RCLCPP_INFO(logger_, "APPROACH -> GO_OUT");
      state_ = State::GO_OUT;
      return true;
    }

    first_ref_stop_path_point_index_ = stop_line_point_idx;

    // Check if stop is coming.
    if (!isStopSignal(traffic_lights)) {
      is_prev_state_stop_ = false;
      return true;
    }

    // Decide whether to stop or pass even if a stop signal is received.
    if (!isPassthrough(signed_arc_length_to_stop_point)) {
      *path = insertStopPose(input_path, stop_line_point_idx, stop_line_point, stop_reason);
      is_prev_state_stop_ = true;
    }
    return true;
  } else if (state_ == State::GO_OUT) {
    // Initialize if vehicle is far from stop_line
    constexpr bool use_initialization_after_start = true;
    constexpr double restart_length = 1.0;
    if (use_initialization_after_start) {
      if (signed_arc_length_to_stop_point > restart_length) {
        RCLCPP_INFO(logger_, "GO_OUT(RESTART) -> APPROACH");
        state_ = State::APPROACH;
      }
    }
    return true;
  }

  return false;
}

bool TrafficLightModule::isStopSignal(const lanelet::ConstLineStringsOrPolygons3d & traffic_lights)
{
  if (!updateTrafficLightState(traffic_lights)) {
    return false;
  }

  return looking_tl_state_.final.judge ==
         autoware_perception_msgs::msg::TrafficLightStateWithJudge::STOP;
}

bool TrafficLightModule::updateTrafficLightState(
  const lanelet::ConstLineStringsOrPolygons3d & traffic_lights)
{
  autoware_perception_msgs::msg::TrafficLightStateStamped tl_state_perception;
  autoware_perception_msgs::msg::TrafficLightStateStamped tl_state_external;
  bool found_perception =
    getHighestConfidenceTrafficLightState(traffic_lights, tl_state_perception);
  bool found_external = getExternalTrafficLightState(traffic_lights, tl_state_external);

  if (!found_perception && !found_external) {
    // Don't stop when UNKNOWN or TIMEOUT as discussed at #508
    input_ = Input::NONE;
    return false;
  }

  if (found_perception) {
    looking_tl_state_.perception = generateTlStateWithJudgeFromTlState(tl_state_perception.state);
    looking_tl_state_.final = looking_tl_state_.perception;
    input_ = Input::PERCEPTION;
  }

  if (found_external) {
    looking_tl_state_.external = generateTlStateWithJudgeFromTlState(tl_state_external.state);
    looking_tl_state_.final = looking_tl_state_.external;
    input_ = Input::EXTERNAL;
  }

  return true;
}

bool TrafficLightModule::isPassthrough(const double & signed_arc_length) const
{
  const double max_acc = planner_data_->max_stop_acceleration_threshold;
  const double max_jerk = planner_data_->max_stop_jerk_threshold;
  const double delay_response_time = planner_data_->delay_response_time;

  const double reachable_distance =
    planner_data_->current_velocity->twist.linear.x * planner_param_.yellow_lamp_period;

  if (!planner_data_->current_accel) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "[traffic_light] empty current acc! check current vel has been received.");
    return false;
  }
  // Calculate distance until ego vehicle decide not to stop,
  // taking into account the jerk and acceleration.
  const double pass_judge_line_distance = planning_utils::calcJudgeLineDistWithJerkLimit(
    planner_data_->current_velocity->twist.linear.x, planner_data_->current_accel.get(), max_acc,
    max_jerk, delay_response_time);

  const bool distance_stoppable = pass_judge_line_distance < signed_arc_length;
  const bool slow_velocity = planner_data_->current_velocity->twist.linear.x < 2.0;
  const bool stoppable = distance_stoppable || slow_velocity;
  const bool reachable = signed_arc_length < reachable_distance;

  const auto & enable_pass_judge = planner_param_.enable_pass_judge;

  if (enable_pass_judge && !stoppable && !is_prev_state_stop_ && input_ == Input::PERCEPTION) {
    // Cannot stop under acceleration and jerk limits.
    // However, ego vehicle can't enter the intersection while the light is yellow.
    // It is called dilemma zone. Make a stop decision to be safe.
    if (!reachable) {
      // dilemma zone: emergency stop
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "[traffic_light] cannot pass through intersection during yellow lamp!");
      return false;
    } else {
      // pass through
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000, "[traffic_light] can pass through intersection during yellow lamp");
      return true;
    }
  } else {
    return false;
  }
}

bool TrafficLightModule::isTrafficLightStateStop(
  const autoware_perception_msgs::msg::TrafficLightState & tl_state) const
{
  if (hasLampState(tl_state, autoware_perception_msgs::msg::LampState::GREEN)) {
    return false;
  }

  const std::string turn_direction = lane_.attributeOr("turn_direction", "else");

  if (turn_direction == "else") {
    return true;
  }
  if (
    turn_direction == "right" &&
    hasLampState(tl_state, autoware_perception_msgs::msg::LampState::RIGHT)) {
    return false;
  }
  if (
    turn_direction == "left" &&
    hasLampState(tl_state, autoware_perception_msgs::msg::LampState::LEFT)) {
    return false;
  }
  if (
    turn_direction == "straight" &&
    hasLampState(tl_state, autoware_perception_msgs::msg::LampState::UP)) {
    return false;
  }

  return true;
}

bool TrafficLightModule::getHighestConfidenceTrafficLightState(
  const lanelet::ConstLineStringsOrPolygons3d & traffic_lights,
  autoware_perception_msgs::msg::TrafficLightStateStamped & highest_confidence_tl_state)
{
  // search traffic light state
  bool found = false;
  double highest_confidence = 0.0;
  std::string reason;
  for (const auto & traffic_light : traffic_lights) {
    // traffic light must be linestrings
    if (!traffic_light.isLineString()) {
      reason = "NotLineString";
      continue;
    }

    const int id = static_cast<lanelet::ConstLineString3d>(traffic_light).id();
    RCLCPP_DEBUG(logger_, "traffic light id: %d (on route)", id);
    const auto tl_state_stamped = planner_data_->getTrafficLightState(id);
    if (!tl_state_stamped) {
      reason = "TrafficLightStateNotFound";
      continue;
    }

    const auto header = tl_state_stamped->header;
    const auto tl_state = tl_state_stamped->state;
    if (!((clock_->now() - header.stamp).seconds() < planner_param_.tl_state_timeout)) {
      reason = "TimeOut";
      continue;
    }

    if (
      tl_state.lamp_states.empty() ||
      tl_state.lamp_states.front().type == autoware_perception_msgs::msg::LampState::UNKNOWN) {
      reason = "LampStateUnknown";
      continue;
    }

    if (highest_confidence < tl_state.lamp_states.front().confidence) {
      highest_confidence = tl_state.lamp_states.front().confidence;
      highest_confidence_tl_state = *tl_state_stamped;
      try {
        debug_data_.traffic_light_points.push_back(getTrafficLightPosition(traffic_light));
        debug_data_.highest_confidence_traffic_light_point = getTrafficLightPosition(traffic_light);
      } catch (const std::invalid_argument & ex) {
        RCLCPP_WARN_STREAM(logger_, ex.what());
        continue;
      }
      found = true;
    }
  }
  if (!found) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000 /* ms */, "cannot find traffic light lamp state (%s).",
      reason.c_str());
    return false;
  }
  return true;
}

autoware_auto_planning_msgs::msg::PathWithLaneId TrafficLightModule::insertStopPose(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input,
  const size_t & insert_target_point_idx, const Eigen::Vector2d & target_point,
  autoware_planning_msgs::msg::StopReason * stop_reason)
{
  autoware_auto_planning_msgs::msg::PathWithLaneId modified_path;
  modified_path = input;

  // Create stop pose
  const int target_velocity_point_idx = std::max(static_cast<int>(insert_target_point_idx - 1), 0);
  auto target_point_with_lane_id = modified_path.points.at(target_velocity_point_idx);
  target_point_with_lane_id.point.pose.position.x = target_point.x();
  target_point_with_lane_id.point.pose.position.y = target_point.y();
  target_point_with_lane_id.point.longitudinal_velocity_mps = 0.0;

  // Insert stop pose into path
  modified_path.points.insert(
    modified_path.points.begin() + insert_target_point_idx, target_point_with_lane_id);
  debug_data_.stop_poses.push_back(target_point_with_lane_id.point.pose);

  // insert 0 velocity after target point
  for (size_t j = insert_target_point_idx; j < modified_path.points.size(); ++j) {
    modified_path.points.at(j).point.longitudinal_velocity_mps = 0.0;
  }
  if (target_velocity_point_idx < first_stop_path_point_index_) {
    first_stop_path_point_index_ = target_velocity_point_idx;
    debug_data_.first_stop_pose = target_point_with_lane_id.point.pose;
  }

  // Get stop point and stop factor
  autoware_planning_msgs::msg::StopFactor stop_factor;
  stop_factor.stop_pose = debug_data_.first_stop_pose;
  stop_factor.stop_factor_points =
    std::vector<geometry_msgs::msg::Point>{debug_data_.highest_confidence_traffic_light_point};
  planning_utils::appendStopReason(stop_factor, stop_reason);

  return modified_path;
}

bool TrafficLightModule::hasLampState(
  const autoware_perception_msgs::msg::TrafficLightState & tl_state,
  const uint8_t & lamp_color) const
{
  const auto it_lamp = std::find_if(
    tl_state.lamp_states.begin(), tl_state.lamp_states.end(),
    [&lamp_color](const auto & x) { return x.type == lamp_color; });

  return it_lamp != tl_state.lamp_states.end();
}

bool TrafficLightModule::getExternalTrafficLightState(
  const lanelet::ConstLineStringsOrPolygons3d & traffic_lights,
  autoware_perception_msgs::msg::TrafficLightStateStamped & external_tl_state)
{
  // search traffic light state
  bool found = false;
  std::string reason;
  for (const auto & traffic_light : traffic_lights) {
    // traffic light must be linestrings
    if (!traffic_light.isLineString()) {
      reason = "NotLineString";
      continue;
    }

    const int id = static_cast<lanelet::ConstLineString3d>(traffic_light).id();
    const auto tl_state_stamped = planner_data_->getExternalTrafficLightState(id);
    if (!tl_state_stamped) {
      reason = "TrafficLightStateNotFound";
      continue;
    }

    const auto header = tl_state_stamped->header;
    const auto tl_state = tl_state_stamped->state;
    if (!((clock_->now() - header.stamp).seconds() < planner_param_.external_tl_state_timeout)) {
      reason = "TimeOut";
      continue;
    }

    external_tl_state = *tl_state_stamped;
    found = true;
  }
  if (!found) {
    RCLCPP_DEBUG_THROTTLE(
      logger_, *clock_, 5000 /* ms */,
      "[traffic_light] cannot find external traffic light lamp state (%s).", reason.c_str());
    return false;
  }
  return true;
}

autoware_perception_msgs::msg::TrafficLightStateWithJudge
TrafficLightModule::generateTlStateWithJudgeFromTlState(
  const autoware_perception_msgs::msg::TrafficLightState tl_state) const
{
  autoware_perception_msgs::msg::TrafficLightStateWithJudge tl_state_with_judge;
  tl_state_with_judge.state = tl_state;
  tl_state_with_judge.has_state = true;
  tl_state_with_judge.judge = isTrafficLightStateStop(tl_state)
                                ? autoware_perception_msgs::msg::TrafficLightStateWithJudge::STOP
                                : autoware_perception_msgs::msg::TrafficLightStateWithJudge::GO;
  return tl_state_with_judge;
}
}  // namespace behavior_velocity_planner
