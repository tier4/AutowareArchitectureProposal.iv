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

#include "scene_module/no_stopping_area/scene_no_stopping_area.hpp"

#include "utilization/arc_lane_util.hpp"
#include "utilization/interpolate.hpp"
#include "utilization/util.hpp"

#include <autoware_utils/autoware_utils.hpp>
#include <interpolation/spline_interpolation.hpp>

#include <lanelet2_core/utility/Optional.h>
#include <tf2_eigen/tf2_eigen.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;
bool splineInterpolate(
  const autoware_planning_msgs::msg::PathWithLaneId & input, const double interval,
  autoware_planning_msgs::msg::PathWithLaneId * output, const rclcpp::Logger logger)
{
  *output = input;

  if (input.points.size() <= 1) {
    RCLCPP_WARN(logger, "Do not interpolate because path size is 1.");
    return false;
  }

  static constexpr double ep = 1.0e-8;

  // calc arclength for path
  std::vector<double> base_x;
  std::vector<double> base_y;
  std::vector<double> base_z;
  for (const auto & p : input.points) {
    base_x.push_back(p.point.pose.position.x);
    base_y.push_back(p.point.pose.position.y);
    base_z.push_back(p.point.pose.position.z);
  }
  std::vector<double> base_s = interpolation::calcEuclidDist(base_x, base_y);

  // remove duplicating sample points
  {
    size_t Ns = base_s.size();
    size_t i = 1;
    while (i < Ns) {
      if (std::fabs(base_s[i - 1] - base_s[i]) < ep) {
        base_s.erase(base_s.begin() + i);
        base_x.erase(base_x.begin() + i);
        base_y.erase(base_y.begin() + i);
        base_z.erase(base_z.begin() + i);
        Ns -= 1;
        i -= 1;
      }
      ++i;
    }
  }

  std::vector<double> resampled_s;
  for (double d = 0.0; d < base_s.back() - ep; d += interval) {
    resampled_s.push_back(d);
  }

  // do spline for xy
  const std::vector<double> resampled_x = ::interpolation::slerp(base_s, base_x, resampled_s);
  const std::vector<double> resampled_y = ::interpolation::slerp(base_s, base_y, resampled_s);
  const std::vector<double> resampled_z = ::interpolation::slerp(base_s, base_z, resampled_s);

  // set xy
  output->points.clear();
  for (size_t i = 0; i < resampled_s.size(); i++) {
    autoware_planning_msgs::msg::PathPointWithLaneId p;
    p.point.pose.position.x = resampled_x.at(i);
    p.point.pose.position.y = resampled_y.at(i);
    p.point.pose.position.z = resampled_z.at(i);
    output->points.push_back(p);
  }

  // set yaw
  for (int i = 1; i < static_cast<int>(resampled_s.size()) - 1; i++) {
    auto p = output->points.at(i - 1).point.pose.position;
    auto n = output->points.at(i + 1).point.pose.position;
    double yaw = std::atan2(n.y - p.y, n.x - p.x);
    output->points.at(i).point.pose.orientation = planning_utils::getQuaternionFromYaw(yaw);
  }
  if (output->points.size() > 1) {
    size_t l = resampled_s.size();
    output->points.front().point.pose.orientation = output->points.at(1).point.pose.orientation;
    output->points.back().point.pose.orientation = output->points.at(l - 1).point.pose.orientation;
  }
  return true;
}

NoStoppingAreaModule::NoStoppingAreaModule(
  const int64_t module_id, const lanelet::autoware::NoStoppingArea & no_stopping_area_reg_elem,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  no_stopping_area_reg_elem_(no_stopping_area_reg_elem),
  planner_param_(planner_param)
{
  state_machine_.setState(StateMachine::State::GO);
  state_machine_.setMarginTime(planner_param_.state_clear_time);
}

boost::optional<LineString2d> NoStoppingAreaModule::getStopLineGeometry2d(
  const autoware_planning_msgs::msg::PathWithLaneId & path, const double stop_line_margin) const
{
  // get stop line from map
  {
    const auto & stop_line = no_stopping_area_reg_elem_.stopLine();
    if (stop_line) {
      return planning_utils::extendLine(
        stop_line.value()[0], stop_line.value()[1], planner_data_->stop_line_extend_length);
    }
  }
  // auto gen stop line
  {
    LineString2d stop_line;
    /**
     * @brief auto gen no stopping area stop line from area polygon if stop line is not set
     *        ---------------
     * ------col-------------|--> ego path
     *        |     Area     |
     *        ---------------
     **/

    for (const auto & no_stopping_area : no_stopping_area_reg_elem_.noStoppingAreas()) {
      const auto & area_poly = lanelet::utils::to2D(no_stopping_area).basicPolygon();
      lanelet::BasicLineString2d path_line;
      for (size_t i = 0; i < path.points.size() - 1; ++i) {
        const auto p0 = path.points.at(i).point.pose.position;
        const auto p1 = path.points.at(i + 1).point.pose.position;
        const LineString2d line{{p0.x, p0.y}, {p1.x, p1.y}};
        std::vector<Point2d> collision_points;
        bg::intersection(area_poly, line, collision_points);
        if (collision_points.empty()) {
          continue;
        }
        const double yaw = autoware_utils::calcAzimuthAngle(p0, p1);
        if (!collision_points.empty()) {
          geometry_msgs::msg::Point left_point;
          const double w = planner_data_->vehicle_info_.vehicle_width_m;
          const double l = stop_line_margin;
          stop_line.emplace_back(
            -l * std::cos(yaw) + collision_points.front().x() + w * std::cos(yaw + M_PI_2),
            collision_points.front().y() + w * std::sin(yaw + M_PI_2));
          stop_line.emplace_back(
            -l * std::cos(yaw) + collision_points.front().x() + w * std::cos(yaw - M_PI_2),
            collision_points.front().y() + w * std::sin(yaw - M_PI_2));
          return stop_line;
        }
      }
    }
  }
  return {};
}

bool NoStoppingAreaModule::modifyPathVelocity(
  autoware_planning_msgs::msg::PathWithLaneId * path,
  autoware_planning_msgs::msg::StopReason * stop_reason)
{
  // Store original path
  const auto original_path = *path;
  const auto & dynamic_obj_arr_ptr = planner_data_->dynamic_objects;
  const auto & current_pose = planner_data_->current_pose;
  if (path->points.size() <= 2) {
    return true;
  }
  // Reset data
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  *stop_reason =
    planning_utils::initializeStopReason(autoware_planning_msgs::msg::StopReason::NO_STOPPING_AREA);
  // Get stop line geometry
  const auto stop_line = getStopLineGeometry2d(original_path, planner_param_.stop_line_margin);
  if (!stop_line) {
    return true;
  }
  const auto stop_point =
    createTargetPoint(original_path, stop_line.value(), planner_param_.stop_margin);
  if (!stop_point) {
    return true;
  }
  const auto & stop_pose = stop_point->second;
  if (isOverDeadLine(original_path, current_pose.pose, stop_pose)) {
    // ego can't stop in front of no stopping area -> GO or OR
    state_machine_.setState(StateMachine::State::GO);
    return true;
  }
  const auto & vi = planner_data_->vehicle_info_;
  const double margin = planner_param_.stop_line_margin;
  const double ego_space_in_front_of_stuck_vehicle =
    margin + vi.vehicle_length_m + planner_param_.stuck_vehicle_front_margin;
  const Polygon2d stuck_vehicle_detect_area = generateEgoNoStoppingAreaLanePolygon(
    *path, current_pose.pose, ego_space_in_front_of_stuck_vehicle,
    planner_param_.detection_area_length);
  const double ego_space_in_front_of_stop_line =
    margin + planner_param_.stop_margin + vi.rear_overhang_m;
  const Polygon2d stop_line_detect_area = generateEgoNoStoppingAreaLanePolygon(
    *path, current_pose.pose, ego_space_in_front_of_stop_line,
    planner_param_.detection_area_length);
  if (stuck_vehicle_detect_area.outer().empty() && stop_line_detect_area.outer().empty()) {
    return true;
  }
  debug_data_.stuck_vehicle_detect_area = toGeomMsg(stuck_vehicle_detect_area);
  debug_data_.stop_line_detect_area = toGeomMsg(stop_line_detect_area);
  // Find stuck vehicle in no stopping area
  const bool is_entry_prohibited_by_stuck_vehicle =
    checkStuckVehiclesInNoStoppingArea(stuck_vehicle_detect_area, dynamic_obj_arr_ptr);
  // Find stop line in no stopping area
  const bool is_entry_prohibited_by_stop_line =
    checkStopLinesInNoStoppingArea(*path, stop_line_detect_area);
  const bool is_entry_prohibited =
    is_entry_prohibited_by_stuck_vehicle || is_entry_prohibited_by_stop_line;
  if (!isStoppable(current_pose.pose, stop_point->second)) {
    state_machine_.setState(StateMachine::State::GO);
    return false;
  } else {
    state_machine_.setStateWithMarginTime(
      is_entry_prohibited ? StateMachine::State::STOP : StateMachine::State::GO,
      logger_.get_child("state_machine"), *clock_);
  }

  if (state_machine_.getState() == StateMachine::State::STOP) {
    // ----------------stop reason and stop point--------------------------
    insertStopPoint(*path, *stop_point);
    // For virtual wall
    debug_data_.stop_poses.push_back(stop_pose);

    // Create StopReason
    {
      autoware_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = stop_point->second;
      stop_factor.stop_factor_points = debug_data_.stuck_points;
      planning_utils::appendStopReason(stop_factor, stop_reason);
    }

    // Create legacy StopReason
    {
      const auto insert_idx = stop_point->first + 1;
      if (
        !first_stop_path_point_index_ ||
        static_cast<int>(insert_idx) < first_stop_path_point_index_) {
        debug_data_.first_stop_pose = stop_pose;
        first_stop_path_point_index_ = static_cast<int>(insert_idx);
      }
    }
  } else if (state_machine_.getState() == StateMachine::State::GO) {
    // reset pass judge if current state is go
    is_stoppable_ = true;
    pass_judged_ = false;
  }
  return true;
}

bool NoStoppingAreaModule::checkStuckVehiclesInNoStoppingArea(
  const Polygon2d & poly,
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr & dynamic_obj_arr_ptr)
{
  // stuck points by dynamic objects
  for (const auto & object : dynamic_obj_arr_ptr->objects) {
    if (!isTargetStuckVehicleType(object)) {
      continue;  // not target vehicle type
    }
    const auto obj_v = std::fabs(object.state.twist_covariance.twist.linear.x);
    if (obj_v > planner_param_.stuck_vehicle_vel_thr) {
      continue;  // not stop vehicle
    }
    // check if the footprint is in the stuck detect area
    const Polygon2d obj_footprint = planning_utils::toFootprintPolygon(object);
    const bool is_in_stuck_area = !bg::disjoint(obj_footprint, poly);
    if (is_in_stuck_area) {
      RCLCPP_DEBUG(logger_, "stuck vehicle found.");
      for (const auto p : obj_footprint.outer()) {
        geometry_msgs::msg::Point point;
        point.x = p.x();
        point.y = p.y();
        point.z = 0.0;
        debug_data_.stuck_points.emplace_back(point);
      }
      return true;
    }
  }
  return false;
}
bool NoStoppingAreaModule::checkStopLinesInNoStoppingArea(
  const autoware_planning_msgs::msg::PathWithLaneId & path, const Polygon2d & poly)
{
  const double stop_vel = std::numeric_limits<float>::min();
  // stuck points by stop line
  for (size_t i = 0; i < path.points.size() - 1; ++i) {
    const auto p0 = path.points.at(i).point.pose.position;
    const auto p1 = path.points.at(i + 1).point.pose.position;
    const auto v0 = path.points.at(i).point.twist.linear.x;
    const auto v1 = path.points.at(i + 1).point.twist.linear.x;
    if (v0 > stop_vel && v1 > stop_vel) {
      continue;
    }
    const LineString2d line{{p0.x, p0.y}, {p1.x, p1.y}};
    std::vector<Point2d> collision_points;
    bg::intersection(poly, line, collision_points);
    if (!collision_points.empty()) {
      geometry_msgs::msg::Point point;
      point.x = collision_points.front().x();
      point.y = collision_points.front().y();
      point.z = 0.0;
      debug_data_.stuck_points.emplace_back(point);
      return true;
    }
  }
  return false;
}

Polygon2d NoStoppingAreaModule::generateEgoNoStoppingAreaLanePolygon(
  const autoware_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Pose & ego_pose, const double margin, const double extra_dist) const
{
  Polygon2d ego_area;  // open polygon
  double dist_from_start_sum = 0.0;
  const double interpolation_interval = 0.5;
  bool is_in_area = false;
  autoware_planning_msgs::msg::PathWithLaneId interpolated_path;
  if (!splineInterpolate(path, interpolation_interval, &interpolated_path, logger_)) {
    return ego_area;
  }
  auto & pp = interpolated_path.points;
  /* calc closest index */
  int closest_idx = -1;
  if (!planning_utils::calcClosestIndex<autoware_planning_msgs::msg::PathWithLaneId>(
        interpolated_path, ego_pose, closest_idx)) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(logger_, *clock_, 1000 /* ms */, "calcClosestIndex fail");
    return ego_area;
  }
  const int num_ignore_nearest = 1;  // Do not consider nearest lane polygon
  size_t ego_area_start_idx = closest_idx + num_ignore_nearest;
  size_t ego_area_end_idx = ego_area_start_idx;
  // return if area size is not intentional
  if (no_stopping_area_reg_elem_.noStoppingAreas().size() != 1) {
    return ego_area;
  }
  const auto no_stopping_area = no_stopping_area_reg_elem_.noStoppingAreas().front();
  for (size_t i = closest_idx + num_ignore_nearest; i < pp.size() - 1; ++i) {
    dist_from_start_sum += planning_utils::calcDist2d(pp.at(i), pp.at(i - 1));
    const auto & p = pp.at(i).point.pose.position;
    if (bg::within(Point2d{p.x, p.y}, lanelet::utils::to2D(no_stopping_area).basicPolygon())) {
      is_in_area = true;
      break;
    }
    if (dist_from_start_sum > extra_dist) {
      return ego_area;
    }
    ++ego_area_start_idx;
  }
  if (ego_area_start_idx > num_ignore_nearest) {
    ego_area_start_idx--;
  }
  if (!is_in_area) {
    return ego_area;
  }
  double dist_from_area_sum = 0.0;
  // decide end idx with extract distance
  ego_area_end_idx = ego_area_start_idx;
  for (size_t i = ego_area_start_idx; i < pp.size() - 1; ++i) {
    dist_from_start_sum += planning_utils::calcDist2d(pp.at(i), pp.at(i - 1));
    const auto & p = pp.at(i).point.pose.position;
    if (!bg::within(Point2d{p.x, p.y}, lanelet::utils::to2D(no_stopping_area).basicPolygon())) {
      dist_from_area_sum += planning_utils::calcDist2d(pp.at(i), pp.at(i - 1));
    }
    if (dist_from_start_sum > extra_dist || dist_from_area_sum > margin) {
      break;
    }
    ++ego_area_end_idx;
  }

  const auto width = planner_param_.path_expand_width;
  ego_area = planning_utils::generatePathPolygon(
    interpolated_path, ego_area_start_idx, ego_area_end_idx, width);
  return ego_area;
}

bool NoStoppingAreaModule::isTargetStuckVehicleType(
  const autoware_perception_msgs::msg::DynamicObject & object) const
{
  if (
    object.semantic.type == autoware_perception_msgs::msg::Semantic::CAR ||
    object.semantic.type == autoware_perception_msgs::msg::Semantic::BUS ||
    object.semantic.type == autoware_perception_msgs::msg::Semantic::TRUCK ||
    object.semantic.type == autoware_perception_msgs::msg::Semantic::MOTORBIKE) {
    return true;
  }
  return false;
}

bool NoStoppingAreaModule::isOverDeadLine(
  const autoware_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Pose & self_pose, const geometry_msgs::msg::Pose & line_pose) const
{
  return autoware_utils::calcSignedArcLength(path.points, self_pose.position, line_pose.position) +
           planner_param_.dead_line_margin <
         0.0;
}

bool NoStoppingAreaModule::isStoppable(
  const geometry_msgs::msg::Pose & self_pose, const geometry_msgs::msg::Pose & line_pose) const
{
  // get vehicle info and compute pass_judge_line_distance
  const auto current_velocity = planner_data_->current_velocity->twist.linear.x;
  const auto current_acceleration = planner_data_->current_accel.get();
  const double max_acc = planner_data_->max_stop_acceleration_threshold;
  const double max_jerk = planner_data_->max_stop_jerk_threshold;
  const double delay_response_time = planner_data_->delay_response_time;
  if (!planner_data_->current_accel) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "[no stopping area] empty current acc! check current vel has been received.");
    return false;
  }
  const double stoppable_distance = planning_utils::calcJudgeLineDistWithJerkLimit(
    current_velocity, current_acceleration, max_acc, max_jerk, delay_response_time);
  const double signed_arc_length =
    arc_lane_utils::calcSignedDistance(self_pose, line_pose.position);
  const bool distance_stoppable = stoppable_distance < signed_arc_length;
  const bool slow_velocity = planner_data_->current_velocity->twist.linear.x < 2.0;
  // ego vehicle is high speed and can't stop before stop line -> GO
  const bool not_stoppable = !distance_stoppable && !slow_velocity;
  // stoppable or not is judged only once
  RCLCPP_DEBUG(
    logger_, "stoppable_dist: %lf signed_arc_length: %lf", stoppable_distance, signed_arc_length);
  if (!distance_stoppable && !pass_judged_) {
    pass_judged_ = true;
    // can't stop using maximum brake consider jerk limit
    if (not_stoppable) {
      // pass through
      is_stoppable_ = false;
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000, "[NoStoppingArea] can't stop in front of no stopping area");
    } else {
      is_stoppable_ = true;
    }
  }
  return is_stoppable_;
}

void NoStoppingAreaModule::insertStopPoint(
  autoware_planning_msgs::msg::PathWithLaneId & path, const PathIndexWithPose & stop_point)
{
  const auto insert_idx = stop_point.first + 1;
  const auto stop_pose = stop_point.second;

  // To PathPointWithLaneId
  autoware_planning_msgs::msg::PathPointWithLaneId stop_point_with_lane_id;
  stop_point_with_lane_id = path.points.at(insert_idx);
  stop_point_with_lane_id.point.pose = stop_pose;
  stop_point_with_lane_id.point.twist.linear.x = 0.0;

  // Insert stop point
  path.points.insert(path.points.begin() + insert_idx, stop_point_with_lane_id);

  // Insert 0 velocity after stop point
  for (size_t j = insert_idx; j < path.points.size(); ++j) {
    path.points.at(j).point.twist.linear.x = 0.0;
  }
}

boost::optional<PathIndexWithPose> NoStoppingAreaModule::createTargetPoint(
  const autoware_planning_msgs::msg::PathWithLaneId & path, const LineString2d & stop_line,
  const double margin) const
{
  // Find collision segment
  const auto collision_segment = arc_lane_utils::findCollisionSegment(path, stop_line);
  if (!collision_segment) {
    // No collision
    return {};
  }

  // Calculate offset length from stop line
  // Use '-' to make the positive direction is forward
  const double offset_length = -(margin + planner_data_->vehicle_info_.max_longitudinal_offset_m);

  // Find offset segment
  const auto offset_segment =
    arc_lane_utils::findOffsetSegment(path, *collision_segment, offset_length);
  if (!offset_segment) {
    // No enough path length
    return {};
  }

  const auto front_idx = offset_segment->first;
  const auto target_pose = arc_lane_utils::calcTargetPose(path, *offset_segment);

  return std::make_pair(front_idx, target_pose);
}

}  // namespace behavior_velocity_planner
