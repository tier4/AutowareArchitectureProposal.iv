// Copyright 2019 Autoware Foundation
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

#include "lane_change_planner/route_handler.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include "lane_change_planner/utilities.hpp"

#include "autoware_planning_msgs/msg/path_with_lane_id.hpp"
#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_extension/utility/utilities.hpp"

#include "lanelet2_core/geometry/Lanelet.h"
#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_core/primitives/LaneletSequence.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using autoware_planning_msgs::msg::PathPointWithLaneId;
using autoware_planning_msgs::msg::PathWithLaneId;
using lanelet::utils::to2D;
namespace
{
template<typename T>
bool exists(const std::vector<T> & vectors, const T & item)
{
  for (const auto & i : vectors) {
    if (i == item) {
      return true;
    }
  }
  return false;
}

bool isRouteLooped(const autoware_planning_msgs::msg::Route & route_msg)
{
  const auto & route_sections = route_msg.route_sections;
  for (std::size_t i = 0; i < route_sections.size(); i++) {
    const auto & route_section = route_sections.at(i);
    for (const auto & lane_id : route_section.lane_ids) {
      for (std::size_t j = i + 1; j < route_sections.size(); j++) {
        const auto & future_route_section = route_sections.at(j);
        if (exists(future_route_section.lane_ids, lane_id)) {
          return true;
        }
      }
    }
  }
  return false;
}

PathWithLaneId combineReferencePath(
  const PathWithLaneId path1, const PathWithLaneId path2, const double interval,
  const size_t N_sample, const rclcpp::Logger & logger)
{
  PathWithLaneId path;
  path.points.insert(path.points.end(), path1.points.begin(), path1.points.end());

  const double dx =
    path2.points.front().point.pose.position.x - path1.points.back().point.pose.position.x;
  const double dy =
    path2.points.front().point.pose.position.y - path1.points.back().point.pose.position.y;
  const double ds = std::hypot(dx, dy);
  if (interval < ds) {
    // calculate samples
    std::vector<double> base_x;
    std::vector<double> base_y;
    std::vector<double> base_z;
    int n_sample_path1 = 0;

    for (size_t i = 0; i < N_sample; ++i) {
      const int idx = static_cast<int>(path1.points.size()) - N_sample + i;
      if (idx < 0) {
        continue;
      }
      base_x.push_back(path1.points.at(idx).point.pose.position.x);
      base_y.push_back(path1.points.at(idx).point.pose.position.y);
      base_z.push_back(path1.points.at(idx).point.pose.position.z);
      n_sample_path1++;
    }
    int n_sample_path2 = 0;
    for (size_t i = 0; i < N_sample; ++i) {
      if (i >= path2.points.size()) {
        continue;
      }
      base_x.push_back(path2.points.at(i).point.pose.position.x);
      base_y.push_back(path2.points.at(i).point.pose.position.y);
      base_z.push_back(path2.points.at(i).point.pose.position.z);
      n_sample_path2++;
    }

    std::vector<double> base_s = {0.0};
    for (size_t i = 1; i < base_x.size(); ++i) {
      const double base_dx = base_x.at(i) - base_x.at(i - 1);
      const double base_dy = base_y.at(i) - base_y.at(i - 1);
      base_s.push_back(base_s.at(i - 1) + std::hypot(base_dx, base_dy));
    }

    // calculate query
    std::vector<double> inner_s;
    for (double d = (base_s.at(n_sample_path1 - 1) + interval); d < base_s.at(n_sample_path1);
      d += interval)
    {
      inner_s.push_back(d);
    }

    lane_change_planner::util::SplineInterpolate spline(logger);
    std::vector<PathPointWithLaneId> inner_points;
    std::vector<double> inner_x;
    std::vector<double> inner_y;
    std::vector<double> inner_z;
    if (
      spline.interpolate(base_s, base_x, inner_s, inner_x) &&
      spline.interpolate(base_s, base_y, inner_s, inner_y) &&
      spline.interpolate(base_s, base_z, inner_s, inner_z))
    {
      // set position and other data
      for (size_t i = 0; i < inner_s.size(); ++i) {
        PathPointWithLaneId inner_point;
        inner_point.lane_ids.insert(
          inner_point.lane_ids.end(), path1.points.back().lane_ids.begin(),
          path1.points.back().lane_ids.end());
        inner_point.lane_ids.insert(
          inner_point.lane_ids.end(), path2.points.front().lane_ids.begin(),
          path2.points.front().lane_ids.end());
        inner_point.point.type = path1.points.back().point.type;
        inner_point.point.twist = path1.points.back().point.twist;
        inner_point.point.pose.position.x = inner_x.at(i);
        inner_point.point.pose.position.y = inner_y.at(i);
        inner_point.point.pose.position.z = inner_z.at(i);
        inner_points.push_back(inner_point);
      }

      // set yaw
      for (size_t i = 0; i < inner_points.size(); ++i) {
        geometry_msgs::msg::Point prev, next;
        if (i == 0) {
          prev = path1.points.back().point.pose.position;
        } else {
          prev = inner_points.at(i - 1).point.pose.position;
        }
        if (i == (inner_s.size() - 1)) {
          next = path2.points.front().point.pose.position;
        } else {
          next = inner_points.at(i + 1).point.pose.position;
        }

        double yaw = std::atan2(next.y - prev.y, next.x - prev.x);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        inner_points.at(i).point.pose.orientation = tf2::toMsg(q);
      }

      path.points.insert(path.points.end(), inner_points.begin(), inner_points.end());

    } else {
      RCLCPP_WARN(
        logger, "[LaneChageModule::splineInterpolate] spline interpolation failed.");
    }
  }
  path.points.insert(path.points.end(), path2.points.begin(), path2.points.end());
  return path;
}

lanelet::ConstPoint3d get3DPointFrom2DArcLength(
  const lanelet::ConstLanelets & lanelet_sequence, const double s)
{
  double accumulated_distance2d = 0;
  for (const auto & llt : lanelet_sequence) {
    const auto & centerline = llt.centerline();
    lanelet::ConstPoint3d prev_pt;
    if (!centerline.empty()) {
      prev_pt = centerline.front();
    }
    for (const auto & pt : centerline) {
      double distance2d = lanelet::geometry::distance2d(to2D(prev_pt), to2D(pt));
      if (accumulated_distance2d + distance2d > s) {
        double ratio = (s - accumulated_distance2d) / distance2d;
        auto interpolated_pt = prev_pt.basicPoint() * (1 - ratio) + pt.basicPoint() * ratio;
        return lanelet::ConstPoint3d(
          lanelet::InvalId, interpolated_pt.x(), interpolated_pt.y(), interpolated_pt.z());
      }
      accumulated_distance2d += distance2d;
      prev_pt = pt;
    }
  }
  return lanelet::ConstPoint3d();
}

}  // namespace

namespace lane_change_planner
{
RouteHandler::RouteHandler(const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr & clock)
: is_map_msg_ready_(false),
  is_route_msg_ready_(false),
  is_handler_ready_(false),
  logger_(logger),
  clock_(clock)
{
}

void RouteHandler::mapCallback(const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);

  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  const auto pedestrian_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  lanelet::routing::RoutingGraphConstPtr vehicle_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules);
  lanelet::routing::RoutingGraphConstPtr pedestrian_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *pedestrian_rules);
  lanelet::routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});
  overall_graphs_ptr_ =
    std::make_shared<const lanelet::routing::RoutingGraphContainer>(overall_graphs);

  is_map_msg_ready_ = true;
  is_handler_ready_ = false;

  setRouteLanelets();
}

void RouteHandler::routeCallback(const autoware_planning_msgs::msg::Route::ConstSharedPtr route_msg)
{
  if (!isRouteLooped(*route_msg)) {
    route_msg_ = *route_msg;
    is_route_msg_ready_ = true;
    is_handler_ready_ = false;
    setRouteLanelets();
  } else {
    RCLCPP_ERROR(
      logger_,
      "Loop detected within route! Currently, no loop is allowed for route! Using previous route");
  }
}

bool RouteHandler::isHandlerReady() {return is_handler_ready_;}

void RouteHandler::setRouteLanelets()
{
  if (!is_route_msg_ready_ || !is_map_msg_ready_) {
    return;
  }
  route_lanelets_.clear();
  preferred_lanelets_.clear();
  for (const auto & route_section : route_msg_.route_sections) {
    for (const auto & id : route_section.lane_ids) {
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      route_lanelets_.push_back(llt);
      if (id == route_section.preferred_lane_id) {
        preferred_lanelets_.push_back(llt);
      }
    }
  }
  goal_lanelets_.clear();
  start_lanelets_.clear();
  if (!route_msg_.route_sections.empty()) {
    for (const auto & id : route_msg_.route_sections.back().lane_ids) {
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      goal_lanelets_.push_back(llt);
    }
    for (const auto & id : route_msg_.route_sections.front().lane_ids) {
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      start_lanelets_.push_back(llt);
    }
  }
  is_handler_ready_ = true;
}

std::vector<lanelet::ConstLanelet> RouteHandler::getLanesAfterGoal(
  const double vehicle_length) const
{
  lanelet::ConstLanelet goal_lanelet;
  if (getGoalLanelet(&goal_lanelet)) {
    const double min_succeeding_length = vehicle_length * 2;
    const auto succeeding_lanes_vec = lanelet::utils::query::getSucceedingLaneletSequences(
      routing_graph_ptr_, goal_lanelet, min_succeeding_length);
    if (succeeding_lanes_vec.empty()) {
      return std::vector<lanelet::ConstLanelet>{};
    } else {
      return succeeding_lanes_vec.front();
    }
  } else {
    return std::vector<lanelet::ConstLanelet>{};
  }
}

lanelet::ConstLanelets RouteHandler::getRouteLanelets() const {return route_lanelets_;}

geometry_msgs::msg::Pose RouteHandler::getGoalPose() const {return route_msg_.goal_pose;}

lanelet::Id RouteHandler::getGoalLaneId() const
{
  if (route_msg_.route_sections.empty()) {
    return lanelet::InvalId;
  } else {
    return route_msg_.route_sections.back().preferred_lane_id;
  }
}

bool RouteHandler::getGoalLanelet(lanelet::ConstLanelet * goal_lanelet) const
{
  const lanelet::Id goal_lane_id = getGoalLaneId();
  for (const auto & llt : route_lanelets_) {
    if (llt.id() == goal_lane_id) {
      *goal_lanelet = llt;
      return true;
    }
  }
  return false;
}

bool RouteHandler::isInGoalRouteSection(const lanelet::ConstLanelet & lanelet) const
{
  if (route_msg_.route_sections.empty()) {
    return false;
  } else {
    return exists(route_msg_.route_sections.back().lane_ids, lanelet.id());
  }
}

lanelet::ConstLanelets RouteHandler::getLaneletsFromIds(const std::vector<uint64_t> ids) const
{
  lanelet::ConstLanelets lanelets;
  for (const auto & id : ids) {
    lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(id));
  }
  return lanelets;
}

bool RouteHandler::isDeadEndLanelet(const lanelet::ConstLanelet & lanelet) const
{
  for (const auto & route_section : route_msg_.route_sections) {
    if (exists(route_section.continued_lane_ids, lanelet.id())) {
      return false;
    }
  }
  return true;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence_forward;
  }

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  while (rclcpp::ok() && length < min_length) {
    lanelet::ConstLanelet next_lanelet;
    if (!getNextLaneletWithinRoute(current_lanelet, &next_lanelet)) {
      break;
    }
    lanelet_sequence_forward.push_back(next_lanelet);
    current_lanelet = next_lanelet;
    length += boost::geometry::length(next_lanelet.centerline().basicLineString());
  }

  return lanelet_sequence_forward;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence_backward;
  }

  lanelet::ConstLanelet current_lanelet = lanelet;
  double length = 0;

  while (rclcpp::ok() && length < min_length) {
    lanelet::ConstLanelet prev_lanelet;
    if (!getPreviousLaneletWithinRoute(current_lanelet, &prev_lanelet)) {
      break;
    }
    lanelet_sequence_backward.push_back(prev_lanelet);
    length += boost::geometry::length(prev_lanelet.centerline().basicLineString());
    current_lanelet = prev_lanelet;
  }

  std::reverse(lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());
  return lanelet_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequence(const lanelet::ConstLanelet & lanelet) const
{
  geometry_msgs::msg::Pose tmp_pose;
  tmp_pose.orientation.w = 1;
  if (!lanelet.centerline().empty()) {
    tmp_pose.position = lanelet::utils::conversion::toGeomMsgPt(lanelet.centerline().front());
  }
  constexpr double double_max = std::numeric_limits<double>::max();
  return getLaneletSequence(lanelet, tmp_pose, double_max, double_max);
}

lanelet::ConstLanelets RouteHandler::getLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Pose & current_pose,
  const double backward_distance, const double forward_distance) const
{
  lanelet::ConstLanelets lanelet_sequence;
  lanelet::ConstLanelets lanelet_sequence_backward;
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence;
  }

  lanelet_sequence_forward = getLaneletSequenceAfter(lanelet, forward_distance);

  const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
  if (arc_coordinate.length < backward_distance) {
    lanelet_sequence_backward = getLaneletSequenceUpTo(lanelet, backward_distance);
  }

  // loop check
  if (!lanelet_sequence_forward.empty() && !lanelet_sequence_backward.empty()) {
    if (lanelet_sequence_backward.back().id() == lanelet_sequence_forward.front().id()) {
      return lanelet_sequence_forward;
    }
  }
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());
  lanelet_sequence.push_back(lanelet);
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_forward.begin(), lanelet_sequence_forward.end());

  return lanelet_sequence;
}

bool RouteHandler::getClosestLaneletWithinRoute(
  const geometry_msgs::msg::Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const
{
  return lanelet::utils::query::getClosestLanelet(route_lanelets_, search_pose, closest_lanelet);
}

bool RouteHandler::getNextLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * next_lanelet) const
{
  if (exists(goal_lanelets_, lanelet)) {
    return false;
  }
  lanelet::ConstLanelets following_lanelets = routing_graph_ptr_->following(lanelet);
  for (const auto & llt : following_lanelets) {
    if (exists(route_lanelets_, llt)) {
      *next_lanelet = llt;
      return true;
    }
  }
  return false;
}

bool RouteHandler::getPreviousLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * prev_lanelet) const
{
  if (exists(start_lanelets_, lanelet)) {
    return false;
  }
  lanelet::ConstLanelets previous_lanelets = routing_graph_ptr_->previous(lanelet);
  for (const auto & llt : previous_lanelets) {
    if (exists(route_lanelets_, llt)) {
      *prev_lanelet = llt;
      return true;
    }
  }
  return false;
}

bool RouteHandler::getRightLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * right_lanelet)
{
  auto opt_right_lanelet = routing_graph_ptr_->right(lanelet);
  if (!!opt_right_lanelet) {
    *right_lanelet = opt_right_lanelet.get();
    return exists(route_lanelets_, *right_lanelet);
  } else {
    return false;
  }
}
bool RouteHandler::getLeftLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * left_lanelet)
{
  auto opt_left_lanelet = routing_graph_ptr_->left(lanelet);
  if (!!opt_left_lanelet) {
    *left_lanelet = opt_left_lanelet.get();
    return exists(route_lanelets_, *left_lanelet);
  } else {
    return false;
  }
}

bool RouteHandler::getLaneChangeTarget(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * target_lanelet) const
{
  int num = getNumLaneToPreferredLane(lanelet);
  if (num == 0) {
    *target_lanelet = lanelet;
    return false;
  }
  if (num < 0) {
    auto right_lanelet = (!!routing_graph_ptr_->right(lanelet)) ?
      routing_graph_ptr_->right(lanelet) :
      routing_graph_ptr_->adjacentRight(lanelet);
    *target_lanelet = right_lanelet.get();
  }

  if (num > 0) {
    auto left_lanelet = (!!routing_graph_ptr_->left(lanelet)) ?
      routing_graph_ptr_->left(lanelet) :
      routing_graph_ptr_->adjacentLeft(lanelet);
    *target_lanelet = left_lanelet.get();
  }
  return true;
}

lanelet::ConstLanelets RouteHandler::getClosestLaneletSequence(
  const geometry_msgs::msg::Pose & pose) const
{
  lanelet::ConstLanelet lanelet;
  lanelet::ConstLanelets empty_lanelets;
  if (!getClosestLaneletWithinRoute(pose, &lanelet)) {
    return empty_lanelets;
  }
  return getLaneletSequence(lanelet);
}

int RouteHandler::getNumLaneToPreferredLane(const lanelet::ConstLanelet & lanelet) const
{
  int num = 0;
  if (exists(preferred_lanelets_, lanelet)) {
    return num;
  }
  const auto & right_lanes =
    lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, lanelet);
  for (const auto & right : right_lanes) {
    num--;
    if (exists(preferred_lanelets_, right)) {
      return num;
    }
  }
  const auto & left_lanes = lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, lanelet);
  num = 0;
  for (const auto & left : left_lanes) {
    num++;
    if (exists(preferred_lanelets_, left)) {
      return num;
    }
  }
  return 0;
}

bool RouteHandler::isInPreferredLane(const geometry_msgs::msg::PoseStamped & pose) const
{
  lanelet::ConstLanelet lanelet;
  if (!getClosestLaneletWithinRoute(pose.pose, &lanelet)) {
    return false;
  }
  return exists(preferred_lanelets_, lanelet);
}
bool RouteHandler::isInTargetLane(
  const geometry_msgs::msg::PoseStamped & pose, const lanelet::ConstLanelets & target) const
{
  lanelet::ConstLanelet lanelet;
  if (!getClosestLaneletWithinRoute(pose.pose, &lanelet)) {
    return false;
  }
  return exists(target, lanelet);
}

PathWithLaneId RouteHandler::getReferencePath(
  const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::msg::Pose & pose,
  const double backward_path_length, const double forward_path_length,
  const double minimum_lane_change_length) const
{
  PathWithLaneId reference_path;

  if (lanelet_sequence.empty()) {
    return reference_path;
  }

  const auto arc_coordinates = lanelet::utils::getArcCoordinates(lanelet_sequence, pose);
  const double s = arc_coordinates.length;
  const double s_backward = std::max(0., s - backward_path_length);
  double s_forward = s + forward_path_length;

  constexpr double buffer = 1.0;  // buffer for min_lane_change_length
  const int num_lane_change = std::abs(getNumLaneToPreferredLane(lanelet_sequence.back()));
  const double lane_length = lanelet::utils::getLaneletLength2d(lanelet_sequence);
  const double lane_change_buffer = num_lane_change * (minimum_lane_change_length + buffer);

  if (isDeadEndLanelet(lanelet_sequence.back())) {
    s_forward = std::min(s_forward, lane_length - lane_change_buffer);
  }

  if (isInGoalRouteSection(lanelet_sequence.back())) {
    const auto goal_arc_coordinates =
      lanelet::utils::getArcCoordinates(lanelet_sequence, getGoalPose());
    s_forward = std::min(s_forward, goal_arc_coordinates.length - lane_change_buffer);
  }

  return getReferencePath(lanelet_sequence, s_backward, s_forward, true);
}

PathWithLaneId RouteHandler::getReferencePath(
  const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end,
  bool use_exact) const
{
  PathWithLaneId reference_path;
  double s = 0;

  for (const auto & llt : lanelet_sequence) {
    lanelet::traffic_rules::SpeedLimitInformation limit = traffic_rules_ptr_->speedLimit(llt);
    const lanelet::ConstLineString3d centerline = llt.centerline();
    for (size_t i = 0; i < centerline.size(); i++) {
      const lanelet::ConstPoint3d pt = centerline[i];
      lanelet::ConstPoint3d next_pt =
        (i + 1 < centerline.size()) ? centerline[i + 1] : centerline[i];
      double distance = lanelet::geometry::distance2d(to2D(pt), to2D(next_pt));

      if (s < s_start && s + distance > s_start) {
        if (use_exact) {
          const auto tmp_p = get3DPointFrom2DArcLength(lanelet_sequence, s_start);
          PathPointWithLaneId point;
          point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(tmp_p);
          point.lane_ids.push_back(llt.id());
          point.point.twist.linear.x = limit.speedLimit.value();
          reference_path.points.push_back(point);
        } else {
          PathPointWithLaneId point;
          point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(pt);
          point.lane_ids.push_back(llt.id());
          point.point.twist.linear.x = limit.speedLimit.value();
          reference_path.points.push_back(point);
        }
      }

      if (s >= s_start && s <= s_end) {
        PathPointWithLaneId point;
        point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(pt);
        point.lane_ids.push_back(llt.id());
        point.point.twist.linear.x = limit.speedLimit.value();
        reference_path.points.push_back(point);
      }
      if (s < s_end && s + distance > s_end) {
        if (use_exact) {
          const auto tmp_p = get3DPointFrom2DArcLength(lanelet_sequence, s_end);
          PathPointWithLaneId point;
          point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(tmp_p);
          point.lane_ids.push_back(llt.id());
          point.point.twist.linear.x = limit.speedLimit.value();
          reference_path.points.push_back(point);
        } else {
          PathPointWithLaneId point;
          point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(next_pt);
          point.lane_ids.push_back(llt.id());
          point.point.twist.linear.x = limit.speedLimit.value();
          reference_path.points.push_back(point);
        }
      }
      s += distance;
    }
  }

  reference_path = util::removeOverlappingPoints(reference_path);

  // set angle
  for (size_t i = 0; i < reference_path.points.size(); i++) {
    double angle = 0;
    auto & pt = reference_path.points.at(i);
    if (i + 1 < reference_path.points.size()) {
      auto next_pt = reference_path.points.at(i + 1);
      angle = std::atan2(
        next_pt.point.pose.position.y - pt.point.pose.position.y,
        next_pt.point.pose.position.x - pt.point.pose.position.x);
    } else if (i != 0) {
      auto prev_pt = reference_path.points.at(i - 1);
      auto pt = reference_path.points.at(i);
      angle = std::atan2(
        pt.point.pose.position.y - prev_pt.point.pose.position.y,
        pt.point.pose.position.x - prev_pt.point.pose.position.x);
    }
    tf2::Quaternion yaw_quat;
    yaw_quat.setRPY(0, 0, angle);
    pt.point.pose.orientation = tf2::toMsg(yaw_quat);
  }

  return reference_path;
}

PathWithLaneId RouteHandler::updatePathTwist(const PathWithLaneId & path) const
{
  PathWithLaneId updated_path = path;
  for (auto & point : updated_path.points) {
    const auto id = point.lane_ids.at(0);
    const auto llt = lanelet_map_ptr_->laneletLayer.get(id);
    lanelet::traffic_rules::SpeedLimitInformation limit = traffic_rules_ptr_->speedLimit(llt);
    point.point.twist.linear.x = limit.speedLimit.value();
  }
  return updated_path;
}

lanelet::ConstLanelets RouteHandler::getLaneChangeTarget(
  const geometry_msgs::msg::Pose & pose) const
{
  lanelet::ConstLanelet lanelet;
  lanelet::ConstLanelets target_lanelets;
  if (!getClosestLaneletWithinRoute(pose, &lanelet)) {
    return target_lanelets;
  }

  int num = getNumLaneToPreferredLane(lanelet);
  if (num < 0) {
    auto right_lanelet = (!!routing_graph_ptr_->right(lanelet)) ?
      routing_graph_ptr_->right(lanelet) :
      routing_graph_ptr_->adjacentRight(lanelet);
    target_lanelets = getLaneletSequence(right_lanelet.get());
  }
  if (num > 0) {
    auto left_lanelet = (!!routing_graph_ptr_->left(lanelet)) ?
      routing_graph_ptr_->left(lanelet) :
      routing_graph_ptr_->adjacentLeft(lanelet);
    target_lanelets = getLaneletSequence(left_lanelet.get());
  }
  return target_lanelets;
}

std::vector<LaneChangePath> RouteHandler::getLaneChangePaths(
  const lanelet::ConstLanelets & original_lanelets, const lanelet::ConstLanelets & target_lanelets,
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Twist & twist,
  const LaneChangerParameters & parameter) const
{
  std::vector<LaneChangePath> candidate_paths;

  if (original_lanelets.empty() || target_lanelets.empty()) {
    return candidate_paths;
  }

  // rename parameter
  const double backward_path_length = parameter.backward_path_length;
  const double forward_path_length = parameter.forward_path_length;
  const double lane_change_prepare_duration = parameter.lane_change_prepare_duration;
  const double lane_changing_duration = parameter.lane_changing_duration;
  const double minimum_lane_change_length = parameter.minimum_lane_change_length;
  const double maximum_deceleration = parameter.maximum_deceleration;
  const int lane_change_sampling_num = parameter.lane_change_sampling_num;

  // get velocity
  const double v0 = util::l2Norm(twist.linear);

  constexpr double buffer = 1.0;  // buffer for min_lane_change_length
  const double acceleration_resolution = std::abs(maximum_deceleration) / lane_change_sampling_num;

  for (double acceleration = 0.0; acceleration >= -maximum_deceleration;
    acceleration -= acceleration_resolution)
  {
    PathWithLaneId reference_path;
    const double v1 = v0 + acceleration * lane_change_prepare_duration;
    const double v2 = v1 + acceleration * lane_changing_duration;

    // skip if velocity becomes less than zero before finishing lane change
    if (v2 < 0.0) {continue;}

    // get path on original lanes
    const double straight_distance = v0 * lane_change_prepare_duration +
      0.5 * acceleration * std::pow(lane_change_prepare_duration, 2);
    double lane_change_distance =
      v1 * lane_changing_duration + 0.5 * acceleration * std::pow(lane_changing_duration, 2);
    lane_change_distance = std::max(lane_change_distance, minimum_lane_change_length);

    PathWithLaneId reference_path1;
    {
      const auto arc_position = lanelet::utils::getArcCoordinates(original_lanelets, pose);
      const double s_start = arc_position.length - backward_path_length;
      const double s_end = arc_position.length + straight_distance;
      reference_path1 = getReferencePath(original_lanelets, s_start, s_end);
    }

    PathWithLaneId reference_path2;
    {
      const double lane_length = lanelet::utils::getLaneletLength2d(target_lanelets);
      const auto arc_position = lanelet::utils::getArcCoordinates(target_lanelets, pose);
      const int num = std::abs(getNumLaneToPreferredLane(target_lanelets.back()));
      double s_start = arc_position.length + straight_distance + lane_change_distance;
      s_start = std::min(s_start, lane_length - num * minimum_lane_change_length);
      double s_end = s_start + forward_path_length;
      s_end = std::min(s_end, lane_length - num * (minimum_lane_change_length + buffer));
      s_end = std::max(s_end, s_start + std::numeric_limits<double>::epsilon());
      reference_path2 = getReferencePath(target_lanelets, s_start, s_end);
    }

    if (reference_path1.points.empty() || reference_path2.points.empty()) {
      RCLCPP_ERROR_STREAM(logger_, "reference path is empty!! something wrong...");
      continue;
    }

    LaneChangePath candidate_path;
    candidate_path.acceleration = acceleration;
    candidate_path.preparation_length = straight_distance;
    candidate_path.lane_change_length = lane_change_distance;
    candidate_path.path = combineReferencePath(reference_path1, reference_path2, 5.0, 2, logger_);

    // set fixed flag
    for (auto & pt : candidate_path.path.points) {
      pt.point.type = autoware_planning_msgs::msg::PathPoint::FIXED;
    }
    candidate_paths.push_back(candidate_path);
  }

  return candidate_paths;
}

double RouteHandler::getLaneChangeableDistance(
  const geometry_msgs::msg::Pose & current_pose, const LaneChangeDirection & direction)
{
  lanelet::ConstLanelet current_lane;
  if (!getClosestLaneletWithinRoute(current_pose, &current_lane)) {
    return 0;
  }

  // get lanelets after current lane
  auto lanelet_sequence = getLaneletSequenceAfter(current_lane);
  lanelet_sequence.insert(lanelet_sequence.begin(), current_lane);

  double accumulated_distance = 0;
  for (const auto & lane : lanelet_sequence) {
    lanelet::ConstLanelet target_lane;
    if (direction == LaneChangeDirection::RIGHT) {
      if (!getRightLaneletWithinRoute(lane, &target_lane)) {
        break;
      }
    }
    if (direction == LaneChangeDirection::LEFT) {
      if (!getLeftLaneletWithinRoute(lane, &target_lane)) {
        break;
      }
    }
    double lane_length = lanelet::utils::getLaneletLength3d(lane);

    // overwrite  goal because lane change must be finished before reaching goal
    if (isInGoalRouteSection(lane)) {
      const auto goal_position = lanelet::utils::conversion::toLaneletPoint(getGoalPose().position);
      const auto goal_arc_coordinates = lanelet::geometry::toArcCoordinates(
        lanelet::utils::to2D(lane.centerline()), lanelet::utils::to2D(goal_position).basicPoint());
      lane_length = std::min(goal_arc_coordinates.length, lane_length);
    }

    // subtract distance up to current position for first lane
    if (lane == current_lane) {
      const auto current_position =
        lanelet::utils::conversion::toLaneletPoint(current_pose.position);
      const auto arc_coordinate = lanelet::geometry::toArcCoordinates(
        lanelet::utils::to2D(lane.centerline()),
        lanelet::utils::to2D(current_position).basicPoint());
      lane_length = std::max(lane_length - arc_coordinate.length, 0.0);
    }
    accumulated_distance += lane_length;
  }
  return accumulated_distance;
}

lanelet::ConstLanelets RouteHandler::getCheckTargetLanesFromPath(
  const autoware_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::ConstLanelets & target_lanes, const double check_length)
{
  std::vector<int64_t> target_lane_ids;
  target_lane_ids.reserve(target_lanes.size());
  for (const auto & llt : target_lanes) {
    target_lane_ids.push_back(llt.id());
  }

  // find first lanelet in target lanes along path
  int64_t root_lane_id = lanelet::InvalId;
  for (const auto & point : path.points) {
    for (const auto & lane_id : point.lane_ids) {
      if (exists(target_lane_ids, lane_id)) {
        root_lane_id = lane_id;
      }
    }
  }
  // return emtpy lane if failed to find root lane_id
  if (root_lane_id == lanelet::InvalId) {
    return target_lanes;
  }
  lanelet::ConstLanelet root_lanelet;
  for (const auto & llt : target_lanes) {
    if (llt.id() == root_lane_id) {
      root_lanelet = llt;
    }
  }

  const auto sequences = lanelet::utils::query::getPrecedingLaneletSequences(
    routing_graph_ptr_, root_lanelet, check_length);
  lanelet::ConstLanelets check_lanelets;
  for (const auto & sequence : sequences) {
    for (const auto & llt : sequence) {
      if (!lanelet::utils::contains(check_lanelets, llt)) {
        check_lanelets.push_back(llt);
      }
    }
  }
  for (const auto & llt : target_lanes) {
    if (!lanelet::utils::contains(check_lanelets, llt)) {
      check_lanelets.push_back(llt);
    }
  }
  return check_lanelets;
}

lanelet::routing::RoutingGraphContainer RouteHandler::getOverallGraph() const
{
  return *overall_graphs_ptr_;
}

}  // namespace lane_change_planner
