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
#include <turn_signal_decider/turn_signal_decider.h>

using autoware_planning_msgs::msg::PathWithLaneId;
using autoware_vehicle_msgs::msg::TurnSignal;

using namespace std::placeholders;

namespace
{
double getDistance3d(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}
}  // namespace

namespace turn_signal_decider
{
TurnSignalDecider::TurnSignalDecider(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options), data_(this)
{
  // setup data manager
  constexpr double vehicle_pose_update_period = 0.1;
  auto vehicle_pose_timer_callback = std::bind(&DataManager::onVehiclePoseUpdate, &data_);
  auto vehicle_pose_timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(vehicle_pose_update_period));

  vehicle_pose_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(vehicle_pose_timer_callback)>>(
    this->get_clock(), vehicle_pose_timer_period, std::move(vehicle_pose_timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(vehicle_pose_timer_, nullptr);

  path_subscription_ = this->create_subscription<autoware_planning_msgs::msg::PathWithLaneId>(
    "input/path_with_lane_id", rclcpp::QoS{1}, std::bind(&DataManager::onPathWithLaneId, &data_, _1));
  map_subscription_ = this->create_subscription<autoware_lanelet2_msgs::msg::MapBin>(
    "input/vector_map", rclcpp::QoS{1}, std::bind(&DataManager::onLaneletMap, &data_, _1));

  // get ROS parameters
  parameters_.lane_change_search_distance = this->declare_parameter("lane_change_search_distance", double(30));
  parameters_.intersection_search_distance = this->declare_parameter("intersection_search_distance", double(30));

  // set publishers
  turn_signal_publisher_ = this->create_publisher<TurnSignal>("output/turn_signal_cmd", rclcpp::QoS{1});

  constexpr double turn_signal_update_period = 0.1;
  auto turn_signal_timer_callback = std::bind(&TurnSignalDecider::onTurnSignalTimer, this);
  auto turn_signal_timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(turn_signal_update_period));

  turn_signal_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(turn_signal_timer_callback)>>(
    this->get_clock(), turn_signal_timer_period, std::move(turn_signal_timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(turn_signal_timer_, nullptr);
}

void TurnSignalDecider::onTurnSignalTimer()
{
  // wait for mandatory topics
  if (!data_.isDataReady()) {
    return;
  }

  // setup
  const auto path = data_.getPath();
  FrenetCoordinate3d vehicle_pose_frenet;
  if (!convertToFrenetCoordinate3d(
        path, data_.getVehiclePoseStamped().pose.position, &vehicle_pose_frenet)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
      "failed to convert vehicle pose into frenet coordinate");
    return;
  }

  // set turn signals according to closest manuevers
  TurnSignal turn_signal, lane_change_signal, intersection_signal;
  double distance_to_lane_change, distance_to_intersection;
  double min_distance = std::numeric_limits<double>::max();
  if (isChangingLane(path, vehicle_pose_frenet, &lane_change_signal, &distance_to_lane_change)) {
    if (min_distance > distance_to_lane_change) {
      min_distance = distance_to_lane_change;
      turn_signal = lane_change_signal;
    }
  }
  if (isTurning(path, vehicle_pose_frenet, &intersection_signal, &distance_to_intersection)) {
    if (min_distance > distance_to_intersection) {
      min_distance = distance_to_lane_change;
      turn_signal = intersection_signal;
    }
  }

  turn_signal.header.stamp = this->now();
  turn_signal.header.frame_id = "base_link";
  turn_signal_publisher_->publish(turn_signal);
}

lanelet::routing::RelationType TurnSignalDecider::getRelation(
  const lanelet::ConstLanelet & prev_lane, const lanelet::ConstLanelet & next_lane) const
{
  const auto routing_graph_ptr = data_.getRoutingGraphPtr();
  if (prev_lane == next_lane) {
    return lanelet::routing::RelationType::None;
  }
  const auto & relation = routing_graph_ptr->routingRelation(prev_lane, next_lane);
  if (relation) {
    return relation.get();
  }

  // check if lane change extends across muliple lanes
  const auto shortest_path = routing_graph_ptr->shortestPath(prev_lane, next_lane);
  if (shortest_path) {
    auto prev_llt = shortest_path->front();
    for (const auto & llt : shortest_path.get()) {
      if (prev_llt == llt) {
        continue;
      }
      const auto & relation = routing_graph_ptr->routingRelation(prev_llt, llt);
      if (!relation) {
        continue;
      }
      if (
        relation.get() == lanelet::routing::RelationType::Left ||
        relation.get() == lanelet::routing::RelationType::Right) {
        return relation.get();
      }
      prev_llt = llt;
    }
  }

  return lanelet::routing::RelationType::None;
}

bool TurnSignalDecider::isChangingLane(
  const autoware_planning_msgs::msg::PathWithLaneId & path, const FrenetCoordinate3d & vehicle_pose_frenet,
  TurnSignal * signal_state_ptr, double * distance_ptr) const
{
  if (signal_state_ptr == nullptr || distance_ptr == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Given argument is nullptr.");
    return false;
  }
  if (path.points.empty()) {
    return false;
  }

  double accumulated_distance = 0;

  auto prev_point = path.points.front();
  auto prev_lane_id = path.points.front().lane_ids.front();
  for (const auto & path_point : path.points) {
    accumulated_distance +=
      getDistance3d(prev_point.point.pose.position, path_point.point.pose.position);
    prev_point = path_point;
    const double distance_from_vehicle = accumulated_distance - vehicle_pose_frenet.length;
    if (distance_from_vehicle < 0) {
      continue;
    }
    for (const auto & lane_id : path_point.lane_ids) {
      if (lane_id == prev_lane_id) {
        continue;
      }

      const auto & prev_lane = data_.getLaneFromId(prev_lane_id);
      const auto & lane = data_.getLaneFromId(lane_id);
      prev_lane_id = lane_id;

      // check lane change relation
      const auto relation = getRelation(prev_lane, lane);
      if (relation == lanelet::routing::RelationType::Left) {
        signal_state_ptr->data = TurnSignal::LEFT;
        *distance_ptr = distance_from_vehicle;
        return true;
      }
      if (relation == lanelet::routing::RelationType::Right) {
        signal_state_ptr->data = TurnSignal::RIGHT;
        *distance_ptr = distance_from_vehicle;
        return true;
      }
    }

    if (distance_from_vehicle > parameters_.lane_change_search_distance) {
      return false;
    }
  }
  return false;
}

bool TurnSignalDecider::isTurning(
  const autoware_planning_msgs::msg::PathWithLaneId & path, const FrenetCoordinate3d & vehicle_pose_frenet,
  TurnSignal * signal_state_ptr, double * distance_ptr) const
{
  if (signal_state_ptr == nullptr || distance_ptr == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Given argument is nullptr.");
    return false;
  }
  if (path.points.empty()) {
    return false;
  }

  double accumulated_distance = 0;

  auto prev_point = path.points.front();
  auto prev_lane_id = lanelet::InvalId;
  for (const auto & path_point : path.points) {
    accumulated_distance +=
      getDistance3d(prev_point.point.pose.position, path_point.point.pose.position);
    prev_point = path_point;
    const double distance_from_vehicle = accumulated_distance - vehicle_pose_frenet.length;
    if (distance_from_vehicle < 0) {
      continue;
    }
    for (const auto & lane_id : path_point.lane_ids) {
      if (lane_id == prev_lane_id) {
        continue;
      }
      prev_lane_id = lane_id;

      const auto & lane = data_.getLaneFromId(lane_id);
      if (lane.attributeOr("turn_direction", std::string("none")) == "left") {
        signal_state_ptr->data = TurnSignal::LEFT;
        *distance_ptr = distance_from_vehicle;
        return true;
      }
      if (lane.attributeOr("turn_direction", std::string("none")) == "right") {
        signal_state_ptr->data = TurnSignal::RIGHT;
        *distance_ptr = distance_from_vehicle;
        return true;
      }
    }
    if (distance_from_vehicle > parameters_.intersection_search_distance) {
      return false;
    }
  }
  return false;
}

}  // namespace turn_signal_decider
