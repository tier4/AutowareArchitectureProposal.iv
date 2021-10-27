// Copyright 2018-2019 Autoware Foundation
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

#include "map_based_prediction_ros.hpp"

#include "map_based_prediction.hpp"

#include <autoware_utils/autoware_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

std::string toHexString(const unique_identifier_msgs::msg::UUID & id)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +id.uuid[i];
  }
  return ss.str();
}

double MapBasedPredictionROS::getObjectYaw(
  const autoware_perception_msgs::msg::DynamicObject & object)
{
  if (object.state.orientation_reliable) {
    return tf2::getYaw(object.state.pose_covariance.pose.orientation);
  }

  geometry_msgs::msg::Pose object_frame_pose;
  geometry_msgs::msg::Pose map_frame_pose;
  object_frame_pose.position.x = object.state.twist_covariance.twist.linear.x * 0.1;
  object_frame_pose.position.y = object.state.twist_covariance.twist.linear.y * 0.1;
  tf2::Transform tf_object2future;
  tf2::Transform tf_map2object;
  tf2::Transform tf_map2future;

  tf2::fromMsg(object.state.pose_covariance.pose, tf_map2object);
  tf2::fromMsg(object_frame_pose, tf_object2future);
  tf_map2future = tf_map2object * tf_object2future;
  tf2::toMsg(tf_map2future, map_frame_pose);
  double dx = map_frame_pose.position.x - object.state.pose_covariance.pose.position.x;
  double dy = map_frame_pose.position.y - object.state.pose_covariance.pose.position.y;
  return std::atan2(dy, dx);
}

double MapBasedPredictionROS::calculateLikelihood(
  const std::vector<geometry_msgs::msg::Pose> & path,
  const autoware_perception_msgs::msg::DynamicObject & object)
{
  // We compute the confidence value based on the object current position and angle
  // Calculate path length
  const double path_len = autoware_utils::calcArcLength(path);
  const size_t nearest_segment_idx =
    autoware_utils::findNearestSegmentIndex(path, object.state.pose_covariance.pose.position);
  const double l = autoware_utils::calcLongitudinalOffsetToSegment(
    path, nearest_segment_idx, object.state.pose_covariance.pose.position);
  const double current_s_position =
    autoware_utils::calcSignedArcLength(path, 0, nearest_segment_idx) + l;
  // If the obstacle is ahead of this path, we assume the confidence for this path is 0
  if (current_s_position > path_len) {
    return 0.0;
  }

  // Euclid Distance
  const double e = autoware_utils::calcDistance2d(
    object.state.pose_covariance.pose.position, path.at(nearest_segment_idx));
  const double abs_d = std::fabs(std::sqrt(std::max(e * e - l * l, 0.0)));

  // Yaw Difference between obstacle and lane angle
  const double lane_yaw = tf2::getYaw(path.at(nearest_segment_idx).orientation);
  const double object_yaw = getObjectYaw(object);
  const double delta_yaw = object_yaw - lane_yaw;
  const double abs_norm_delta_yaw = std::fabs(autoware_utils::normalizeRadian(delta_yaw));

  // Compute Chi-squared distributed (Equation (8) in the paper)
  const double sigma_d = sigma_lateral_offset_;  // Standard Deviation for lateral position
  const double sigma_yaw = M_PI * sigma_yaw_angle_ / 180.0;  // Standard Deviation for yaw angle
  Eigen::Vector2d delta;
  delta << abs_d, abs_norm_delta_yaw;
  Eigen::Matrix2d P_inv;
  P_inv << 1.0 / (sigma_d * sigma_d), 0.0, 0.0, 1.0 / (sigma_yaw * sigma_yaw);
  const double MINIMUM_DISTANCE = 1e-6;
  const double dist = std::max(delta.dot(P_inv * delta), MINIMUM_DISTANCE);
  return 1.0 / dist;
}

bool MapBasedPredictionROS::checkCloseLaneletCondition(
  const std::pair<double, lanelet::Lanelet> & lanelet,
  const autoware_perception_msgs::msg::DynamicObject & object,
  const lanelet::BasicPoint2d & search_point)
{
  // Step1. If we only have one point in the centerline, we will ignore the lanelet
  if (lanelet.second.centerline().size() <= 1) {
    return false;
  }

  // Step2. Check if the obstacle is inside of this lanelet
  if (!lanelet::geometry::inside(lanelet.second, search_point)) {
    return false;
  }

  // Step3. Calculate the angle difference between the lane angle and obstacle angle
  double object_yaw = getObjectYaw(object);
  const double lane_yaw =
    lanelet::utils::getLaneletAngle(lanelet.second, object.state.pose_covariance.pose.position);
  const double delta_yaw = object_yaw - lane_yaw;
  const double normalized_delta_yaw = std::atan2(std::sin(delta_yaw), std::cos(delta_yaw));
  const double abs_norm_delta = std::fabs(normalized_delta_yaw);

  // Step4. Check if the closest lanelet is valid, and add all
  // of the lanelets that are below max_dist and max_delta_yaw
  if (
    lanelet.first < dist_threshold_for_searching_lanelet_ &&
    abs_norm_delta < delta_yaw_threshold_for_searching_lanelet_) {
    return true;
  }

  return false;
}

bool MapBasedPredictionROS::getClosestLanelets(
  const autoware_perception_msgs::msg::DynamicObject & object,
  const lanelet::LaneletMapPtr & lanelet_map_ptr_, std::vector<lanelet::Lanelet> & closest_lanelets,
  const std::string uuid_string)
{
  std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();

  // obstacle point
  lanelet::BasicPoint2d search_point(
    object.state.pose_covariance.pose.position.x, object.state.pose_covariance.pose.position.y);

  // nearest lanelet
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 10);

  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  debug_accumulated_time_ += time.count() / (1000.0 * 1000.0);

  // No Closest Lanelets
  if (nearest_lanelets.empty()) {
    return false;
  }

  // uuid is an unique object id
  if (uuid2laneids_.empty() || uuid2laneids_.count(uuid_string) == 0) {
    // First time or an object which is detected first time
    bool is_found_target_closest_lanelet = false;
    lanelet::Lanelet target_closest_lanelet;
    for (const auto & lanelet : nearest_lanelets) {
      // Check if the close lanelets meet the necessary condition for start lanelets
      if (checkCloseLaneletCondition(lanelet, object, search_point)) {
        target_closest_lanelet = lanelet.second;
        is_found_target_closest_lanelet = true;
        closest_lanelets.push_back(target_closest_lanelet);
      }
    }

    // If the closest lanelet is valid, return true
    if (is_found_target_closest_lanelet) {
      return true;
    }
  } else {
    bool is_found_target_closest_lanelet = false;
    lanelet::Lanelet target_closest_lanelet;
    for (const auto & laneid : uuid2laneids_.at(uuid_string)) {
      for (const auto & lanelet : nearest_lanelets) {
        if (laneid != lanelet.second.id()) {
          continue;
        }

        // Check if the lanelet is inside the closest_lanelets
        bool inside_closest_lanelets = false;
        for (const auto & closest_lanelet : closest_lanelets) {
          if (closest_lanelet.id() == lanelet.second.id()) {
            inside_closest_lanelets = true;
            break;
          }
        }
        if (inside_closest_lanelets) {
          continue;
        }

        // Check if the close lanelets meet the necessary condition for start lanelets
        if (checkCloseLaneletCondition(lanelet, object, search_point)) {
          target_closest_lanelet = lanelet.second;
          is_found_target_closest_lanelet = true;
          closest_lanelets.push_back(target_closest_lanelet);
        }
      }
    }
    if (is_found_target_closest_lanelet) {
      return true;
    }
  }

  return false;
}

MapBasedPredictionROS::MapBasedPredictionROS(const rclcpp::NodeOptions & node_options)
: Node("map_based_prediction", node_options),
  interpolating_resolution_(0.5),
  debug_accumulated_time_(0.0)
{
  auto ret =
    rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(clock);
  tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_);
  has_subscribed_map_ = declare_parameter("map_based_prediction/has_subscribed_map", false);
  prediction_time_horizon_ = declare_parameter("prediction_time_horizon", 10.0);
  prediction_sampling_delta_time_ = declare_parameter("prediction_sampling_delta_time", 0.5);
  dist_threshold_for_searching_lanelet_ =
    declare_parameter("dist_threshold_for_searching_lanelet", 3.0);
  delta_yaw_threshold_for_searching_lanelet_ =
    declare_parameter("delta_yaw_threshold_for_searching_lanelet", 0.785);
  sigma_lateral_offset_ = declare_parameter("sigma_lateral_offset", 0.5);
  sigma_yaw_angle_ = declare_parameter("sigma_yaw_angle", 5.0);
  map_based_prediction_ = std::make_shared<MapBasedPrediction>(
    interpolating_resolution_, prediction_time_horizon_, prediction_sampling_delta_time_);

  sub_objects_ = this->create_subscription<autoware_perception_msgs::msg::DynamicObjectArray>(
    "/perception/object_recognition/tracking/objects", 1,
    std::bind(&MapBasedPredictionROS::objectsCallback, this, std::placeholders::_1));
  sub_map_ = this->create_subscription<autoware_lanelet2_msgs::msg::MapBin>(
    "/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MapBasedPredictionROS::mapCallback, this, std::placeholders::_1));

  pub_objects_ = this->create_publisher<autoware_perception_msgs::msg::DynamicObjectArray>(
    "objects", rclcpp::QoS{1});
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "objects_path_markers", rclcpp::QoS{1});
}

void MapBasedPredictionROS::addValidPath(
  const lanelet::routing::LaneletPaths & candidate_paths,
  lanelet::routing::LaneletPaths & valid_paths)
{
  // Check if candidate paths are already in the valid paths
  for (const auto & candidate_path : candidate_paths) {
    bool already_searched = false;
    for (const auto & valid_path : valid_paths) {
      for (const auto & llt : valid_path) {
        if (candidate_path.back().id() == llt.id()) {
          already_searched = true;
        }
      }
    }
    if (!already_searched) {
      valid_paths.push_back(candidate_path);
    }
  }
}

void MapBasedPredictionROS::objectsCallback(
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr in_objects)
{
  debug_accumulated_time_ = 0.0;
  std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();

  if (!lanelet_map_ptr_) {
    return;
  }

  geometry_msgs::msg::TransformStamped world2map_transform;
  geometry_msgs::msg::TransformStamped map2world_transform;
  geometry_msgs::msg::TransformStamped debug_map2lidar_transform;
  try {
    world2map_transform = tf_buffer_ptr_->lookupTransform(
      "map",                        // target
      in_objects->header.frame_id,  // src
      in_objects->header.stamp, rclcpp::Duration::from_seconds(0.1));
    map2world_transform = tf_buffer_ptr_->lookupTransform(
      in_objects->header.frame_id,  // target
      "map",                        // src
      in_objects->header.stamp, rclcpp::Duration::from_seconds(0.1));
    debug_map2lidar_transform = tf_buffer_ptr_->lookupTransform(
      "base_link",  // target
      "map",        // src
      rclcpp::Time(), rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
    return;
  }

  autoware_perception_msgs::msg::DynamicObjectArray tmp_objects_without_map;
  tmp_objects_without_map.header = in_objects->header;
  DynamicObjectWithLanesArray prediction_input;
  prediction_input.header = in_objects->header;

  for (const auto & object : in_objects->objects) {
    DynamicObjectWithLanes tmp_object;
    tmp_object.object = object;
    if (in_objects->header.frame_id != "map") {
      geometry_msgs::msg::PoseStamped pose_in_map;
      geometry_msgs::msg::PoseStamped pose_orig;
      pose_orig.pose = object.state.pose_covariance.pose;
      tf2::doTransform(pose_orig, pose_in_map, world2map_transform);
      tmp_object.object.state.pose_covariance.pose = pose_in_map.pose;
    }

    if (
      object.semantic.type != autoware_perception_msgs::msg::Semantic::CAR &&
      object.semantic.type != autoware_perception_msgs::msg::Semantic::BUS &&
      object.semantic.type != autoware_perception_msgs::msg::Semantic::TRUCK) {
      tmp_objects_without_map.objects.push_back(tmp_object.object);
      continue;
    }

    // If the obstacle is too slow, we do linear prediction
    const double min_lon_velocity_ms_for_map_based_prediction = 1.0;
    if (
      std::fabs(object.state.twist_covariance.twist.linear.x) <
      min_lon_velocity_ms_for_map_based_prediction) {
      tmp_objects_without_map.objects.push_back(tmp_object.object);
      continue;
    }

    // generate non redundant lanelet vector
    std::vector<lanelet::Lanelet> start_lanelets;      // current lanelet
    std::string uuid_string = toHexString(object.id);  // object id
    if (!getClosestLanelets(tmp_object.object, lanelet_map_ptr_, start_lanelets, uuid_string)) {
      tmp_objects_without_map.objects.push_back(object);
      continue;
    }

    // Obtain valid Paths
    const double delta_horizon = 1.0;
    const double obj_vel = object.state.twist_covariance.twist.linear.x;
    lanelet::routing::LaneletPaths paths;
    for (const auto & start_lanelet : start_lanelets) {
      // Step1. Get the right lanelet
      lanelet::routing::LaneletPaths right_paths;
      auto opt_right = routing_graph_ptr_->right(start_lanelet);
      if (!!opt_right) {
        for (double horizon = prediction_time_horizon_; horizon > 0; horizon -= delta_horizon) {
          const double search_dist = horizon * obj_vel + 10.0;
          lanelet::routing::LaneletPaths tmp_paths =
            routing_graph_ptr_->possiblePaths(*opt_right, search_dist, 0, false);
          addValidPath(tmp_paths, right_paths);
        }
      }

      // Step2. Get the left lanelet
      lanelet::routing::LaneletPaths left_paths;
      auto opt_left = routing_graph_ptr_->left(start_lanelet);
      if (!!opt_left) {
        for (double horizon = prediction_time_horizon_; horizon > 0; horizon -= delta_horizon) {
          const double search_dist = horizon * obj_vel + 10.0;
          lanelet::routing::LaneletPaths tmp_paths =
            routing_graph_ptr_->possiblePaths(*opt_left, search_dist, 0, false);
          addValidPath(tmp_paths, left_paths);
        }
      }

      // Step3. Get the centerline
      lanelet::routing::LaneletPaths center_paths;
      for (double horizon = prediction_time_horizon_; horizon > 0; horizon -= delta_horizon) {
        const double search_dist = horizon * obj_vel + 10.0;
        lanelet::routing::LaneletPaths tmp_paths =
          routing_graph_ptr_->possiblePaths(start_lanelet, search_dist, 0, false);
        addValidPath(tmp_paths, center_paths);
      }

      // Step4. Insert Valid Paths
      paths.insert(paths.end(), center_paths.begin(), center_paths.end());
      paths.insert(paths.end(), right_paths.begin(), right_paths.end());
      paths.insert(paths.end(), left_paths.begin(), left_paths.end());
    }

    // If there is no valid path, we'll mark this object as map-less object
    if (paths.empty()) {
      tmp_objects_without_map.objects.push_back(object);
      continue;
    }

    std::vector<int> lanelet_ids;
    for (const auto & path : paths) {
      for (const auto & lanelet : path) {
        lanelet_ids.push_back(lanelet.id());
      }
    }

    std::string uid_string = toHexString(object.id);
    if (uuid2laneids_.count(uid_string) == 0) {
      uuid2laneids_.emplace(uid_string, lanelet_ids);
    } else {
      // add if not yet having lanelet_id
      for (const auto & current_uid : lanelet_ids) {
        bool is_redundant = false;
        for (const auto & cached_uid : uuid2laneids_.at(uid_string)) {
          if (cached_uid == current_uid) {
            is_redundant = true;
            break;
          }
        }
        if (is_redundant) {
          continue;
        }
        uuid2laneids_.at(uid_string).push_back(current_uid);
      }
    }

    std::vector<std::vector<geometry_msgs::msg::Pose>> tmp_paths;
    std::vector<double> tmp_confidence;
    for (const auto & path : paths) {
      std::vector<geometry_msgs::msg::Pose> tmp_path;

      // Insert Positions. Note that we insert points from previous lanelet
      if (!path.empty()) {
        lanelet::ConstLanelets prev_lanelets = routing_graph_ptr_->previous(path.front());
        if (!prev_lanelets.empty()) {
          lanelet::ConstLanelet prev_lanelet = prev_lanelets.front();
          for (const auto & point : prev_lanelet.centerline()) {
            geometry_msgs::msg::Pose tmp_pose;
            tmp_pose.position.x = point.x();
            tmp_pose.position.y = point.y();
            tmp_pose.position.z = point.z();
            tmp_path.push_back(tmp_pose);
          }
        }
      }

      for (const auto & lanelet : path) {
        for (const auto & point : lanelet.centerline()) {
          geometry_msgs::msg::Pose tmp_pose;
          tmp_pose.position.x = point.x();
          tmp_pose.position.y = point.y();
          tmp_pose.position.z = point.z();

          // Prevent from inserting same points
          if (!tmp_path.empty()) {
            const auto prev_pose = tmp_path.back();
            const double tmp_dist = autoware_utils::calcDistance2d(prev_pose, tmp_pose);
            if (tmp_dist < 1e-6) {
              continue;
            }
          }

          tmp_path.push_back(tmp_pose);
        }
      }

      if (tmp_path.size() < 2) {
        continue;
      }

      // Compute yaw angles
      for (size_t pose_id = 0; pose_id < tmp_path.size() - 1; ++pose_id) {
        double tmp_yaw = std::atan2(
          tmp_path.at(pose_id + 1).position.y - tmp_path.at(pose_id).position.y,
          tmp_path.at(pose_id + 1).position.x - tmp_path.at(pose_id).position.x);
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, tmp_yaw);
        tmp_path.at(pose_id).orientation = tf2::toMsg(quat);
      }
      tmp_path.back().orientation = tmp_path.at(tmp_path.size() - 2).orientation;

      //////////////////////////////////////////////////////////////////////
      // Calculate Confidence of each path(centerline) for this obstacle //
      ////////////////////////////////////////////////////////////////////
      const double confidence = calculateLikelihood(tmp_path, object);
      // Ignore a path that has too low confidence
      if (confidence < 1e-6) {
        continue;
      }

      tmp_paths.push_back(tmp_path);
      tmp_confidence.push_back(confidence);
    }

    tmp_object.lanes = tmp_paths;
    tmp_object.confidence = tmp_confidence;
    prediction_input.objects.push_back(tmp_object);
  }

  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);

  std::vector<autoware_perception_msgs::msg::DynamicObject> out_objects_in_map;
  map_based_prediction_->doPrediction(prediction_input, out_objects_in_map);
  autoware_perception_msgs::msg::DynamicObjectArray output;
  output.header = in_objects->header;
  output.header.frame_id = "map";
  output.objects = out_objects_in_map;

  std::vector<autoware_perception_msgs::msg::DynamicObject> out_objects_without_map;
  map_based_prediction_->doLinearPrediction(tmp_objects_without_map, out_objects_without_map);
  output.objects.insert(
    output.objects.begin(), out_objects_without_map.begin(), out_objects_without_map.end());
  pub_objects_->publish(output);
}

void MapBasedPredictionROS::mapCallback(
  const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Start loading lanelet");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  RCLCPP_INFO(get_logger(), "Map is loaded");
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MapBasedPredictionROS)
