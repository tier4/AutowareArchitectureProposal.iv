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
 *
 * Authors: Yukihiro Saito
 *
 */

#ifndef TRAFFIC_LIGHT_MAP_BASED_DETECTOR__NODE_HPP_
#define TRAFFIC_LIGHT_MAP_BASED_DETECTOR__NODE_HPP_

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp"
#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_traffic_rules/TrafficRulesFactory.h"

#include "autoware_lanelet2_msgs/msg/map_bin.hpp"
#include "autoware_perception_msgs/msg/traffic_light_roi_array.hpp"
#include "autoware_planning_msgs/msg/route.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rclcpp/rclcpp.hpp"

#include <memory>

namespace traffic_light
{
class MapBasedDetector : public rclcpp::Node
{
public:
  MapBasedDetector();
  ~MapBasedDetector() {}

private:
  struct Config
  {
    double max_vibration_pitch;
    double max_vibration_yaw;
    double max_vibration_height;
    double max_vibration_width;
    double max_vibration_depth;
  };

  struct IdLessThan
  {
    bool operator()(
      const lanelet::ConstLineString3d & left,
      const lanelet::ConstLineString3d & right) const
    {
      return left.id() < right.id();
    }
  };

private:
  rclcpp::Subscription<autoware_lanelet2_msgs::msg::MapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Route>::SharedPtr route_sub_;

  rclcpp::Publisher<autoware_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr roi_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  using TrafficLightSet = std::set<lanelet::ConstLineString3d, IdLessThan>;

  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_ptr_;
  std::shared_ptr<TrafficLightSet> all_traffic_lights_ptr_;
  std::shared_ptr<TrafficLightSet> route_traffic_lights_ptr_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  Config config_;

  void mapCallback(const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr input_msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_msg);
  void routeCallback(const autoware_planning_msgs::msg::Route::ConstSharedPtr input_msg);
  void isInVisibility(
    const TrafficLightSet & all_traffic_lights,
    const geometry_msgs::msg::Pose & camera_pose,
    const sensor_msgs::msg::CameraInfo & camera_info,
    std::vector<lanelet::ConstLineString3d> & visible_traffic_lights);
  bool isInDistanceRange(
    const geometry_msgs::msg::Point & tl_point,
    const geometry_msgs::msg::Point & camera_point,
    const double max_distance_range);
  bool isInAngleRange(
    const double & tl_yaw, const double & camera_yaw, const double max_angele_range);
  bool isInImageFrame(
    const sensor_msgs::msg::CameraInfo & camera_info,
    const geometry_msgs::msg::Point & point);
  double normalizeAngle(const double & angle);
  bool getTrafficLightRoi(
    const geometry_msgs::msg::Pose & camera_pose,
    const sensor_msgs::msg::CameraInfo & camera_info,
    const lanelet::ConstLineString3d traffic_light,
    const Config & config,
    autoware_perception_msgs::msg::TrafficLightRoi & tl_roi);
  void publishVisibleTrafficLights(
    const geometry_msgs::msg::PoseStamped camera_pose_stamped,
    const std::vector<lanelet::ConstLineString3d> & visible_traffic_lights,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub);
};
}  // namespace traffic_light
#endif  // TRAFFIC_LIGHT_MAP_BASED_DETECTOR__NODE_HPP_
