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
#ifndef DYNAMIC_OBJECT_VISUALIZATION__DYNAMIC_OBJECT_VISUALIZER_HPP_
#define DYNAMIC_OBJECT_VISUALIZATION__DYNAMIC_OBJECT_VISUALIZER_HPP_

#include <iomanip>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_with_feature_array.hpp"
#include "autoware_perception_msgs/msg/predicted_path.hpp"
#include "autoware_perception_msgs/msg/shape.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class DynamicObjectVisualizer : public rclcpp::Node
{
private:
  // ros
  bool with_feature_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>::SharedPtr
    sub_with_feature_;

  void dynamicObjectWithFeatureCallback(
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr input_msg);
  void dynamicObjectCallback(
    const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg);
  bool calcBoundingBoxLineList(
    const autoware_perception_msgs::msg::Shape & shape,
    std::vector<geometry_msgs::msg::Point> & points);
  bool calcCylinderLineList(
    const autoware_perception_msgs::msg::Shape & shape,
    std::vector<geometry_msgs::msg::Point> & points);
  bool calcCircleLineList(
    const geometry_msgs::msg::Point center, const double radius,
    std::vector<geometry_msgs::msg::Point> & points, const int n = 20);
  bool calcPolygonLineList(
    const autoware_perception_msgs::msg::Shape & shape,
    std::vector<geometry_msgs::msg::Point> & points);
  bool calcPathLineList(
    const autoware_perception_msgs::msg::PredictedPath & path,
    std::vector<geometry_msgs::msg::Point> & points);
  bool getLabel(const autoware_perception_msgs::msg::Semantic & semantic, std::string & label);
  void getColor(
    const autoware_perception_msgs::msg::DynamicObject & object, std_msgs::msg::ColorRGBA & color);
  void initColorList(std::vector<std_msgs::msg::ColorRGBA> & colors);
  void initPose(geometry_msgs::msg::Pose & pose);

  bool only_known_objects_;
  std::vector<std_msgs::msg::ColorRGBA> colors_;


  inline std::string uuid_to_string(unique_identifier_msgs::msg::UUID const & u)
  {
    std::stringstream ss;
    for (auto i = 0; i < 16; ++i) {
      ss << std::hex << std::setfill('0') << std::setw(2) << +u.uuid[i];
    }
    return ss.str();
  }

public:
  explicit DynamicObjectVisualizer(const rclcpp::NodeOptions & node_options);
};

#endif  // DYNAMIC_OBJECT_VISUALIZATION__DYNAMIC_OBJECT_VISUALIZER_HPP_
