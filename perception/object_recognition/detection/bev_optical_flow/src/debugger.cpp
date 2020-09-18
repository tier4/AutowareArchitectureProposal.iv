/*
 * Copyright 2020 TierIV. All rights reserved.
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

#include "bev_optical_flow/debugger.h"

Debugger::Debugger() : nh_(""), pnh_("~")
{
  debug_image_pub_ =
    pnh_.advertise<sensor_msgs::Image>("output/debug_image", 1);
  debug_marker_array_pub_ =
    pnh_.advertise<visualization_msgs::MarkerArray>("output/debug_marker_line", 1);
  debug_text_marker_array_pub_ =
    pnh_.advertise<visualization_msgs::MarkerArray>("output/debug_text_marker", 1);
  utils_ = std::make_shared<bev_optical_flow::Utils>();
}

bool Debugger::createMarker(
  const autoware_perception_msgs::DynamicObjectWithFeature& scene_flow,
  visualization_msgs::Marker& debug_marker,
  visualization_msgs::Marker& debug_text_marker,
  int idx,
  double topic_rate)
{
  auto current_point = scene_flow.object.state.pose_covariance.pose.position;
  auto kph_twist = scene_flow.object.state.twist_covariance.twist;
  geometry_msgs::Vector3 mptopic_twist = utils_->kph2mptopic(kph_twist.linear, topic_rate);
  geometry_msgs::Point prev_point;
  prev_point.x = current_point.x - mptopic_twist.x;
  prev_point.y = current_point.y - mptopic_twist.y;
  prev_point.z = current_point.z - mptopic_twist.z;

  debug_marker.ns = "flow" + std::to_string(idx);
  debug_marker.id = idx;
  debug_marker.action = visualization_msgs::Marker::ADD;
  debug_marker.pose.orientation.w = 1.0;
  debug_marker.type = visualization_msgs::Marker::LINE_STRIP;
  debug_marker.scale.x = 0.05;
  debug_marker.color.g = 1.0;
  debug_marker.color.a = 1.0;
  debug_marker.points.push_back(current_point);
  debug_marker.points.push_back(prev_point);

  debug_text_marker.ns = "flow" + std::to_string(idx);
  debug_text_marker.id = idx;
  debug_text_marker.action = visualization_msgs::Marker::ADD;
  debug_text_marker.pose.orientation.w = 1.0;
  debug_text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  debug_text_marker.pose.position = current_point;
  float v = std::sqrt(std::pow(kph_twist.linear.x, 2) +
    std::pow(kph_twist.linear.y, 2) +
    std::pow(kph_twist.linear.z, 2));
  debug_text_marker.text = std::to_string(v) + "[km/h]";
  debug_text_marker.scale.x = 0.3;
  debug_text_marker.scale.y = 0.3;
  debug_text_marker.scale.z = 0.1;
  debug_text_marker.color.r = 1.0f;
  debug_text_marker.color.g = 1.0f;
  debug_text_marker.color.b = 1.0f;
  debug_text_marker.color.a = 1.0;

  return true;
}

void Debugger::publishOpticalFlowImage(const cv::Point2f& vehicle_vel)
{
  // publish messages
  cv::Mat debug_image;
  cv::cvtColor(image_, debug_image, CV_GRAY2BGR);
  cv::Point2f current_point(static_cast<int>
    (image_.cols * 0.5), static_cast<int>(image_.rows * 0.5));
  cv::Point2f prev_point = current_point - vehicle_vel;
  cv::line(debug_image, current_point, prev_point, cv::Scalar(0,255,0), 1, 8, 0);
  cv::circle(debug_image, current_point, 2, cv::Scalar(255,0,0), -1);
  cv::circle(debug_image, current_point, 20, cv::Scalar(255,200,100));
  cv::circle(debug_image, current_point, static_cast<int>(image_.cols * 0.5), cv::Scalar(100,100,100));

  for (size_t i=0; i<optical_flow_array_.feature_objects.size(); i++) {
    auto flow = optical_flow_array_.feature_objects.at(i);
    float pose_x = flow.object.state.pose_covariance.pose.position.x;
    float pose_y = flow.object.state.pose_covariance.pose.position.y;
    float twist_x = flow.object.state.twist_covariance.twist.linear.x;
    float twist_y = flow.object.state.twist_covariance.twist.linear.y;
    cv::Point2d current_px(pose_x, pose_y);
    cv::Point2d prev_px(pose_x + twist_x, pose_y + twist_y);
    cv::line(debug_image, prev_px, current_px, cv::Scalar(0,255,0), 1, 8, 0);
    cv::circle(debug_image, current_px, 0, cv::Scalar(255,100,0), -1);
  }

  sensor_msgs::ImagePtr output_image_msg =
    cv_bridge::CvImage(optical_flow_array_.header,
      sensor_msgs::image_encodings::BGR8,
      debug_image).toImageMsg();
  debug_image_pub_.publish(output_image_msg);
}

void Debugger::publishSceneFlowMarker(double topic_rate)
{
  visualization_msgs::MarkerArray debug_marker_array;
  visualization_msgs::MarkerArray debug_text_marker_array;
  for (size_t i=0; i<scene_flow_array_.feature_objects.size(); i++) {
    auto scene_flow = scene_flow_array_.feature_objects.at(i);
    visualization_msgs::Marker debug_marker;
    visualization_msgs::Marker debug_text_marker;
    createMarker(scene_flow, debug_marker, debug_text_marker, i, topic_rate);
    // line marker
    debug_marker.header = scene_flow_array_.header;
    debug_marker_array.markers.push_back(debug_marker);
    // text marker
    debug_text_marker.header = scene_flow_array_.header;
    debug_text_marker_array.markers.push_back(debug_text_marker);

  }
  debug_marker_array_pub_.publish(debug_marker_array);
  debug_text_marker_array_pub_.publish(debug_text_marker_array);
}

bool Debugger::publishDebugVisualizations(
  const autoware_perception_msgs::DynamicObjectWithFeatureArray& optical_flow_array,
  const autoware_perception_msgs::DynamicObjectWithFeatureArray& scene_flow_array,
  const cv::Mat& image,
  double topic_rate,
  const cv::Point2f& vehicle_vel)
{
  optical_flow_array_ = optical_flow_array;
  scene_flow_array_ = scene_flow_array;
  image.copyTo(image_);

  publishOpticalFlowImage(vehicle_vel);
  publishSceneFlowMarker(topic_rate);
  return true;
}
