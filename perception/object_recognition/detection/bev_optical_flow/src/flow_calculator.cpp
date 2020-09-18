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

#include "bev_optical_flow/flow_calculator.h"

namespace bev_optical_flow
{
FlowCalculator::FlowCalculator() : nh_(""), pnh_("~")
{
  pnh_.param<float>("quality_level", quality_level_, 0.01);
  pnh_.param<int>("min_distance", min_distance_, 10);
  pnh_.param<int>("block_size", block_size_, 3);
  pnh_.param<float>("harris_k", harris_k_, 0.04);
  pnh_.param<int>("max_corners", max_corners_, 10000);

  pnh_.param<int>("sparce_size", sparce_size_, 4);
  pnh_.param<int>("num_split", num_split_, 3);
  pnh_.param<bool>("debug", debug_, false);

  utils_ = std::make_shared<Utils>();
  lidar_to_image_ = std::make_shared<LidarToBEVImage>();
}

bool FlowCalculator::isInitialized() {
  return setup_;
}

bool FlowCalculator::calcOpticalFlow(
  cv::Mat& current_image,
  cv::Mat& prev_image,
  std::vector<cv::Point2f>& prev_points,
  autoware_perception_msgs::DynamicObjectWithFeatureArray& flow_array_msg)
{
  if (prev_image.empty())
    current_image.copyTo(prev_image);

  std::vector<cv::Point2f> current_points;
  cv::goodFeaturesToTrack(
    current_image, current_points, max_corners_, quality_level_, min_distance_,
    cv::Mat(), block_size_, true, harris_k_);

  cv::TermCriteria termcrit(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.01);
  // cv::cornerSubPix(current_image, current_points, cv::Size(10, 10), cv::Size(-1, -1), termcrit);

  if ( !prev_points.empty() ) {
    std::vector<unsigned char> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK
      (prev_image, current_image, current_points, prev_points,
        status, err, cv::Size(15, 15), 2, termcrit, 0, 0.01);

    for (size_t i=0; i<current_points.size(); i++) {
      if (!status[i]) {
        continue;
      }
      int depth = static_cast<int>(image_.at<uchar>(current_points[i].y, current_points[i].x));
      if ( depth > 0) {
        autoware_perception_msgs::DynamicObjectWithFeature flow;
        flow.object.state.pose_covariance.pose.position.x = current_points[i].x;
        flow.object.state.pose_covariance.pose.position.y = current_points[i].y;
        flow.object.state.pose_covariance.pose.position.z = depth;
        flow.object.state.twist_covariance.twist.linear.x = current_points[i].x - prev_points[i].x - vehicle_vel_.x;
        flow.object.state.twist_covariance.twist.linear.y = current_points[i].y - prev_points[i].y - vehicle_vel_.y;
        flow_array_msg.feature_objects.push_back(flow);
      }
    }
  } else {
    prev_points = current_points;
  }
  cv::swap(current_image, prev_image);
  return true;
}

bool FlowCalculator::calcSceneFlow(
  const autoware_perception_msgs::DynamicObjectWithFeature& optical_flow,
  autoware_perception_msgs::DynamicObjectWithFeature& scene_flow)
{
  geometry_msgs::Point current_point =
    utils_->pixel2point(optical_flow.object.state.pose_covariance.pose.position,
      geometry_msgs::Vector3(),
      image_.size(),
      utils_->getMap2BaseAngle(current_stamp_));
  geometry_msgs::Point prev_point =
    utils_->pixel2point(optical_flow.object.state.pose_covariance.pose.position,
      optical_flow.object.state.twist_covariance.twist.linear,
      image_.size(),
      utils_->getMap2BaseAngle(current_stamp_));
  geometry_msgs::Vector3 mptopic_twist;
  mptopic_twist.x = current_point.x - prev_point.x;
  mptopic_twist.y = current_point.y - prev_point.y;
  mptopic_twist.z = current_point.z - prev_point.z;
  geometry_msgs::Vector3 kph_twist = utils_->mptopic2kph(mptopic_twist,topic_rate_);
  scene_flow.object.state.pose_covariance.pose.position = current_point;
  scene_flow.object.state.twist_covariance.twist.linear = kph_twist;

  return true;
}

bool FlowCalculator::getSceneFlowArray(
  const autoware_perception_msgs::DynamicObjectWithFeatureArray& optical_flow_array,
  autoware_perception_msgs::DynamicObjectWithFeatureArray& scene_flow_array) {
  for (auto optical_flow : optical_flow_array.feature_objects) {
    autoware_perception_msgs::DynamicObjectWithFeature scene_flow;
    calcSceneFlow(optical_flow, scene_flow);
    scene_flow_array.feature_objects.push_back(scene_flow);
  }
  return true;
}

void FlowCalculator::setup(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  if (prev_stamp_.sec == 0) {
    setup_ = false;
    prev_stamp_ = cloud_msg->header.stamp;
    return;
  }
  topic_rate_ = (cloud_msg->header.stamp - prev_stamp_).toSec();

  cv::Mat image;
  lidar_to_image_->getBEVImage(cloud_msg, image);
  vehicle_vel_ = utils_->getVehicleVel(cloud_msg->header.stamp, prev_stamp_);

  if (image.channels() > 1) {
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
  }
  image.copyTo(image_);
  current_stamp_ = cloud_msg->header.stamp;

  setup_ = true;
}

void FlowCalculator::run(
  autoware_perception_msgs::DynamicObjectWithFeatureArray& scene_flow_array)
{
  autoware_perception_msgs::DynamicObjectWithFeatureArray optical_flow_array;
  optical_flow_array.header = scene_flow_array.header;

  calcOpticalFlow(image_, prev_image_, prev_points_, optical_flow_array);
  getSceneFlowArray(optical_flow_array, scene_flow_array);

  prev_stamp_ = current_stamp_;

  if ( debug_ ) {
    debugger_.publishDebugVisualizations
      (optical_flow_array, scene_flow_array, image_, topic_rate_, vehicle_vel_);
  }
}

} //bev_optical_flow
