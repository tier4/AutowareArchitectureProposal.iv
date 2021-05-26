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
#ifndef ROI_IMAGE_SAVER__NODE_HPP_
#define ROI_IMAGE_SAVER__NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "autoware_perception_msgs/msg/traffic_light_roi_array.hpp"
#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber_filter.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "sensor_msgs/msg/image.hpp"

namespace traffic_light
{
class TrafficLightRoiImageSaver : public rclcpp::Node
{
public:
  explicit TrafficLightRoiImageSaver(const rclcpp::NodeOptions & node_options);

private:
  void imageRoiCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
    const autoware_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_tl_roi_msg);

  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<autoware_perception_msgs::msg::TrafficLightRoiArray> roi_sub_;

  std::string save_dir_;
  std::shared_ptr<rclcpp::Rate> save_rate_ptr_;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, autoware_perception_msgs::msg::TrafficLightRoiArray>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  Sync sync_;
};

}  // namespace traffic_light

#endif  // ROI_IMAGE_SAVER__NODE_HPP_
