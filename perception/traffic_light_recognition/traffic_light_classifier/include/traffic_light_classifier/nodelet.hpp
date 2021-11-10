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

#ifndef TRAFFIC_LIGHT_CLASSIFIER__NODELET_HPP_
#define TRAFFIC_LIGHT_CLASSIFIER__NODELET_HPP_

#include "traffic_light_classifier/classifier_interface.hpp"

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/traffic_light.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <memory>
#include <mutex>

#if ENABLE_GPU
#include "traffic_light_classifier/cnn_classifier.hpp"
#endif

#include "traffic_light_classifier/color_classifier.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace traffic_light
{
class TrafficLightClassifierNodelet : public rclcpp::Node
{
public:
  explicit TrafficLightClassifierNodelet(const rclcpp::NodeOptions & options);
  void imageRoiCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
    const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr &
      input_rois_msg);

private:
  enum ClassifierType {
    HSVFilter = 0,
    CNN = 1,
  };
  void connectCb();

  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<autoware_auto_perception_msgs::msg::TrafficLightRoiArray> roi_sub_;
  typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image, autoware_auto_perception_msgs::msg::TrafficLightRoiArray>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, autoware_auto_perception_msgs::msg::TrafficLightRoiArray>
    ApproximateSyncPolicy;
  typedef message_filters::Synchronizer<ApproximateSyncPolicy> ApproximateSync;
  std::shared_ptr<ApproximateSync> approximate_sync_;
  bool is_approximate_sync_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>::SharedPtr
    traffic_signal_array_pub_;
  std::shared_ptr<ClassifierInterface> classifier_ptr_;
};

}  // namespace traffic_light

#endif  // TRAFFIC_LIGHT_CLASSIFIER__NODELET_HPP_
