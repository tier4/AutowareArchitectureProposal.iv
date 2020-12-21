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
#include "rclcpp/rclcpp.hpp"

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "autoware_perception_msgs/msg/dynamic_object_with_feature_array.hpp"

// STL
#include <chrono>
#include <memory>
#include <string>

// local
#include "TrtNet.hpp"
#include "data_reader.hpp"

class TensorrtYoloROS : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>::SharedPtr
    pub_objects_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;

  std::unique_ptr<Tn::trtNet> net_ptr_;

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr in_image);
  std::vector<float> prepareImage(cv::Mat & in_img);
  std::vector<Tn::Bbox> postProcessImg(
    std::vector<Yolo::Detection> & detections, const int classes, cv::Mat & img,
    autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & out_objects);
  void doNms(std::vector<Yolo::Detection> & detections, int classes, float nmsThresh);
  /* data */

public:
  TensorrtYoloROS(/* args */);
  ~TensorrtYoloROS();

  void createROSPubSub();
};
