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

#pragma once

#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <autoware_perception_msgs/DynamicObjectWithFeatureArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>

#include "trt_yolo.hpp"

namespace object_recognition
{
class TensorrtYoloNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();
  void connectCb();
  void callback(const sensor_msgs::Image::ConstPtr & image_msg);
  bool readLabelFile(const std::string & filepath, std::vector<std::string> * labels);

private:
  ros::NodeHandle nh_, pnh_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  std::mutex connect_mutex_;

  image_transport::Publisher image_pub_;
  ros::Publisher objects_pub_;

  image_transport::Subscriber image_sub_;

  yolo::Config yolo_config_;

  std::vector<std::string> labels_;
  std::unique_ptr<float[]> out_scores_;
  std::unique_ptr<float[]> out_boxes_;
  std::unique_ptr<float[]> out_classes_;
  std::unique_ptr<yolo::Net> net_ptr_;
};

}  // namespace object_recognition
