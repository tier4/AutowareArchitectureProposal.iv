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

#include "tensorrt_yolo/nodelet.hpp"
#include <glob.h>

namespace
{
std::vector<std::string> getFilePath(const std::string & input_dir)
{
  glob_t globbuf;
  std::vector<std::string> files;
  glob((input_dir + "*").c_str(), 0, NULL, &globbuf);
  for (size_t i = 0; i < globbuf.gl_pathc; i++) {
    files.push_back(globbuf.gl_pathv[i]);
  }
  globfree(&globbuf);
  return files;
}
}  // namespace
namespace object_recognition
{
void TensorrtYoloNodelet::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(pnh_));
  std::string onnx_file;
  std::string engine_file;
  std::string label_file;
  std::string calib_image_directory;
  std::string calib_cache_file;
  std::string mode;
  pnh_.param<std::string>("onnx_file", onnx_file, "");
  pnh_.param<std::string>("engine_file", engine_file, "");
  pnh_.param<std::string>("label_file", label_file, "");
  pnh_.param<std::string>("calib_image_directory", calib_image_directory, "");
  pnh_.param<std::string>("calib_cache_file", calib_cache_file, "");
  pnh_.param<std::string>("mode", mode, "FP32");
  pnh_.param<int>("num_anchors", yolo_config_.num_anchors, 3);
  if (!pnh_.getParam("anchors", yolo_config_.anchors)) {
    NODELET_WARN("Fail to load anchors");
    yolo_config_.anchors = {10, 13, 16,  30,  33, 23,  30,  61,  62,
                            45, 59, 119, 116, 90, 156, 198, 373, 326};
  }
  if (!pnh_.getParam("scale_x_y", yolo_config_.scale_x_y)) {
    NODELET_WARN("Fail to load scale_x_y");
    yolo_config_.scale_x_y = {1.0, 1.0, 1.0};
  }
  pnh_.param<float>("score_thresh", yolo_config_.score_thresh, 0.1);
  pnh_.param<float>("iou_thresh", yolo_config_.iou_thresh, 0.45);
  pnh_.param<int>("detections_per_im", yolo_config_.detections_per_im, 100);
  pnh_.param<bool>("use_darknet_layer", yolo_config_.use_darknet_layer, true);
  pnh_.param<float>("ignore_thresh", yolo_config_.ignore_thresh, 0.5);

  if (!readLabelFile(label_file, &labels_)) {
    NODELET_ERROR("Could not find label file");
  }
  std::ifstream fs(engine_file);
  const auto calibration_images = getFilePath(calib_image_directory);
  if (fs.is_open()) {
    NODELET_INFO("Found %s", engine_file.c_str());
    net_ptr_.reset(new yolo::Net(engine_file, false));
    if (net_ptr_->getMaxBatchSize() != 1) {
      NODELET_INFO(
        "Max batch size %d should be 1. Rebuild engine from file", net_ptr_->getMaxBatchSize());
      net_ptr_.reset(
        new yolo::Net(onnx_file, mode, 1, yolo_config_, calibration_images, calib_cache_file));
      net_ptr_->save(engine_file);
    }
  } else {
    NODELET_INFO("Could not find %s, try making TensorRT engine from onnx", engine_file.c_str());
    net_ptr_.reset(
      new yolo::Net(onnx_file, mode, 1, yolo_config_, calibration_images, calib_cache_file));
    net_ptr_->save(engine_file);
  }
  image_transport::SubscriberStatusCallback connect_cb =
    boost::bind(&TensorrtYoloNodelet::connectCb, this);
  std::lock_guard<std::mutex> lock(connect_mutex_);
  objects_pub_ = pnh_.advertise<autoware_perception_msgs::DynamicObjectWithFeatureArray>(
    "out/objects", 1, boost::bind(&TensorrtYoloNodelet::connectCb, this),
    boost::bind(&TensorrtYoloNodelet::connectCb, this));
  image_pub_ = it_->advertise("out/image", 1, connect_cb, connect_cb);
  out_scores_ =
    std::make_unique<float[]>(net_ptr_->getMaxBatchSize() * net_ptr_->getMaxDetections());
  out_boxes_ =
    std::make_unique<float[]>(net_ptr_->getMaxBatchSize() * net_ptr_->getMaxDetections() * 4);
  out_classes_ =
    std::make_unique<float[]>(net_ptr_->getMaxBatchSize() * net_ptr_->getMaxDetections());
}

void TensorrtYoloNodelet::connectCb()
{
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (objects_pub_.getNumSubscribers() == 0 && image_pub_.getNumSubscribers() == 0)
    image_sub_.shutdown();
  else if (!image_sub_)
    image_sub_ = it_->subscribe("in/image", 1, &TensorrtYoloNodelet::callback, this);
}

void TensorrtYoloNodelet::callback(const sensor_msgs::Image::ConstPtr & in_image_msg)
{
  autoware_perception_msgs::DynamicObjectWithFeatureArray out_objects;

  cv_bridge::CvImagePtr in_image_ptr;
  try {
    in_image_ptr = cv_bridge::toCvCopy(in_image_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    NODELET_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  if (!net_ptr_->detect(
        in_image_ptr->image, out_scores_.get(), out_boxes_.get(), out_classes_.get())) {
    NODELET_WARN("Fail to inference");
    return;
  }
  const auto width = in_image_ptr->image.cols;
  const auto height = in_image_ptr->image.rows;
  for (int i = 0; i < yolo_config_.detections_per_im; ++i) {
    if (out_scores_[i] < yolo_config_.ignore_thresh) break;
    autoware_perception_msgs::DynamicObjectWithFeature object;
    object.feature.roi.x_offset = out_boxes_[4 * i] * width;
    object.feature.roi.y_offset = out_boxes_[4 * i + 1] * height;
    object.feature.roi.width = out_boxes_[4 * i + 2] * width;
    object.feature.roi.height = out_boxes_[4 * i + 3] * height;
    object.object.semantic.confidence = out_scores_[i];
    const auto class_id = static_cast<int>(out_classes_[i]);
    if (labels_[class_id] == "car") {
      object.object.semantic.type = autoware_perception_msgs::Semantic::CAR;
    } else if (labels_[class_id] == "person") {
      object.object.semantic.type = autoware_perception_msgs::Semantic::PEDESTRIAN;
    } else if (labels_[class_id] == "bus") {
      object.object.semantic.type = autoware_perception_msgs::Semantic::BUS;
    } else if (labels_[class_id] == "truck") {
      object.object.semantic.type = autoware_perception_msgs::Semantic::TRUCK;
    } else if (labels_[class_id] == "bicycle") {
      object.object.semantic.type = autoware_perception_msgs::Semantic::BICYCLE;
    } else if (labels_[class_id] == "motorbike") {
      object.object.semantic.type = autoware_perception_msgs::Semantic::MOTORBIKE;
    } else {
      object.object.semantic.type = autoware_perception_msgs::Semantic::UNKNOWN;
    }
    out_objects.feature_objects.push_back(object);
    const auto left = std::max(0, static_cast<int>(object.feature.roi.x_offset));
    const auto top = std::max(0, static_cast<int>(object.feature.roi.y_offset));
    const auto right =
      std::min(static_cast<int>(object.feature.roi.x_offset + object.feature.roi.width), width);
    const auto bottom =
      std::min(static_cast<int>(object.feature.roi.y_offset + object.feature.roi.height), height);
    cv::rectangle(
      in_image_ptr->image, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 255), 3,
      8, 0);
  }
  image_pub_.publish(in_image_ptr->toImageMsg());

  out_objects.header = in_image_msg->header;
  objects_pub_.publish(out_objects);
}

bool TensorrtYoloNodelet::readLabelFile(
  const std::string & filepath, std::vector<std::string> * labels)
{
  std::ifstream labelsFile(filepath);
  if (!labelsFile.is_open()) {
    NODELET_ERROR("Could not open label file. [%s]", filepath.c_str());
    return false;
  }
  std::string label;
  while (getline(labelsFile, label)) {
    labels->push_back(label);
  }
  return true;
}

}  // namespace object_recognition

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(object_recognition::TensorrtYoloNodelet, nodelet::Nodelet)
