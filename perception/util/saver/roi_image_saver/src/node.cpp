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
#include <memory>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "roi_image_saver/node.hpp"

namespace traffic_light
{
TrafficLightRoiImageSaver::TrafficLightRoiImageSaver(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("traffic_light_roi_image_saver_node", node_options),
  sync_(SyncPolicy(10), image_sub_, roi_sub_)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  image_sub_.subscribe(this, "input/image", "raw", rclcpp::QoS{1}.get_rmw_qos_profile());
  roi_sub_.subscribe(this, "input/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
  sync_.registerCallback(std::bind(&TrafficLightRoiImageSaver::imageRoiCallback, this, _1, _2));

  save_dir_ = declare_parameter("save_dir", "");
  double save_rate = declare_parameter("save_rate", 1.0);
  save_rate_ptr_ = std::make_shared<rclcpp::Rate>(save_rate);
}

void TrafficLightRoiImageSaver::imageRoiCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const autoware_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_tl_roi_msg)
{
  auto current_time = this->now();
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::BGR8);
    for (size_t i = 0; i < input_tl_roi_msg->rois.size(); ++i) {
      const sensor_msgs::msg::RegionOfInterest & roi = input_tl_roi_msg->rois.at(i).roi;
      cv::Mat clipped_image(
        cv_ptr->image, cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height));
      std::stringstream save_fine_name_stream;
      save_fine_name_stream << std::fixed << save_dir_ << "/" << input_tl_roi_msg->rois.at(i).id <<
        "_" << current_time.seconds() << ".png";
      std::string save_fine_name;
      save_fine_name_stream >> save_fine_name;
      cv::imwrite(save_fine_name, clipped_image);
      RCLCPP_INFO(this->get_logger(), "%s", save_fine_name.c_str());
    }
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Could not convert from '%s' to 'bgr8'.",
      input_image_msg->encoding.c_str());
  }
  save_rate_ptr_->sleep();
}
}  // namespace traffic_light

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::TrafficLightRoiImageSaver)
