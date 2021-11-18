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

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: concatenate_data.cpp 35231 2011-01-14 05:33:20Z rusu $
 *
 */

#include "pointcloud_preprocessor/concatenate_data/concatenate_data_nodelet.hpp"

#include <pcl_ros/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

//////////////////////////////////////////////////////////////////////////////////////////////

namespace pointcloud_preprocessor
{
PointCloudConcatenateDataSynchronizerComponent::PointCloudConcatenateDataSynchronizerComponent(
  const rclcpp::NodeOptions & node_options)
: Node("point_cloud_concatenator_component", node_options)
{
  // Set parameters
  {
    output_frame_ = static_cast<std::string>(declare_parameter("output_frame", ""));
    if (output_frame_.empty()) {
      RCLCPP_ERROR(get_logger(), "Need an 'output_frame' parameter to be set before continuing!");
      return;
    }
    declare_parameter("input_topics", std::vector<std::string>());
    input_topics_ = get_parameter("input_topics").as_string_array();
    if (input_topics_.empty()) {
      RCLCPP_ERROR(get_logger(), "Need a 'input_topics' parameter to be set before continuing!");
      return;
    }
    if (input_topics_.size() == 1) {
      RCLCPP_ERROR(get_logger(), "Only one topic given. Need at least two topics to continue.");
      return;
    }

    // Optional parameters
    maximum_queue_size_ = static_cast<int>(declare_parameter("max_queue_size", 5));
    timeout_sec_ = static_cast<double>(declare_parameter("timeout_sec", 0.1));
  }

  // Initialize not_subscribed_topic_names_
  {
    for (const std::string & e : input_topics_) {
      not_subscribed_topic_names_.insert(e);
    }
  }

  // Publishers
  {
    pub_output_ = this->create_publisher<PointCloud2>(
      "output", rclcpp::SensorDataQoS().keep_last(maximum_queue_size_));
  }

  // Subscribers
  {
    RCLCPP_INFO_STREAM(
      get_logger(), "Subscribing to " << input_topics_.size() << " user given topics as inputs:");
    for (auto & input_topic : input_topics_) {
      RCLCPP_INFO_STREAM(get_logger(), " - " << input_topic);
    }

    // Subscribe to the filters
    filters_.resize(input_topics_.size());

    // First input_topics_.size () filters are valid
    for (size_t d = 0; d < input_topics_.size(); ++d) {
      cloud_stdmap_.insert(std::make_pair(input_topics_[d], nullptr));
      cloud_stdmap_tmp_ = cloud_stdmap_;

      // CAN'T use auto type here.
      std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr msg)> cb = std::bind(
        &PointCloudConcatenateDataSynchronizerComponent::cloud_callback, this,
        std::placeholders::_1, input_topics_[d]);

      filters_[d].reset();
      filters_[d] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topics_[d], rclcpp::SensorDataQoS().keep_last(maximum_queue_size_), cb);
    }
    auto twist_cb = std::bind(
      &PointCloudConcatenateDataSynchronizerComponent::twist_callback, this, std::placeholders::_1);
    sub_twist_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
      "/vehicle/status/velocity_status", rclcpp::QoS{100}, twist_cb);
  }

  // Set timer
  {
    auto cb = std::bind(&PointCloudConcatenateDataSynchronizerComponent::timer_callback, this);
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(timeout_sec_));
    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(cb)>>(
      get_clock(), period, std::move(cb), get_node_base_interface()->get_context());
    get_node_timers_interface()->add_timer(timer_, nullptr);
  }

  // Diagnostic Updater
  {
    updater_.setHardwareID("concatenate_data_checker");
    updater_.add(
      "concat_status", this, &PointCloudConcatenateDataSynchronizerComponent::checkConcatStatus);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudConcatenateDataSynchronizerComponent::transformPointCloud(
  const PointCloud2::ConstSharedPtr & in, PointCloud2::SharedPtr & out)
{
  // Transform the point clouds into the specified output frame
  if (output_frame_ != in->header.frame_id) {
    // TODO(YamatoAndo): use TF2
    if (!pcl_ros::transformPointCloud(output_frame_, *in, *out, *tf2_buffer_)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[transformPointCloud] Error converting first input dataset from %s to %s.",
        in->header.frame_id.c_str(), output_frame_.c_str());
      return;
    }
  } else {
    out = std::make_shared<PointCloud2>(*in);
  }
}

void PointCloudConcatenateDataSynchronizerComponent::combineClouds(
  const PointCloud2::ConstSharedPtr & in1, const PointCloud2::ConstSharedPtr & in2,
  PointCloud2::SharedPtr & out)
{
  if (twist_ptr_queue_.empty()) {
    pcl::concatenatePointCloud(*in1, *in2, *out);
    out->header.stamp = std::min(rclcpp::Time(in1->header.stamp), rclcpp::Time(in2->header.stamp));
    return;
  }

  const auto old_stamp = std::min(rclcpp::Time(in1->header.stamp), rclcpp::Time(in2->header.stamp));
  auto old_twist_ptr_it = std::lower_bound(
    std::begin(twist_ptr_queue_), std::end(twist_ptr_queue_), old_stamp,
    [](const geometry_msgs::msg::TwistStamped::ConstSharedPtr & x_ptr, const rclcpp::Time & t) {
      return rclcpp::Time(x_ptr->header.stamp) < t;
    });
  old_twist_ptr_it =
    old_twist_ptr_it == twist_ptr_queue_.end() ? (twist_ptr_queue_.end() - 1) : old_twist_ptr_it;

  const auto new_stamp = std::max(rclcpp::Time(in1->header.stamp), rclcpp::Time(in2->header.stamp));
  auto new_twist_ptr_it = std::lower_bound(
    std::begin(twist_ptr_queue_), std::end(twist_ptr_queue_), new_stamp,
    [](const geometry_msgs::msg::TwistStamped::ConstSharedPtr & x_ptr, const rclcpp::Time & t) {
      return rclcpp::Time(x_ptr->header.stamp) < t;
    });
  new_twist_ptr_it =
    new_twist_ptr_it == twist_ptr_queue_.end() ? (twist_ptr_queue_.end() - 1) : new_twist_ptr_it;

  auto prev_time = old_stamp;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  for (auto twist_ptr_it = old_twist_ptr_it; twist_ptr_it != new_twist_ptr_it + 1; ++twist_ptr_it) {
    const double dt =
      (twist_ptr_it != new_twist_ptr_it)
        ? (rclcpp::Time((*twist_ptr_it)->header.stamp) - rclcpp::Time(prev_time)).seconds()
        : (rclcpp::Time(new_stamp) - rclcpp::Time(prev_time)).seconds();

    if (std::fabs(dt) > 0.1) {
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(10000).count(),
        "Time difference is too large. Cloud not interpolate. Please confirm twist topic and "
        "timestamp");
      break;
    }

    const double dis = (*twist_ptr_it)->twist.linear.x * dt;
    yaw += (*twist_ptr_it)->twist.angular.z * dt;
    x += dis * std::cos(yaw);
    y += dis * std::sin(yaw);
    prev_time = (*twist_ptr_it)->header.stamp;
  }
  Eigen::AngleAxisf rotation_x(0, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rotation_y(0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rotation_z(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f translation(x, y, 0);
  Eigen::Matrix4f rotation_matrix = (translation * rotation_z * rotation_y * rotation_x).matrix();

  // TODO(YamatoAndo): if output_frame_ is not base_link, we must transform

  if (rclcpp::Time(in1->header.stamp) > rclcpp::Time(in2->header.stamp)) {
    sensor_msgs::msg::PointCloud2::SharedPtr in1_t(new sensor_msgs::msg::PointCloud2());
    pcl_ros::transformPointCloud(rotation_matrix, *in1, *in1_t);
    pcl::concatenatePointCloud(*in1_t, *in2, *out);
    out->header.stamp = in2->header.stamp;
  } else {
    sensor_msgs::msg::PointCloud2::SharedPtr in2_t(new sensor_msgs::msg::PointCloud2());
    pcl_ros::transformPointCloud(rotation_matrix, *in2, *in2_t);
    pcl::concatenatePointCloud(*in1, *in2_t, *out);
    out->header.stamp = in1->header.stamp;
  }
}

void PointCloudConcatenateDataSynchronizerComponent::publish()
{
  sensor_msgs::msg::PointCloud2::SharedPtr concat_cloud_ptr_ = nullptr;
  not_subscribed_topic_names_.clear();

  for (const auto & e : cloud_stdmap_) {
    if (e.second != nullptr) {
      sensor_msgs::msg::PointCloud2::SharedPtr transformed_cloud_ptr(
        new sensor_msgs::msg::PointCloud2());
      transformPointCloud(e.second, transformed_cloud_ptr);
      if (concat_cloud_ptr_ == nullptr) {
        concat_cloud_ptr_ = transformed_cloud_ptr;
      } else {
        PointCloudConcatenateDataSynchronizerComponent::combineClouds(
          concat_cloud_ptr_, transformed_cloud_ptr, concat_cloud_ptr_);
      }

    } else {
      not_subscribed_topic_names_.insert(e.first);
    }
  }

  if (concat_cloud_ptr_) {
    auto output = std::make_unique<sensor_msgs::msg::PointCloud2>(*concat_cloud_ptr_);
    pub_output_->publish(std::move(output));
  } else {
    RCLCPP_WARN(this->get_logger(), "concat_cloud_ptr_ is nullptr, skipping pointcloud publish.");
  }

  updater_.force_update();

  cloud_stdmap_ = cloud_stdmap_tmp_;
  std::for_each(std::begin(cloud_stdmap_tmp_), std::end(cloud_stdmap_tmp_), [](auto & e) {
    e.second = nullptr;
  });
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudConcatenateDataSynchronizerComponent::removeRADTFields(
  const sensor_msgs::msg::PointCloud2 & input_cloud, sensor_msgs::msg::PointCloud2 & output_cloud)
{
  bool has_intensity = std::any_of(
    input_cloud.fields.begin(), input_cloud.fields.end(),
    [](auto & field) { return field.name == "intensity"; });

  if (input_cloud.fields.size() == 3 || (input_cloud.fields.size() == 4 && has_intensity)) {
    output_cloud = input_cloud;
  } else if (has_intensity) {
    pcl::PointCloud<PointXYZI> tmp_cloud;
    pcl::fromROSMsg(input_cloud, tmp_cloud);
    pcl::toROSMsg(tmp_cloud, output_cloud);
    output_cloud.header = input_cloud.header;
  } else {
    pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
    pcl::fromROSMsg(input_cloud, tmp_cloud);
    pcl::toROSMsg(tmp_cloud, output_cloud);
    output_cloud.header = input_cloud.header;
  }
}

void PointCloudConcatenateDataSynchronizerComponent::setPeriod(const int64_t new_period)
{
  if (!timer_) {
    return;
  }
  int64_t old_period = 0;
  rcl_ret_t ret = rcl_timer_get_period(timer_->get_timer_handle().get(), &old_period);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't get old period");
  }
  ret = rcl_timer_exchange_period(timer_->get_timer_handle().get(), new_period, &old_period);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't exchange_period");
  }
}

void PointCloudConcatenateDataSynchronizerComponent::cloud_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_ptr, const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);

  sensor_msgs::msg::PointCloud2 xyz_cloud;
  removeRADTFields(*input_ptr, xyz_cloud);
  sensor_msgs::msg::PointCloud2::ConstSharedPtr xyz_input_ptr(
    new sensor_msgs::msg::PointCloud2(xyz_cloud));

  const bool is_already_subscribed_this = (cloud_stdmap_[topic_name] != nullptr);
  const bool is_already_subscribed_tmp = std::any_of(
    std::begin(cloud_stdmap_tmp_), std::end(cloud_stdmap_tmp_),
    [](const auto & e) { return e.second != nullptr; });

  if (is_already_subscribed_this) {
    cloud_stdmap_tmp_[topic_name] = xyz_input_ptr;

    if (!is_already_subscribed_tmp) {
      auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timeout_sec_));
      try {
        setPeriod(period.count());
      } catch (rclcpp::exceptions::RCLError & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
      }
      timer_->reset();
    }
  } else {
    cloud_stdmap_[topic_name] = xyz_input_ptr;

    const bool is_subscribed_all = std::all_of(
      std::begin(cloud_stdmap_), std::end(cloud_stdmap_),
      [](const auto & e) { return e.second != nullptr; });

    if (is_subscribed_all) {
      for (const auto & e : cloud_stdmap_tmp_) {
        if (e.second != nullptr) {
          cloud_stdmap_[e.first] = e.second;
        }
      }
      std::for_each(std::begin(cloud_stdmap_tmp_), std::end(cloud_stdmap_tmp_), [](auto & e) {
        e.second = nullptr;
      });

      timer_->cancel();
      publish();
    }
  }
}

void PointCloudConcatenateDataSynchronizerComponent::timer_callback()
{
  using std::chrono_literals::operator""ms;
  timer_->cancel();
  if (mutex_.try_lock()) {
    publish();
    mutex_.unlock();
  } else {
    try {
      std::chrono::nanoseconds period = 10ms;
      setPeriod(period.count());
    } catch (rclcpp::exceptions::RCLError & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
    }
    timer_->reset();
  }
}

void PointCloudConcatenateDataSynchronizerComponent::twist_callback(
  const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr input)
{
  // if rosbag restart, clear buffer
  if (!twist_ptr_queue_.empty()) {
    if (rclcpp::Time(twist_ptr_queue_.front()->header.stamp) > rclcpp::Time(input->header.stamp)) {
      twist_ptr_queue_.clear();
    }
  }

  // pop old data
  while (!twist_ptr_queue_.empty()) {
    if (
      rclcpp::Time(twist_ptr_queue_.front()->header.stamp) + rclcpp::Duration::from_seconds(1.0) >
      rclcpp::Time(input->header.stamp)) {
      break;
    }
    twist_ptr_queue_.pop_front();
  }

  auto twist_ptr = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_ptr->header.stamp = input->header.stamp;
  twist_ptr->twist.linear.x = input->longitudinal_velocity;
  twist_ptr->twist.linear.y = input->lateral_velocity;
  twist_ptr->twist.angular.z = input->heading_rate;
  twist_ptr_queue_.push_back(twist_ptr);
}

void PointCloudConcatenateDataSynchronizerComponent::checkConcatStatus(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  for (const std::string & e : input_topics_) {
    const std::string subscribe_status = not_subscribed_topic_names_.count(e) ? "NG" : "OK";
    stat.add(e, subscribe_status);
  }

  const int8_t level = not_subscribed_topic_names_.empty()
                         ? diagnostic_msgs::msg::DiagnosticStatus::OK
                         : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  const std::string message = not_subscribed_topic_names_.empty()
                                ? "Concatenate all topics"
                                : "Some topics are not concatenated";
  stat.summary(level, message);
}
}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent)
