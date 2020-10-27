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

#include "pointcloud_preprocessor/concatenate_data/concatenate_data_component.h"

//#include <pcl_ros/transforms.h>
//#include <pluginlib/class_list_macros.hpp>

#include <pcl_conversions/pcl_conversions.h>

//////////////////////////////////////////////////////////////////////////////////////////////

namespace pointcloud_preprocessor
{
PointCloudConcatenateDataSynchronizerComponent::PointCloudConcatenateDataSynchronizerComponent(
  const rclcpp::NodeOptions & node_options
)
  : Node("point_cloud_concatenator_component", node_options),
   maximum_queue_size_(3),
   timeout_sec_(0.1)
{
  // ---[ Mandatory parameters
  output_frame_ = static_cast<std::string>(declare_parameter("output_frame").get<std::string>());
  if (output_frame_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "[Constructor] Need a 'output_frame' paramter to be set before continuing!");
    return;
  }
  declare_parameter("input_topics");
  get_parameter("input_topics").as_string_array();
  //  input_topics_ = static_cast<std::string>(declare_paramter("input_topics").get<std::string>());
  if (input_topics_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "[Constructor] Need a 'input_topics' paramter to be set before continuing!");
    return;
  }
  if (input_topics_.size() == 1) {
    RCLCPP_ERROR(this->get_logger(), "[Constructor] Need more than one input topic for concatenation!");
    return;
  }
  // ---[ Optional parameters
  maximum_queue_size_ = static_cast<int>(declare_parameter("max_queue_size").get<std::size_t>());
  timeout_sec_ = static_cast<double>(declare_parameter("timeout_sec").get<double>());
}
  
void PointCloudConcatenateDataSynchronizerComponent::onInit()
{
  //  nodelet_topic_tools::NodeletLazy::onInit();

  // Paramter setup moved to constructor for ROS2 porting

  /*
  pnh_->getParam("output_frame", output_frame_);
  
  if (output_frame_.empty()) {
    NODELET_ERROR("[onInit] Need an 'output_frame' parameter to be set before continuing!");
    return;
  }

  if (!pnh_->getParam("input_topics", input_topics_)) {
    NODELET_ERROR("[onInit] Need a 'input_topics' parameter to be set before continuing!");
    return;
  }
  if (input_topics_.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    NODELET_ERROR("[onInit] Invalid 'input_topics' parameter given!");
    return;
  }
  if (input_topics_.size() == 1) {
    NODELET_ERROR("[onInit] Only one topic given. Need at least two topics to continue.");
    return;
  }
  */
  // ---[ Optional parameters
  // pnh_->getParam("max_queue_size", maximum_queue_size_);
  // pnh_->getParam("timeout_sec", timeout_sec_);

  // Output
  pub_output_ = this->create_publisher<PointCloud2>("output", maximum_queue_size_);
  pub_concat_num_ = this->create_publisher<std_msgs::msg::Int32>("concat_num", 10);
  pub_not_subscribed_topic_name_ =
    this->create_publisher<std_msgs::msg::String>("not_subscribed_topic_name", 10);


  // set up transform listener
  // Q. Should this be in subscribe function (if ROS2 supports NodeletLazy subscribe functionality)
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(
    tf2_buffer_, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false);
  //  onInitPostProcess();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudConcatenateDataSynchronizerComponent::subscribe()
{
  RCLCPP_INFO_STREAM(get_logger(), "Subscribing to " << input_topics_.size() << " user given topics as inputs:");
  for (int d = 0; d < input_topics_.size(); ++d)
    RCLCPP_INFO_STREAM(get_logger(), " - " << input_topics_[d]);

  // Subscribe to the filters
  filters_.resize(input_topics_.size());

  sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
									   "/vehicle/status/twist", rclcpp::QoS(100),
									   std::bind(&PointCloudConcatenateDataSynchronizerComponent::twist_callback, this, std::placeholders::_1));

  // First input_topics_.size () filters are valid
  for (int d = 0; d < input_topics_.size(); ++d) {
    cloud_stdmap_.insert(std::make_pair(input_topics_[d], nullptr));
    cloud_stdmap_tmp_ = cloud_stdmap_;

    //    filters_[d].reset(new ros::Subscriber());
    filters_[d].reset();
    //const rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr tmp_sub;

    //    filters_[d] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //    static_cast<std::string>(input_topics_[d]), rclcpp::QoS(maximum_queue_size_),
    //   std::bind(
    //	&PointCloudConcatenateDataSynchronizerComponent::cloud_callback, this, std::placeholders::_1,
    //	static_cast<std::string>(input_topics_[d])));
 
    //    auto filter_callback =  std::bind(
    //				      &PointCloudConcatenateDataSynchronizerComponent::cloud_callback, this, std::placeholders::_1,
    //				      input_topics_[d]);
 
    filters_[d] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
								       "points_in", rclcpp::QoS(10),//maximum_queue_size_),
								        std::bind(
				      &PointCloudConcatenateDataSynchronizerComponent::cloud_callback, this, std::placeholders::_1, input_topics_[d]));
  }


  auto on_timer_callback = std::bind(&PointCloudConcatenateDataSynchronizerComponent::timer_callback, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
     std::chrono::duration<double>(timeout_sec_));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer_callback)>>(
    this->get_clock(), period, std::move(on_timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
  //  timer_ = pnh_->createTimer(
  //   ros::Duration(timeout_sec_), &PointCloudConcatenateDataSynchronizerNodelet::timer_callback,
  //   this, true);

  // ROS2 port: timer_ changed to timer_base, but timer_base has no member named stop. Is this important for node operation?
  // timer_->stop();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudConcatenateDataSynchronizerComponent::unsubscribe()
{
  for (size_t d = 0; d < filters_.size(); ++d) {
    //    filters_[d]->shutdown();

    // ROS2 port - no equaivalent to shutdown - reset the shared ptr. Is this safe>
    filters_[d].reset();
  }
  //  sub_twist_.shutdown();
  sub_twist_.reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudConcatenateDataSynchronizerComponent::transformPointCloud(
  const PointCloud2::ConstSharedPtr & in, PointCloud2::SharedPtr & out)
{
  // Transform the point clouds into the specified output frame
  if (output_frame_ != in->header.frame_id) {
    // TODO use TF2
    /*
    if (!pcl_ros::transformPointCloud(output_frame_, *in, *out, tf2_listener)) {
      NODELET_ERROR(
        "[%s::transformPointCloud] Error converting first input dataset from %s to %s.",
        getName().c_str(), in->header.frame_id.c_str(), output_frame_.c_str());
      return;
    }
    */
    geometry_msgs::msg::TransformStamped transform;
    try {

      const auto time_point = tf2::TimePoint(std::chrono::milliseconds(0));
      transform = tf2_buffer_->lookupTransform(output_frame_, in->header.frame_id, time_point, tf2::durationFromSec(0.0));
      tf2::doTransform(*in, *out, transform);
    }
    catch (tf2::TransformException& ex)
      {
	RCLCPP_ERROR(this->get_logger(), "[transformPointCloud] Error converting first input dataset from %s to %s.",
		     in->header.frame_id.c_str(), output_frame_.c_str());
	return;

      }
  } else {
    //out = boost::make_shared<PointCloud2>(*in);
    *out = *in;
  }
}

void PointCloudConcatenateDataSynchronizerComponent::combineClouds(
  const PointCloud2::ConstSharedPtr & in1, const PointCloud2::ConstSharedPtr & in2, PointCloud2::SharedPtr & out)
{
  if (twist_ptr_queue_.empty()) {
    pcl::concatenatePointCloud(*in1, *in2, *out);
    out->header.stamp = std::min(in1->header.stamp, in2->header.stamp);
    return;
  }

  const auto old_stamp = std::min(in1->header.stamp, in2->header.stamp);
  auto old_twist_ptr_it = std::lower_bound(
    std::begin(twist_ptr_queue_), std::end(twist_ptr_queue_), old_stamp,
    [](const geometry_msgs::msg::TwistStamped::ConstSharedPtr & x_ptr, rclcpp::Time t) {
      return rclcpp::Time(x_ptr->header.stamp) < t;
    });
  old_twist_ptr_it =
    old_twist_ptr_it == twist_ptr_queue_.end() ? (twist_ptr_queue_.end() - 1) : old_twist_ptr_it;

  const auto new_stamp = std::max(in1->header.stamp, in2->header.stamp);
  auto new_twist_ptr_it = std::lower_bound(
    std::begin(twist_ptr_queue_), std::end(twist_ptr_queue_), new_stamp,
    [](const geometry_msgs::msg::TwistStamped::ConstSharedPtr & x_ptr, rclcpp::Time t) {
      return rclcpp::Time(x_ptr->header.stamp) < t;
    });
  new_twist_ptr_it =
    new_twist_ptr_it == twist_ptr_queue_.end() ? (twist_ptr_queue_.end() - 1) : new_twist_ptr_it;

  auto prev_time = old_stamp;
  double x = 0.0, y = 0.0, yaw = 0.0;
  for (auto twist_ptr_it = old_twist_ptr_it; twist_ptr_it != new_twist_ptr_it + 1; ++twist_ptr_it) {
    const double dt = (twist_ptr_it != new_twist_ptr_it)
      ? (rclcpp::Time((*twist_ptr_it)->header.stamp) - rclcpp::Time(prev_time)).seconds()
      : (rclcpp::Time(new_stamp) - rclcpp::Time(prev_time)).seconds();

    if (std::fabs(dt) > 0.1) {
      RCLCPP_WARN_STREAM_THROTTLE(
	get_logger(), *get_clock(), 10,
        "Time difference is too large. Cloud not interpolate. Please comfirm twist topic and timestamp");
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

  // TODO if output_frame_ is not base_link, we must transform

  if (rclcpp::Time(in1->header.stamp) > rclcpp::Time(in2->header.stamp)) {
    sensor_msgs::msg::PointCloud2::SharedPtr in1_t(new sensor_msgs::msg::PointCloud2());
    //pcl_ros::transformPointCloud(rotation_matrix, *in1, *in1_t);
    pcl::transformPointCloud(rotation_matrix, *in1, *in1_t);
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
  sensor_msgs::PointCloud2::Ptr concat_cloud_ptr_ = nullptr;
  std::string not_subscribed_topic_name = "";
  size_t concat_num = 0;

  for (const auto & e : cloud_stdmap_) {
    if (e.second != nullptr) {
      sensor_msgs::PointCloud2::Ptr transed_cloud_ptr(new sensor_msgs::PointCloud2());
      transformPointCloud(e.second, transed_cloud_ptr);
      if (concat_cloud_ptr_ == nullptr) {
        concat_cloud_ptr_ = transed_cloud_ptr;
      } else {
        PointCloudConcatenateDataSynchronizerNodelet::combineClouds(
          concat_cloud_ptr_, transed_cloud_ptr, concat_cloud_ptr_);
      }
      ++concat_num;

    } else {
      if (not_subscribed_topic_name.empty()) {
        not_subscribed_topic_name = e.first;
      } else {
        not_subscribed_topic_name += "," + e.first;
      }
    }
  }
  if (!not_subscribed_topic_name.empty()) {
    ROS_WARN_STREAM_THROTTLE(
      1, "Skipped " << not_subscribed_topic_name << ". Please confirm topic.");
  }

  pub_output_.publish(*concat_cloud_ptr_);

  std_msgs::Int32 concat_num_msg;
  concat_num_msg.data = concat_num;
  pub_concat_num_.publish(concat_num_msg);

  std_msgs::String not_subscribed_topic_name_msg;
  not_subscribed_topic_name_msg.data = not_subscribed_topic_name;
  pub_not_subscribed_topic_name_.publish(not_subscribed_topic_name_msg);

  cloud_stdmap_ = cloud_stdmap_tmp_;
  std::for_each(std::begin(cloud_stdmap_tmp_), std::end(cloud_stdmap_tmp_), [](auto & e) {
    e.second = nullptr;
  });
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudConcatenateDataSynchronizerComponent::convertToXYZCloud(
  const sensor_msgs::PointCloud2 & input_cloud,
  sensor_msgs::PointCloud2 & output_cloud)
{
  pcl::PointCloud<pcl::PointXYZ> tmp_xyz_cloud;
  pcl::fromROSMsg(input_cloud, tmp_xyz_cloud);
  pcl::toROSMsg(tmp_xyz_cloud, output_cloud);
  output_cloud.header = input_cloud.header;
}

void PointCloudConcatenateDataSynchronizerComponent::cloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr input_ptr, const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);

  sensor_msgs::PointCloud2 xyz_cloud;
  convertToXYZCloud(*input_ptr, xyz_cloud);
  sensor_msgs::PointCloud2::ConstPtr xyz_input_ptr(new sensor_msgs::PointCloud2(xyz_cloud));

  const bool is_already_subscribed_this = (cloud_stdmap_[topic_name] != nullptr);
  const bool is_already_subscribed_tmp = std::any_of(
    std::begin(cloud_stdmap_tmp_), std::end(cloud_stdmap_tmp_),
    [](const auto & e) { return e.second != nullptr; });

  if (is_already_subscribed_this) {

    cloud_stdmap_tmp_[topic_name] = xyz_input_ptr;

    if (!is_already_subscribed_tmp) {
      timer_.setPeriod(ros::Duration(timeout_sec_), true);
      timer_.start();
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
      std::for_each(std::begin(cloud_stdmap_tmp_), std::end(cloud_stdmap_tmp_),
        [](auto & e) { e.second = nullptr; });

      timer_.stop();
      publish();
    }
  }
}

void PointCloudConcatenateDataSynchronizerComponent::timer_callback(const ros::TimerEvent &)
{
  timer_.stop();
  if(mutex_.try_lock()) {
    publish();
    mutex_.unlock();
  }
  else {
    timer_.setPeriod(ros::Duration(0.01), true);
    timer_.start();
  }
}

void PointCloudConcatenateDataSynchronizerComponent::twist_callback(
								    const geometry_msgs::msg::TwistStamped::SharedPtr input)
{
  // if rosbag restart, clear buffer
  if (!twist_ptr_queue_.empty()) {
    if (twist_ptr_queue_.front()->header.stamp > input->header.stamp) {
      twist_ptr_queue_.clear();
    }
  }

  // pop old data
  while (!twist_ptr_queue_.empty()) {
    if (twist_ptr_queue_.front()->header.stamp + ros::Duration(1.0) > input->header.stamp) {
      break;
    }
    twist_ptr_queue_.pop_front();
  }
  twist_ptr_queue_.push_back(input);
}

}  // namespace pointcloud_preprocessor


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent)
//PLUGINLIB_EXPORT_CLASS(
//  pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerNodelet, nodelet::Nodelet);
