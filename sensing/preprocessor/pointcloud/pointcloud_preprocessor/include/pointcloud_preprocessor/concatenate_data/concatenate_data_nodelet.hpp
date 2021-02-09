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

#ifndef POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CONCATENATE_DATA_NODELET_HPP_
#define POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CONCATENATE_DATA_NODELET_HPP_

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// ROS includes
#include "message_filters/pass_through.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

namespace pointcloud_preprocessor
{
/** \brief @b PointCloudConcatenateDataSynchronizerComponent is a special form of data
 * synchronizer: it listens for a set of input PointCloud messages on the same topic,
 * checks their timestamps, and concatenates their fields together into a single
 * PointCloud output message.
 * \author Radu Bogdan Rusu
 */
class PointCloudConcatenateDataSynchronizerComponent : public rclcpp::Node
{
public:
  typedef sensor_msgs::msg::PointCloud2 PointCloud2;

  /** \brief constructor. */
  explicit PointCloudConcatenateDataSynchronizerComponent(const rclcpp::NodeOptions & node_options);

  /** \brief constructor.
   * \param queue_size the maximum queue size
   */
  PointCloudConcatenateDataSynchronizerComponent(
    const rclcpp::NodeOptions & node_options, int queue_size);

  /** \brief Empty destructor. */
  virtual ~PointCloudConcatenateDataSynchronizerComponent() {}

private:
  /** \brief The output PointCloud publisher. */
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_concat_num_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_not_subscribed_topic_name_;

  /** \brief The maximum number of messages that we can store in the queue. */
  int maximum_queue_size_ = 3;

  double timeout_sec_ = 0.1;

  /** \brief A vector of subscriber. */
  std::vector<rclcpp::Subscription<PointCloud2>::SharedPtr> filters_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;

  rclcpp::TimerBase::SharedPtr timer_;

  /** \brief Output TF frame the concatenated points should be transformed to. */
  std::string output_frame_;

  /** \brief Input point cloud topics. */
  // XmlRpc::XmlRpcValue input_topics_;
  std::vector<std::string> input_topics_;

  /** \brief TF listener object. */
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> twist_ptr_queue_;

  std::map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> cloud_stdmap_;
  std::map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> cloud_stdmap_tmp_;
  std::mutex mutex_;

  void transformPointCloud(const PointCloud2::ConstSharedPtr & in, PointCloud2::SharedPtr & out);
  void combineClouds(
    const PointCloud2::ConstSharedPtr & in1, const PointCloud2::ConstSharedPtr & in2,
    PointCloud2::SharedPtr & out);
  void publish();

  void convertToXYZCloud(
    const sensor_msgs::msg::PointCloud2 & input_cloud,
    sensor_msgs::msg::PointCloud2 & output_cloud);
  void cloud_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_ptr,
    const std::string & topic_name);
  void twist_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr input);
  void timer_callback();
};
}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CONCATENATE_DATA_NODELET_HPP_
