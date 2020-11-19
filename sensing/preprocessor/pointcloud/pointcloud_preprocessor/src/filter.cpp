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
 * $Id: filter.cpp 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#include "pointcloud_preprocessor/filter.h"
#include <pcl/io/io.h>
#include "pcl_ros/transforms.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pointcloud_preprocessor::Filter::Filter(
  const std::string & filter_name, const rclcpp::NodeOptions & options)
: Node(filter_name, options)
{
  // Set parameters (moved from NodeletLazy onInit)
  {
    max_queue_size_ = static_cast<int>(declare_parameter("max_queue_size").get<std::size_t>());

    // ---[ Optional parameters
    use_indices_ = static_cast<bool>(declare_parameter("use_indices").get<bool>());
    latched_indices_ = static_cast<bool>(declare_parameter("latched_indices").get<bool>());
    approximate_sync_ = static_cast<bool>(declare_parameter("approximate_sync").get<bool>());

    RCLCPP_DEBUG(
      this->get_logger(),
      "[pointcloud_preprocessor] Filter (as Component) successfully created with the following "
      "parameters:\n"
      " - approximate_sync : %s\n"
      " - use_indices      : %s\n"
      " - latched_indices  : %s\n"
      " - max_queue_size   : %d",
      (approximate_sync_) ? "true" : "false", (use_indices_) ? "true" : "false",
      (latched_indices_) ? "true" : "false", max_queue_size_);
  }

  // Set publisher
  {
    pub_output_ = this->create_publisher<PointCloud2>("output", max_queue_size_);
  }

  // Set subscriber
  {
    if (use_indices_) {
      // Subscribe to the input using a filter
      sub_input_filter_.subscribe(
        this, "input", rclcpp::QoS{max_queue_size_}.get_rmw_qos_profile());
      sub_indices_filter_.subscribe(
        this, "indices", rclcpp::QoS{max_queue_size_}.get_rmw_qos_profile());

      if (approximate_sync_) {
        sync_input_indices_a_ = boost::make_shared<ApproximateTimeSyncPolicy>(max_queue_size_);
        sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
        sync_input_indices_a_->registerCallback(
          bind(&Filter::input_indices_callback, this, _1, _2));
      } else {
        sync_input_indices_e_ = boost::make_shared<ExactTimeSyncPolicy>(max_queue_size_);
        sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
        sync_input_indices_e_->registerCallback(
          bind(&Filter::input_indices_callback, this, _1, _2));
      }
    } else {
      // Subscribe in an old fashion to input only (no filters)
      // CAN'T use auto-type here.
      std::function<void(const PointCloud2ConstPtr msg)> cb = std::bind(
        &Filter::input_indices_callback, this, std::placeholders::_1, PointIndicesConstPtr());
      sub_input_ = create_subscription<PointCloud2>("input", rclcpp::QoS{max_queue_size_}, cb);
    }
  }
 
  // Set tf_listener, tf_buffer.
  setupTF();

  RCLCPP_DEBUG(
    this->get_logger(), "[%s::onInit] Nodelet successfully created.", filter_field_name_.c_str());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pointcloud_preprocessor::Filter::setupTF()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false);
  auto cti = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(cti);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pointcloud_preprocessor::Filter::computePublish(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices)
{
  PointCloud2 output;
  // Call the virtual method in the child
  filter(input, indices, output);

  PointCloud2::SharedPtr cloud_tf(new PointCloud2(output));  // set the output by default
  // Check whether the user has given a different output TF frame
  if (!tf_output_frame_.empty() && output.header.frame_id != tf_output_frame_) {
    RCLCPP_DEBUG(
      this->get_logger(), "[%s::computePublish] Transforming output dataset from %s to %s.",
      filter_field_name_.c_str(), output.header.frame_id.c_str(), tf_output_frame_.c_str());
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    if (!pcl_ros::transformPointCloud(tf_output_frame_, output, cloud_transformed, *tf_buffer_)) {
      RCLCPP_ERROR(this->get_logger(),
        "[%s::computePublish] Error converting output dataset from %s to %s.", filter_field_name_.c_str(),
        output.header.frame_id.c_str(), tf_output_frame_.c_str());
      return;
    }
    cloud_tf.reset(new PointCloud2(cloud_transformed));
  }
  if (tf_output_frame_.empty() && output.header.frame_id != tf_input_orig_frame_)
  // no tf_output_frame given, transform the dataset to its original frame
  {
    RCLCPP_DEBUG(
      this->get_logger(), "[%s::computePublish] Transforming output dataset from %s back to %s.",
      filter_field_name_.c_str(), output.header.frame_id.c_str(), tf_input_orig_frame_.c_str());
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    if (!pcl_ros::transformPointCloud(
          tf_input_orig_frame_, output, cloud_transformed, *tf_buffer_)) {
      RCLCPP_ERROR(this->get_logger(),
        "[%s::computePublish] Error converting output dataset from %s back to %s.",
        filter_field_name_.c_str(), output.header.frame_id.c_str(), tf_input_orig_frame_.c_str());
      return;
    }
    cloud_tf.reset(new PointCloud2(cloud_transformed));
  }

  // Copy timestamp to keep it
  cloud_tf->header.stamp = input->header.stamp;

  // Publish a boost shared ptr
  pub_output_->publish(*cloud_tf);
}

//////////////////////////////////////////////////////////////////////////////////////////////
// void pointcloud_preprocessor::Filter::config_callback(
//   pcl_ros::FilterConfig & config, uint32_t level)
// {
//   // The following parameters are updated automatically for all PCL_ROS Nodelet Filters as they are inexistent in PCL
//   if (tf_input_frame_ != config.input_frame) {
//     tf_input_frame_ = config.input_frame;
//     RCLCPP_DEBUG(
//       this->get_logger(), "[%s::config_callback] Setting the input TF frame to: %s.",
//       filter_field_name_.c_str(), tf_input_frame_.c_str());
//   }
//   if (tf_output_frame_ != config.output_frame) {
//     tf_output_frame_ = config.output_frame;
//     RCLCPP_DEBUG(
//       this->get_logger(), "[%s::config_callback] Setting the output TF frame to: %s.",
//       filter_field_name_.c_str(), tf_output_frame_.c_str());
//   }
// }

//////////////////////////////////////////////////////////////////////////////////////////////
void pointcloud_preprocessor::Filter::input_indices_callback(
  const PointCloud2ConstPtr cloud, const PointIndicesConstPtr indices)
{
  // If cloud is given, check if it's valid
  if (!isValid(cloud)) {
    RCLCPP_ERROR(
      this->get_logger(), "[%s::input_indices_callback] Invalid input!", filter_field_name_.c_str());
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid(indices)) {
    RCLCPP_ERROR(
      this->get_logger(), "[%s::input_indices_callback] Invalid indices!", filter_field_name_.c_str());
    return;
  }

  /// DEBUG
  if (indices)
    RCLCPP_DEBUG(
      this->get_logger(),
      "[%s::input_indices_callback]\n"
      "                                 - PointCloud with %d data points (%s), stamp %f, and frame "
      "%s on inpu topic"
      "received.\n"
      "                                 - PointIndices with %zu values, stamp %f, and frame %s on "
      "indices topic received.",
      filter_field_name_.c_str(), cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
      rclcpp::Time(cloud->header.stamp).seconds(), cloud->header.frame_id.c_str(),
      indices->indices.size(), rclcpp::Time(indices->header.stamp).seconds(),
      indices->header.frame_id.c_str());
  else
    RCLCPP_DEBUG(
      this->get_logger(),
      "[%s::input_indices_callback] PointCloud with %d data points and frame %s on input topic "
      "received.",
      filter_field_name_.c_str(), cloud->width * cloud->height, cloud->header.frame_id.c_str());
  ///

  // Check whether the user has given a different input TF frame
  tf_input_orig_frame_ = cloud->header.frame_id;
  PointCloud2ConstPtr cloud_tf;
  if (!tf_input_frame_.empty() && cloud->header.frame_id != tf_input_frame_) {
    RCLCPP_DEBUG(
      this->get_logger(), "[%s::input_indices_callback] Transforming input dataset from %s to %s.",
      filter_field_name_.c_str(), cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
    // Save the original frame ID
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    auto status = tf_buffer_->waitForTransform(
      tf_input_frame_, cloud->header.frame_id, cloud->header.stamp, std::chrono::milliseconds(0),
      [this](auto) {}).wait_for(std::chrono::milliseconds(1000));
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(
        this->get_logger(), "[%s::input_indices_callback] timeout tf", filter_field_name_.c_str());
      return;
    }

    if (!pcl_ros::transformPointCloud(tf_input_frame_, *cloud, cloud_transformed, *tf_buffer_)) {
      RCLCPP_ERROR(this->get_logger(),
        "[%s::input_indices_callback] Error converting input dataset from %s to %s.",
        filter_field_name_.c_str(), cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
      return;
    }
    cloud_tf = std::make_shared<PointCloud2>(cloud_transformed);
  } else {
    cloud_tf = cloud;
  }
  // Need setInputCloud () here because we have to extract x/y/z
  IndicesPtr vindices;
  if (indices) vindices.reset(new std::vector<int>(indices->indices));

  computePublish(cloud_tf, vindices);
}
