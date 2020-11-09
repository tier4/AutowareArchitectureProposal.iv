/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: pcl_nodelet.h 33238 2010-03-11 00:46:58Z rusu $
 *
 */

/**

\author Radu Bogdan Rusu
* port to ROS2 component class
* Simon Thompson
**/

#ifndef PCL_COMPONET_H_
#define PCL_COMPONENT_H_

#include <sensor_msgs/msg/point_cloud2.h>
// PCL includes
#include <pcl_msgs/msg/point_indices.h>
#include <pcl_msgs/msg/model_coefficients.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//#include "pcl_ros/point_cloud.h"
// ROS Nodelet includes
//#include <nodelet_topic_tools/nodelet_lazy.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

// Include TF
//#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using pcl_conversions::fromPCL;

namespace pointcloud_preprocessor
{
  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b PCLComponent represents the base PCL Component class. All PCL componennts should inherit from this class. */
class PCLComponent : public rclcpp::Node

  {
    public:
    typedef sensor_msgs::msg::PointCloud2 PointCloud2;

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    //typedef boost::shared_ptr<PointCloud> PointCloudPtr;
    //typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

    typedef pcl_msgs::msg::PointIndices PointIndices;
    typedef PointIndices::SharedPtr PointIndicesPtr;
    typedef PointIndices::ConstSharedPtr PointIndicesConstPtr;

    typedef pcl_msgs::msg::ModelCoefficients ModelCoefficients;
    typedef ModelCoefficients::SharedPtr ModelCoefficientsPtr;
    typedef ModelCoefficients::ConstSharedPtr ModelCoefficientsConstPtr;
    
    typedef pcl::IndicesPtr IndicesPtr;
    typedef pcl::IndicesConstPtr IndicesConstPtr;
    
    /** \brief Empty constructor. */

    explicit PCLComponent (const rclcpp::NodeOptions & options) : Node("pcl_component", options),
use_indices_ (false), latched_indices_ (false), 
      max_queue_size_ (3), approximate_sync_ (false) {};
    
  protected:
    /** \brief Set to true if point indices are used.
       *
       * When receiving a point cloud, if use_indices_ is false, the entire
       * point cloud is processed for the given operation. If use_indices_ is
       * true, then the ~indices topic is read to get the vector of point
       * indices specifying the subset of the point cloud that will be used for
       * the operation. In the case where use_indices_ is true, the ~input and
       * ~indices topics must be synchronised in time, either exact or within a
       * specified jitter. See also @ref latched_indices_ and approximate_sync.
       **/
      bool use_indices_;
      /** \brief Set to true if the indices topic is latched.
       *
       * If use_indices_ is true, the ~input and ~indices topics generally must
       * be synchronised in time. By setting this flag to true, the most recent
       * value from ~indices can be used instead of requiring a synchronised
       * message.
       **/
      bool latched_indices_;

      // ROS2 port: message filters is already ported to ROS2!
      /** \brief The message filter subscriber for PointCloud2. */
      message_filters::Subscriber<PointCloud> sub_input_filter_;

      /** \brief The message filter subscriber for PointIndices. */
      message_filters::Subscriber<PointIndices> sub_indices_filter_;

    // ROS2 port - type of publisher message not defined....OK? -> No need to declare message type
    // The publisher is not used in this base class -> so leave implmentation to child class???
      /** \brief The output PointCloud publisher. */
    //ros::Publisher pub_output_;
    //    rclcpp::Publisher::SharedPtr pub_output_;

      /** \brief The maximum queue size (default: 3). */
      int max_queue_size_;

      /** \brief True if we use an approximate time synchronizer versus an exact one (false by default). */
      bool approximate_sync_;

      /** \brief TF listener object. */
      //tf::TransformListener tf_listener_;
      std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

      /** \brief Test whether a given PointCloud message is "valid" (i.e., has points, and width and height are non-zero).
        * \param cloud the point cloud to test
        * \param topic_name an optional topic name (only used for printing, defaults to "input")
	* ROS2 port: could not find equivalent to pnh->resolveName to print out topic name
        */
      inline bool
      isValid (const PointCloud2::ConstSharedPtr &cloud, const std::string &topic_name = "input")
      {
        if (cloud->width * cloud->height * cloud->point_step != cloud->data.size ())
        {
          RCLCPP_WARN (this->get_logger(), "Invalid PointCloud (data = %zu, width = %d, height = %d, step = %d) with stamp %f, and frame %sreceived!", cloud->data.size (), cloud->width, cloud->height, cloud->point_step, rclcpp::Time(cloud->header.stamp).seconds(), cloud->header.frame_id.c_str ());

          return (false);
        }
        return (true);
      }


      /** \brief Test whether a given PointIndices message is "valid" (i.e., has values).
        * \param indices the point indices message to test
        * \param topic_name an optional topic name (only used for printing, defaults to "indices")
        */
      inline bool
      isValid (const PointIndicesConstPtr &/*indices*/, const std::string &/*topic_name*/ = "indices")
      {
        /*if (indices->indices.empty ())
        {
          NODELET_WARN ("[%s] Empty indices (values = %zu) with stamp %f, and frame %s on topic %s received!", getName ().c_str (), indices->indices.size (), indices->header.stamp.toSec (), indices->header.frame_id.c_str (), pnh_->resolveName (topic_name).c_str ());
          return (true);
        }*/
        return (true);
      }

      /** \brief Test whether a given ModelCoefficients message is "valid" (i.e., has values).
        * \param model the model coefficients to test
        * \param topic_name an optional topic name (only used for printing, defaults to "model")
        */
      inline bool
      isValid (const ModelCoefficientsConstPtr &/*model*/, const std::string &/*topic_name*/ = "model")
      {
        /*if (model->values.empty ())
        {
          NODELET_WARN ("[%s] Empty model (values = %zu) with stamp %f, and frame %s on topic %s received!", getName ().c_str (), model->values.size (), model->header.stamp.toSec (), model->header.frame_id.c_str (), pnh_->resolveName (topic_name).c_str ());
          return (false);
        }*/
        return (true);
      }

    // remove these as moving towards Components which do not have these festures at the moment
      /** \brief Lazy transport subscribe/unsubscribe routine. It is optional for backward compatibility. */
    //   virtual void subscribe () {}
    //  virtual void unsubscribe () {}

      /** \brief Nodelet initialization routine. Reads in global parameters used by all nodelets. */
      virtual void
      onInit ()
      {
	//   nodelet_topic_tools::NodeletLazy::onInit();

        // Parameters that we care about only at startup
	//        pnh_->getParam ("max_queue_size", max_queue_size_);

	max_queue_size_ = static_cast<int>(declare_parameter("max_queue_size").get<std::size_t>());

        // ---[ Optional parameters
        // pnh_->getParam ("use_indices", use_indices_);
        // pnh_->getParam ("latched_indices", latched_indices_);
        // pnh_->getParam ("approximate_sync", approximate_sync_);

	use_indices_ = static_cast<bool>(declare_parameter("use_indices").get<bool>());
	latched_indices_ = static_cast<bool>(declare_parameter("latched_indices").get<bool>());
	approximate_sync_ = static_cast<bool>(declare_parameter("approximate_sync").get<bool>());
	
        RCLCPP_DEBUG (this->get_logger(), "[onInit] PCL Nodelet (as Component)  successfully created with the following parameters:\n"
            " - approximate_sync : %s\n"
            " - use_indices      : %s\n"
            " - latched_indices  : %s\n"
            " - max_queue_size   : %d",
            (approximate_sync_) ? "true" : "false",
            (use_indices_) ? "true" : "false", 
            (latched_indices_) ? "true" : "false", 
            max_queue_size_);
      }

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef PCL_COMPONENT_H_
