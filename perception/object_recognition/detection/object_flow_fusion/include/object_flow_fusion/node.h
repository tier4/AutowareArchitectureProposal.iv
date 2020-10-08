/*
 * Copyright 2020 TierIV. All rights reserved.
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

#include <iostream>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <autoware_perception_msgs/DynamicObjectWithFeatureArray.h>
#include "object_flow_fusion.h"

namespace object_flow_fusion
{
class ObjectFlowFusionNode
{
public:
  ObjectFlowFusionNode();
  void callback(
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr& object_msg,
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr& flow_msg);
private:
  typedef message_filters::sync_policies::ApproximateTime<
  autoware_perception_msgs::DynamicObjectWithFeatureArray,
  autoware_perception_msgs::DynamicObjectWithFeatureArray
  > ApproximateSync;

  typedef message_filters::sync_policies::ExactTime<
    autoware_perception_msgs::DynamicObjectWithFeatureArray,
    autoware_perception_msgs::DynamicObjectWithFeatureArray
    > Sync;

  boost::shared_ptr<message_filters::Synchronizer<ApproximateSync> > approximate_sync_;
  boost::shared_ptr<message_filters::Synchronizer<Sync> > sync_;
  message_filters::Subscriber<autoware_perception_msgs::DynamicObjectWithFeatureArray> object_sub_;
  message_filters::Subscriber<autoware_perception_msgs::DynamicObjectWithFeatureArray> flow_sub_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_;
  bool is_approximate_sync_;
  bool use_flow_pose_;
  float flow_vel_thresh_;
  std::shared_ptr<ObjectFlowFusion> object_flow_fusion_;

};
} // object_flow_fusion
