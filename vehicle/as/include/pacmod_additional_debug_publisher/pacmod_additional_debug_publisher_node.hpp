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
#include <can_msgs/Frame.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

class PacmodAdditionalDebugPublisherNode
{
private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher debug_pub_;
  ros::Publisher accel_cal_rpt_pub_;
  ros::Publisher brake_cal_rpt_pub_;
  ros::Publisher steer_cal_rpt_pub_;
  ros::Subscriber sub_;
  std_msgs::Float32MultiArray debug_value_;
  std_msgs::Float32MultiArray accel_cal_rpt_;
  std_msgs::Float32MultiArray brake_cal_rpt_;
  std_msgs::Float32MultiArray steer_cal_rpt_;
  bool calibration_active_;
  void canTxCallback(const can_msgs::FrameConstPtr & msg);

public:
  PacmodAdditionalDebugPublisherNode();
  ~PacmodAdditionalDebugPublisherNode(){};
};
