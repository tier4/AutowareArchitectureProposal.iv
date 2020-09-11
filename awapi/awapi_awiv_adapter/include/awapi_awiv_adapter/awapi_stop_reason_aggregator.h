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

#include <awapi_awiv_adapter/awapi_autoware_util.h>

namespace autoware_api
{
class AutowareIvStopReasonAggregator
{
public:
  AutowareIvStopReasonAggregator(const double timeout, const double thresh_dist_to_stop_pose);
  autoware_planning_msgs::StopReasonArray::ConstPtr updateStopReasonArray(
    const autoware_planning_msgs::StopReasonArray::ConstPtr & msg_ptr,
    const AutowareInfo & aw_info);

private:
  void applyUpdate(
    const autoware_planning_msgs::StopReasonArray::ConstPtr & msg_ptr,
    const AutowareInfo & aw_info);
  bool checkMatchingReason(
    const autoware_planning_msgs::StopReasonArray::ConstPtr & msg_stop_reason_array,
    const autoware_planning_msgs::StopReasonArray & stop_reason_array);
  void applyTimeOut();
  void appendStopReasonToArray(
    const autoware_planning_msgs::StopReason & stop_reason,
    autoware_planning_msgs::StopReasonArray * stop_reason_array, const AutowareInfo & aw_info);
  autoware_planning_msgs::StopReasonArray::ConstPtr makeStopReasonArray(
    const AutowareInfo & aw_info);
  autoware_planning_msgs::StopReason inputStopDistToStopReason(
    const autoware_planning_msgs::StopReason & stop_reason, const AutowareInfo & aw_info);
  double calcStopDistToStopFactor(
    const autoware_planning_msgs::StopFactor & stop_factor, const AutowareInfo & aw_info);
  autoware_planning_msgs::StopReason getNearStopReason(
    const autoware_planning_msgs::StopReason & stop_reason, const AutowareInfo & aw_info);

  double timeout_;
  double thresh_dist_to_stop_pose_;
  std::vector<autoware_planning_msgs::StopReasonArray> stop_reason_array_vec_;
};

}  // namespace autoware_api
