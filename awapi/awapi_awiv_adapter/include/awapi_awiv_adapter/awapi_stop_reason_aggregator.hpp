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

#ifndef AWAPI_AWIV_ADAPTER__AWAPI_STOP_REASON_AGGREGATOR_HPP_
#define AWAPI_AWIV_ADAPTER__AWAPI_STOP_REASON_AGGREGATOR_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "awapi_awiv_adapter/awapi_autoware_util.hpp"

namespace autoware_api
{
class AutowareIvStopReasonAggregator
{
public:
  AutowareIvStopReasonAggregator(
    rclcpp::Node & node, const double timeout, const double thresh_dist_to_stop_pose);
  autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr updateStopReasonArray(
    const autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr & msg_ptr,
    const AutowareInfo & aw_info);

private:
  void applyUpdate(
    const autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr & msg_ptr,
    const AutowareInfo & aw_info);
  bool checkMatchingReason(
    const autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr & msg_stop_reason_array,
    const autoware_planning_msgs::msg::StopReasonArray & stop_reason_array);
  void applyTimeOut();
  void appendStopReasonToArray(
    const autoware_planning_msgs::msg::StopReason & stop_reason,
    autoware_planning_msgs::msg::StopReasonArray * stop_reason_array, const AutowareInfo & aw_info);
  autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr makeStopReasonArray(
    const AutowareInfo & aw_info);
  autoware_planning_msgs::msg::StopReason inputStopDistToStopReason(
    const autoware_planning_msgs::msg::StopReason & stop_reason, const AutowareInfo & aw_info);
  double calcStopDistToStopFactor(
    const autoware_planning_msgs::msg::StopFactor & stop_factor, const AutowareInfo & aw_info);
  autoware_planning_msgs::msg::StopReason getNearStopReason(
    const autoware_planning_msgs::msg::StopReason & stop_reason, const AutowareInfo & aw_info);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  double timeout_;
  double thresh_dist_to_stop_pose_;
  std::vector<autoware_planning_msgs::msg::StopReasonArray> stop_reason_array_vec_;
};

}  // namespace autoware_api

#endif  // AWAPI_AWIV_ADAPTER__AWAPI_STOP_REASON_AGGREGATOR_HPP_
