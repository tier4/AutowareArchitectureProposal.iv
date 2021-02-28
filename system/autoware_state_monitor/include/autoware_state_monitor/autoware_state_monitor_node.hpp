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

#ifndef AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_MONITOR_NODE_HPP_
#define AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_MONITOR_NODE_HPP_

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "autoware_state_monitor/autoware_state.hpp"
#include "autoware_state_monitor/config.hpp"
#include "autoware_state_monitor/state_machine.hpp"

#include "autoware_planning_msgs/msg/route.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_control_msgs/msg/emergency_mode.hpp"
#include "autoware_system_msgs/msg/autoware_state.hpp"
#include "autoware_vehicle_msgs/msg/control_mode.hpp"
#include "autoware_vehicle_msgs/msg/engage.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp_generic/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"

class AutowareStateMonitorNode : public rclcpp::Node
{
public:
  AutowareStateMonitorNode();

private:
  // Parameter
  double update_rate_;
  bool disengage_on_route_;
  bool disengage_on_goal_;

  std::vector<TopicConfig> topic_configs_;
  std::vector<ParamConfig> param_configs_;
  std::vector<TfConfig> tf_configs_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_;

  // Subscriber
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Engage>::SharedPtr sub_autoware_engage_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ControlMode>::SharedPtr
    sub_vehicle_control_mode_;
  rclcpp::Subscription<autoware_control_msgs::msg::EmergencyMode>::SharedPtr sub_is_emergency_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Route>::SharedPtr sub_route_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;

  void onAutowareEngage(const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg);
  void onVehicleControlMode(const autoware_vehicle_msgs::msg::ControlMode::ConstSharedPtr msg);
  void onIsEmergency(const autoware_control_msgs::msg::EmergencyMode::ConstSharedPtr msg);
  void onRoute(const autoware_planning_msgs::msg::Route::ConstSharedPtr msg);
  void onTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);

  // Topic Buffer
  void onTopic(
    const std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string & topic_name);
  void registerTopicCallback(
    const std::string & topic_name, const std::string & topic_type, const bool transient_local);

  std::map<std::string, rclcpp_generic::GenericSubscription::SharedPtr> sub_topic_map_;
  std::map<std::string, std::deque<rclcpp::Time>> topic_received_time_buffer_;

  // Service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_shutdown_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_route_;

  bool onShutdownService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool onResetRouteService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);


  // Publisher
  rclcpp::Publisher<autoware_system_msgs::msg::AutowareState>::SharedPtr pub_autoware_state_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Engage>::SharedPtr pub_autoware_engage_;

  bool isEngaged();
  void setDisengage();

  // Timer
  void onTimer();
  rclcpp::TimerBase::SharedPtr timer_;

  // Stats
  TopicStats getTopicStats() const;
  ParamStats getParamStats() const;
  TfStats getTfStats() const;

  // State Machine
  std::shared_ptr<StateMachine> state_machine_;
  StateInput state_input_;
  StateParam state_param_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;

  void setupDiagnosticUpdater();
  void checkTopicStatus(
    diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & module_name);
  void checkTFStatus(
    diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & module_name);
};

#endif  // AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_MONITOR_NODE_HPP_
