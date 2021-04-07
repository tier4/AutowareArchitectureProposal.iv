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

#ifndef PATH_SAVER__PATH_SAVER_NODE_HPP_
#define PATH_SAVER__PATH_SAVER_NODE_HPP_
#include <string>
#include <vector>
#include <map>
#include <memory>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"


class PathSaverNode : public rclcpp::Node
{
private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_ptr_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_ptr_;        //!< @brief measured pose
  geometry_msgs::msg::Twist last_saved_twist_;
  geometry_msgs::msg::Pose last_saved_pose_;
  std::shared_ptr<rclcpp::Time> last_saved_time_;
  std::ofstream ofs_;
  double dist_threshold_;
  double time_threshold_;

  void callbackTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void timerCallback();
  void updateCurrentPose();

public:
  explicit PathSaverNode(const rclcpp::NodeOptions & node_options);
  ~PathSaverNode() = default;
};

#endif  // PATH_SAVER__PATH_SAVER_NODE_HPP_
