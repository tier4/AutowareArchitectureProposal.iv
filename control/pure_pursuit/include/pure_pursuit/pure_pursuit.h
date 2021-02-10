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
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"
#include "Eigen/Geometry"

#include <geometry_msgs/msg/pose.hpp>
#define PURE_PURSUIT_LOGGER "pure_pursuit"

namespace planning_utils
{
class PurePursuit
{
public:
  PurePursuit()
  : lookahead_distance_(0.0), clst_thr_dist_(3.0), clst_thr_ang_(M_PI / 4) {}
  ~PurePursuit() = default;


  rclcpp::Logger logger = rclcpp::get_logger(PURE_PURSUIT_LOGGER);
  // setter
  void setCurrentPose(const geometry_msgs::msg::Pose & msg);
  void setWaypoints(const std::vector<geometry_msgs::msg::Pose> & msg);
  void setLookaheadDistance(double ld) {lookahead_distance_ = ld;}
  void setClosestThreshold(double clst_thr_dist, double clst_thr_ang)
  {
    clst_thr_dist_ = clst_thr_dist;
    clst_thr_ang_ = clst_thr_ang;
  }

  // getter
  geometry_msgs::msg::Point getLocationOfNextWaypoint() const {return loc_next_wp_;}
  geometry_msgs::msg::Point getLocationOfNextTarget() const {return loc_next_tgt_;}

  bool isDataReady();
  std::pair<bool, double> run();  // calculate curvature

private:
  // variables for debug
  geometry_msgs::msg::Point loc_next_wp_;
  geometry_msgs::msg::Point loc_next_tgt_;

  // variables got from outside
  double lookahead_distance_, clst_thr_dist_, clst_thr_ang_;
  std::shared_ptr<std::vector<geometry_msgs::msg::Pose>> curr_wps_ptr_;
  std::shared_ptr<geometry_msgs::msg::Pose> curr_pose_ptr_;

  // functions
  int32_t findNextPointIdx(int32_t search_start_idx);
  std::pair<bool, geometry_msgs::msg::Point> lerpNextTarget(int32_t next_wp_idx);
};

}  // namespace planning_utils
