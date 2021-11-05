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

#ifndef LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_HPP_
#define LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_HPP_

#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <boost/optional.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace lane_departure_checker
{
using autoware_auto_planning_msgs::msg::HADMapRoute;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_utils::LinearRing2d;
using autoware_utils::PoseDeviation;
using TrajectoryPointArray = std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>;

struct Param
{
  double footprint_margin_scale;
  double resample_interval;
  double max_deceleration;
  double delay_time;
  double max_lateral_deviation;
  double max_longitudinal_deviation;
  double max_yaw_deviation_deg;
};

struct Input
{
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose{};
  nav_msgs::msg::Odometry::ConstSharedPtr current_odom{};
  lanelet::LaneletMapPtr lanelet_map{};
  HADMapRoute::ConstSharedPtr route{};
  lanelet::ConstLanelets route_lanelets{};
  Trajectory::ConstSharedPtr reference_trajectory{};
  Trajectory::ConstSharedPtr predicted_trajectory{};
};

struct Output
{
  std::map<std::string, double> processing_time_map{};
  bool will_leave_lane{};
  bool is_out_of_lane{};
  PoseDeviation trajectory_deviation{};
  lanelet::ConstLanelets candidate_lanelets{};
  Trajectory resampled_trajectory{};
  std::vector<LinearRing2d> vehicle_footprints{};
  std::vector<LinearRing2d> vehicle_passing_areas{};
};

class LaneDepartureChecker
{
public:
  Output update(const Input & input);

  void setParam(const Param & param, const vehicle_info_util::VehicleInfo vehicle_info)
  {
    param_ = param;
    vehicle_info_ptr_ = std::make_shared<vehicle_info_util::VehicleInfo>(vehicle_info);
  }

  void setParam(const Param & param) { param_ = param; }

private:
  Param param_;
  std::shared_ptr<vehicle_info_util::VehicleInfo> vehicle_info_ptr_;

  static PoseDeviation calcTrajectoryDeviation(
    const Trajectory & trajectory, const geometry_msgs::msg::Pose & pose);

  //! This function assumes the input trajectory is sampled dense enough
  static Trajectory resampleTrajectory(const Trajectory & trajectory, const double interval);

  static Trajectory cutTrajectory(const Trajectory & trajectory, const double length);

  std::vector<LinearRing2d> createVehicleFootprints(
    const geometry_msgs::msg::PoseWithCovariance & covariance, const Trajectory & trajectory,
    const Param & param);

  static std::vector<LinearRing2d> createVehiclePassingAreas(
    const std::vector<LinearRing2d> & vehicle_footprints);

  static bool willLeaveLane(
    const lanelet::ConstLanelets & candidate_lanelets,
    const std::vector<LinearRing2d> & vehicle_footprints);

  static bool isOutOfLane(
    const lanelet::ConstLanelets & candidate_lanelets, const LinearRing2d & vehicle_footprint);
};
}  // namespace lane_departure_checker

#endif  // LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_HPP_
