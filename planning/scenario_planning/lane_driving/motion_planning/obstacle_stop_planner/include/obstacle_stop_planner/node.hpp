// Copyright 2019 Autoware Foundation
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

#ifndef OBSTACLE_STOP_PLANNER__NODE_HPP_
#define OBSTACLE_STOP_PLANNER__NODE_HPP_

#include <map>
#include <memory>
#include <vector>

#include "autoware_debug_msgs/msg/bool_stamped.hpp"
#include "autoware_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "autoware_debug_msgs/msg/float32_stamped.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_planning_msgs/msg/expand_stop_range.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/velocity_limit.hpp"
#include "autoware_planning_msgs/msg/velocity_limit_clear_command.hpp"
#include "autoware_utils/autoware_utils.hpp"
#include "boost/assert.hpp"
#include "boost/assign/list_of.hpp"
#include "boost/format.hpp"
#include "boost/geometry.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "obstacle_stop_planner/adaptive_cruise_control.hpp"
#include "obstacle_stop_planner/debug_marker.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "pcl/common/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "signal_processing/lowpass_filter_1d.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "vehicle_info_util/vehicle_info_util.hpp"

namespace motion_planning
{
namespace bg = boost::geometry;
using autoware_debug_msgs::msg::BoolStamped;
using autoware_debug_msgs::msg::Float32MultiArrayStamped;
using autoware_debug_msgs::msg::Float32Stamped;
using autoware_perception_msgs::msg::DynamicObjectArray;
using autoware_planning_msgs::msg::ExpandStopRange;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_planning_msgs::msg::VelocityLimit;
using autoware_planning_msgs::msg::VelocityLimitClearCommand;
using autoware_utils::Point2d;
using autoware_utils::Polygon2d;
using vehicle_info_util::VehicleInfo;

struct StopPoint
{
  TrajectoryPoint point{};
  size_t index;
};

struct SlowDownSection
{
  TrajectoryPoint start_point{};
  TrajectoryPoint end_point{};
  size_t slow_down_start_idx;
  size_t slow_down_end_idx;
  double velocity;
};

class ObstacleStopPlannerNode : public rclcpp::Node
{
public:
  explicit ObstacleStopPlannerNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    bool enable_slow_down;     // set True, slow down for obstacle beside the path
    double max_velocity;       // max velocity [m/s]
    double lowpass_gain;       // smoothing calculated current acceleration [-]
    double hunting_threshold;  // keep slow down or stop state if obstacle vanished [s]
  };

  struct StopParam
  {
    double stop_margin;               // stop margin distance from obstacle on the path [m]
    double min_behavior_stop_margin;  // margin distance, any other stop point is inserted [m]
    double expand_stop_range;         // margin of vehicle footprint [m]
    double extend_distance;           // trajectory extend_distance [m]
    double step_length;               // step length for pointcloud search range [m]
    double stop_search_radius;        // search radius for obstacle point cloud [m]
  };

  struct SlowDownParam
  {
    double normal_min_jerk;         // min jerk limit for mild stop [m/sss]
    double normal_min_acc;          // min deceleration limit for mild stop [m/ss]
    double limit_min_jerk;          // min jerk limit [m/sss]
    double limit_min_acc;           // min deceleration limit [m/ss]
    double forward_margin;          // slow down margin(vehicle front -> obstacle) [m]
    double backward_margin;         // slow down margin(obstacle vehicle rear) [m]
    double expand_slow_down_range;  // lateral range of detection area [m]
    double max_slow_down_vel;       // maximum speed in slow down section [m/s]
    double min_slow_down_vel;       // minimum velocity in slow down section [m/s]
    bool consider_constraints;      // set "True", decel point is planned under jerk/dec constraints
    double slow_down_vel;           // target slow down velocity [m/s]
    double forward_margin_min;      // min margin for relaxing slow down margin [m/s]
    double forward_margin_span;     // fineness param for relaxing slow down margin [m/s]
    double slow_down_min_jerk;      // min slow down jerk constraint [m/sss]
    double jerk_start;              // init jerk used for deceleration planning [m/sss]
    double jerk_span;               // fineness param for planning deceleration jerk [m/sss]
    double vel_threshold_reset_velocity_limit_;  // velocity threshold,
                                                 // check complete deceleration [m/s]
    double dec_threshold_reset_velocity_limit_;  // acceleration threshold,
                                                 // check complete deceleration [m/ss]
    double slow_down_search_radius;  // search radius for slow down obstacle point cloud [m]
  };

  struct PlannerData
  {
    diagnostic_msgs::msg::DiagnosticStatus stop_reason_diag{};

    geometry_msgs::msg::Pose current_pose{};

    pcl::PointXYZ nearest_collision_point;
    pcl::PointXYZ nearest_slow_down_point;
    pcl::PointXYZ lateral_nearest_slow_down_point;
    rclcpp::Time nearest_collision_point_time{};
    double lateral_deviation{0.0};

    size_t trajectory_trim_index{};
    size_t decimate_trajectory_collision_index{};
    size_t decimate_trajectory_slow_down_index{};
    std::map<size_t, size_t> decimate_trajectory_index_map{};  // key: decimate index
                                                               // value: original index

    bool found_collision_points{false};
    bool found_slow_down_points{false};
    bool stop_require{false};
    bool slow_down_require{false};
    bool enable_adaptive_cruise{false};
  };

private:
  /*
   * ROS
   */
  // publisher and subscriber
  rclcpp::Subscription<Trajectory>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pointcloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr current_velocity_sub_;
  rclcpp::Subscription<DynamicObjectArray>::SharedPtr dynamic_object_sub_;
  rclcpp::Subscription<ExpandStopRange>::SharedPtr expand_stop_range_sub_;
  rclcpp::Publisher<Trajectory>::SharedPtr path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr stop_reason_diag_pub_;
  rclcpp::Publisher<VelocityLimitClearCommand>::SharedPtr pub_clear_velocity_limit_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr pub_velocity_limit_;

  std::unique_ptr<motion_planning::AdaptiveCruiseController> acc_controller_;
  std::shared_ptr<ObstacleStopPlannerDebugNode> debug_ptr_;
  std::shared_ptr<LowpassFilter1d> lpf_acc_{nullptr};
  boost::optional<SlowDownSection> latest_slow_down_section_{};
  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  sensor_msgs::msg::PointCloud2::SharedPtr obstacle_ros_pointcloud_ptr_{nullptr};
  DynamicObjectArray::ConstSharedPtr object_ptr_{nullptr};
  rclcpp::Time last_detection_time_;

  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_velocity_ptr_{nullptr};
  geometry_msgs::msg::TwistStamped::ConstSharedPtr prev_velocity_ptr_{nullptr};
  double current_acc_{0.0};

  bool set_velocity_limit_{false};

  VehicleInfo vehicle_info_;
  NodeParam node_param_;
  StopParam stop_param_;
  SlowDownParam slow_down_param_;

  /*
   * Callback
   */
  void obstaclePointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
  void pathCallback(const Trajectory::ConstSharedPtr input_msg);
  void dynamicObjectCallback(const DynamicObjectArray::ConstSharedPtr input_msg);
  void currentVelocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr input_msg);
  void externalExpandStopRangeCallback(const ExpandStopRange::ConstSharedPtr input_msg);

private:
  bool withinPolygon(
    const std::vector<cv::Point2d> & cv_polygon, const double radius, const Point2d & prev_point,
    const Point2d & next_point, pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_points_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr within_points_ptr);

  bool convexHull(
    const std::vector<cv::Point2d> & pointcloud, std::vector<cv::Point2d> & polygon_points);

  void searchObstacle(
    const Trajectory & decimate_trajectory, Trajectory & output, PlannerData & planner_data);

  void insertVelocity(Trajectory & trajectory, PlannerData & planner_data);

  Trajectory decimateTrajectory(
    const Trajectory & input, const double step_length, std::map<size_t, size_t> & index_map);

  Trajectory trimTrajectoryWithIndexFromSelfPose(
    const Trajectory & input, const geometry_msgs::msg::Pose & self_pose, size_t & index);

  bool searchPointcloudNearTrajectory(
    const Trajectory & trajectory,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_points_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_points_ptr);

  void createOneStepPolygon(
    const geometry_msgs::msg::Pose & base_step_pose,
    const geometry_msgs::msg::Pose & next_step_pose, std::vector<cv::Point2d> & polygon,
    const double expand_width = 0.0);

  bool getSelfPose(
    const std_msgs::msg::Header & header, const tf2_ros::Buffer & tf_buffer,
    geometry_msgs::msg::Pose & self_pose);

  void getNearestPoint(
    const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const geometry_msgs::msg::Pose & base_pose,
    pcl::PointXYZ * nearest_collision_point, rclcpp::Time * nearest_collision_point_time);

  void getLateralNearestPoint(
    const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const geometry_msgs::msg::Pose & base_pose,
    pcl::PointXYZ * lateral_nearest_point, double * deviation);

  geometry_msgs::msg::Pose getVehicleCenterFromBase(const geometry_msgs::msg::Pose & base_pose);

  void insertStopPoint(
    const StopPoint & stop_point, Trajectory & output,
    diagnostic_msgs::msg::DiagnosticStatus & stop_reason_diag);

  StopPoint searchInsertPoint(
    const int idx, const Trajectory & base_trajectory, const double dist_remain);

  StopPoint createTargetPoint(
    const int idx, const double margin, const Trajectory & base_trajectory,
    const double dist_remain);

  SlowDownSection createSlowDownSection(
    const int idx, const Trajectory & base_trajectory, const double lateral_deviation,
    const double dist_remain, const double dist_vehicle_to_obstacle);

  SlowDownSection createSlowDownSectionFromMargin(
    const int idx, const Trajectory & base_trajectory, const double forward_margin,
    const double backward_margin, const double velocity);

  void insertSlowDownSection(const SlowDownSection & slow_down_section, Trajectory & output);

  Trajectory extendTrajectory(const Trajectory & input, const double extend_distance);

  TrajectoryPoint getExtendTrajectoryPoint(
    double extend_distance, const TrajectoryPoint & goal_point);

  void setExternalVelocityLimit();

  void resetExternalVelocityLimit();

  void publishDebugData(const PlannerData & planner_data);
};
}  // namespace motion_planning

#endif  // OBSTACLE_STOP_PLANNER__NODE_HPP_
