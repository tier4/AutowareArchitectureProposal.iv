// Copyright 2018-2019 Autoware Foundation
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

#include "motion_velocity_optimizer/motion_velocity_optimizer.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <utility>
#include <string>
#include <vector>

#include "motion_velocity_optimizer/optimizer/l2_pseudo_jerk_optimizer.hpp"
#include "motion_velocity_optimizer/optimizer/linf_pseudo_jerk_optimizer.hpp"

#include "tf2_ros/create_timer_ros.h"

using std::placeholders::_1;

#define UPDATE_PARAM(NAME) update_param(parameters, #NAME, planning_param_.NAME);
namespace
{
template<typename T>
void update_param(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & name, T & value)
{
  auto it = std::find_if(
    parameters.cbegin(), parameters.cend(),
    [&name](const rclcpp::Parameter & parameter) {return parameter.get_name() == name;});
  if (it != parameters.cend()) {
    value = it->template get_value<T>();
  }
}
}  // namespace

MotionVelocityOptimizer::MotionVelocityOptimizer()
: Node("motion_velocity_optimizer"), tf_listener_(tf_buffer_)
{
  auto & p = planning_param_;
  p.max_velocity = declare_parameter("max_velocity", 20.0);  // 72.0 kmph
  p.max_accel = declare_parameter("max_accel", 2.0);         // 0.11G
  p.min_decel = declare_parameter("min_decel", -3.0);        // -0.2G

  p.max_lateral_accel = declare_parameter("max_lateral_accel", 0.2);  //
  p.decel_distance_before_curve = declare_parameter("decel_distance_before_curve", 3.5);
  p.decel_distance_after_curve = declare_parameter("decel_distance_after_curve", 0.0);
  p.min_curve_velocity = declare_parameter("min_curve_velocity", 1.38);

  p.replan_vel_deviation = declare_parameter("replan_vel_deviation", 3.0);
  p.engage_velocity = declare_parameter("engage_velocity", 0.3);
  p.engage_acceleration = declare_parameter("engage_acceleration", 0.1);
  p.engage_exit_ratio = declare_parameter("engage_exit_ratio", 0.5);
  p.engage_exit_ratio = std::min(std::max(p.engage_exit_ratio, 0.0), 1.0);

  p.extract_ahead_dist = declare_parameter("extract_ahead_dist", 200.0);
  p.extract_behind_dist = declare_parameter("extract_behind_dist", 3.0);
  p.max_trajectory_length = declare_parameter("max_trajectory_length", 200.0);
  p.min_trajectory_length = declare_parameter("min_trajectory_length", 30.0);
  p.resample_time = declare_parameter("resample_time", 10.0);
  p.resample_dt = declare_parameter("resample_dt", 0.1);
  p.min_trajectory_interval_distance = declare_parameter("min_trajectory_interval_distance", 0.1);
  p.stop_dist_to_prohibit_engage = declare_parameter("stop_dist_to_prohibit_engage", 1.5);
  p.delta_yaw_threshold = declare_parameter("delta_yaw_threshold", M_PI / 3.0);

  p.algorithm_type = declare_parameter("algorithm_type", "L2");
  if (p.algorithm_type != "L2" && p.algorithm_type != "Linf") {
    RCLCPP_WARN(get_logger(), "[MotionVelocityOptimizer] undesired algorithm is selected. set L2.");
    p.algorithm_type = "L2";
  }

  publish_debug_trajs_ = declare_parameter("publish_debug_trajs", false);

  pub_trajectory_ =
    create_publisher<autoware_planning_msgs::msg::Trajectory>("output/trajectory", rclcpp::QoS{1});
  pub_dist_to_stopline_ = create_publisher<autoware_debug_msgs::msg::Float32Stamped>(
    "distance_to_stopline", rclcpp::QoS{1});
  sub_current_trajectory_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/trajectory", 1,
    std::bind(&MotionVelocityOptimizer::callbackCurrentTrajectory, this, _1));
  sub_current_velocity_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "/localization/twist", 1,
    std::bind(&MotionVelocityOptimizer::callbackCurrentVelocity, this, _1));
  sub_external_velocity_limit_ = create_subscription<autoware_planning_msgs::msg::VelocityLimit>(
    "external_velocity_limit_mps", 1,
    std::bind(&MotionVelocityOptimizer::callbackExternalVelocityLimit, this, _1));

  if (p.algorithm_type == "L2") {
    L2PseudoJerkOptimizer::OptimizerParam param;
    param.max_accel = p.max_accel;
    param.min_decel = p.min_decel;
    param.pseudo_jerk_weight = declare_parameter("pseudo_jerk_weight", 100.0);
    param.over_v_weight = declare_parameter("over_v_weight", 100000.0);
    param.over_a_weight = declare_parameter("over_a_weight", 1000.0);
    optimizer_ = std::make_shared<L2PseudoJerkOptimizer>(param);
  } else if (p.algorithm_type == "Linf") {
    LinfPseudoJerkOptimizer::OptimizerParam param;
    param.max_accel = p.max_accel;
    param.min_decel = p.min_decel;
    param.pseudo_jerk_weight = declare_parameter("pseudo_jerk_weight", 200.0);
    param.over_v_weight = declare_parameter("over_v_weight", 100000.0);
    param.over_a_weight = declare_parameter("over_a_weight", 5000.0);
    optimizer_ = std::make_shared<LinfPseudoJerkOptimizer>(param);
  } else {
    RCLCPP_ERROR(get_logger(), "unknown velocity optimizer.");
  }

  /* debug */
  debug_closest_velocity_ =
    create_publisher<autoware_debug_msgs::msg::Float32Stamped>("closest_velocity", rclcpp::QoS{1});
  debug_closest_acc_ = create_publisher<autoware_debug_msgs::msg::Float32Stamped>(
    "closest_acceleration", rclcpp::QoS{1});
  pub_trajectory_raw_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "debug/trajectory_raw", rclcpp::QoS{1});
  pub_trajectory_vel_lim_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "debug/trajectory_external_velocity_limitted", rclcpp::QoS{1});
  pub_trajectory_latcc_filtered_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "debug/trajectory_lateral_acc_filtered", rclcpp::QoS{1});
  pub_trajectory_resampled_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "debug/trajectory_time_resampled", rclcpp::QoS{1});

  /* timer */
  {
    external_velocity_limit_update_rate_ = get_parameter("over_a_weight").as_double();
    auto timer_callback = std::bind(&MotionVelocityOptimizer::timerCallback, this);
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / external_velocity_limit_update_rate_));
    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
      get_clock(), period, std::move(timer_callback), get_node_base_interface()->get_context());
    get_node_timers_interface()->add_timer(timer_, nullptr);
  }

  /* parameter update */
  set_param_res_ =
    this->add_on_set_parameters_callback(
    std::bind(
      &MotionVelocityOptimizer::paramCallback, this,
      _1));


  /* wait to get vehicle position */
  blockUntilVehiclePositionAvailable(tf2::durationFromSec(1.0));
}
MotionVelocityOptimizer::~MotionVelocityOptimizer() {}

void MotionVelocityOptimizer::publishTrajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory) const
{
  pub_trajectory_->publish(trajectory);
}

void MotionVelocityOptimizer::callbackCurrentVelocity(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  current_velocity_ptr_ = msg;
}
void MotionVelocityOptimizer::callbackCurrentTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  base_traj_raw_ptr_ = msg;
  run();
}
void MotionVelocityOptimizer::callbackExternalVelocityLimit(
  const autoware_planning_msgs::msg::VelocityLimit::ConstSharedPtr msg)
{
  external_velocity_limit_ptr_ = msg;
}

void MotionVelocityOptimizer::updateCurrentPose()
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      get_logger(), "[MotionVelocityOptimizer] cannot get map to base_link transform. %s",
      ex.what());
    return;
  }

  geometry_msgs::msg::PoseStamped ps;
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  current_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(ps);
}

void MotionVelocityOptimizer::run()
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto t_start = std::chrono::system_clock::now();
  RCLCPP_DEBUG(
    get_logger(), "============================== run() start ==============================");

  updateCurrentPose();

  /* guard */
  if (!current_pose_ptr_ || !current_velocity_ptr_ || !base_traj_raw_ptr_) {
    RCLCPP_DEBUG(
      get_logger(), "wait topics : current_pose = %d, current_vel = %d, base_traj = %d",
      (bool)current_pose_ptr_, (bool)current_velocity_ptr_, (bool)base_traj_raw_ptr_);
    return;
  }
  if (base_traj_raw_ptr_->points.empty()) {
    RCLCPP_DEBUG(get_logger(), "received trajectory is empty");
    return;
  }

  /* calculate trajectory velocity */
  autoware_planning_msgs::msg::Trajectory output = calcTrajectoryVelocity(*base_traj_raw_ptr_);

  /* publish message */
  output.header = base_traj_raw_ptr_->header;
  publishTrajectory(output);

  prev_output_ = output;

  auto t_end = std::chrono::system_clock::now();
  double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start).count();
  RCLCPP_DEBUG(get_logger(), "run: calculation time = %f [ms]", elapsed * 1.0e-6);
  RCLCPP_DEBUG(
    get_logger(), "============================== run() end ==============================\n\n");
}

autoware_planning_msgs::msg::Trajectory MotionVelocityOptimizer::calcTrajectoryVelocity(
  const autoware_planning_msgs::msg::Trajectory & traj_input)
{
  /* Find the nearest point to reference_traj */
  int input_closest = vpu::calcClosestWaypoint(
    traj_input, current_pose_ptr_->pose, planning_param_.delta_yaw_threshold);
  if (input_closest < 0) {
    RCLCPP_WARN(
      get_logger(), "[velocity planner] cannot find closest waypoint for input trajectory");
    return prev_output_;
  }

  autoware_planning_msgs::msg::Trajectory traj_extracted;    // extructed around current_position
  autoware_planning_msgs::msg::Trajectory traj_vel_limtted;  // external velocity limitted
  autoware_planning_msgs::msg::Trajectory
    traj_latacc_filtered;  // max lateral acceleration limitted
  autoware_planning_msgs::msg::Trajectory
    traj_resampled;                                // resampled depending on the current_velocity
  autoware_planning_msgs::msg::Trajectory output;  // velocity is optimized by qp solver

  /* Extract trajectory around self-position with desired forward-backwaed length*/
  if (!extractPathAroundIndex(traj_input, input_closest, /* out */ traj_extracted)) {
    return prev_output_;
  }

  /* Apply external velocity limit */
  externalVelocityLimitFilter(traj_extracted, /* out */ traj_vel_limtted);

  /* Lateral acceleration limt */
  if (!lateralAccelerationFilter(traj_vel_limtted, /* out */ traj_latacc_filtered)) {
    return prev_output_;
  }

  /* Resample trajectory with ego-velocity based interval distance */
  if (!resampleTrajectory(traj_latacc_filtered, /* out */ traj_resampled)) {
    return prev_output_;
  }
  int traj_resampled_closest = vpu::calcClosestWaypoint(
    traj_resampled, current_pose_ptr_->pose, planning_param_.delta_yaw_threshold);
  if (traj_resampled_closest < 0) {
    RCLCPP_WARN(
      get_logger(), "[velocity planner] cannot find closest waypoint for resampled trajectory");
    return prev_output_;
  }

  /* Change trajectory velocity to zero when current_velocity == 0 & stop_dist is close */
  preventMoveToCloseStopLine(traj_resampled_closest, traj_resampled);

  /* for reverse velocity */
  const bool is_reverse = (traj_resampled.points.at(traj_resampled_closest).twist.linear.x < 0.0);
  if (is_reverse) {
    vpu::multiplyConstantToTrajectoryVelocity(-1.0, /* out */ traj_resampled);
  }

  /*
   * Calculate the closest point on the previously planned traj
   * (used to get initial planning speed)
   */
  int prev_output_closest = vpu::calcClosestWaypoint(
    prev_output_, current_pose_ptr_->pose, planning_param_.delta_yaw_threshold);
  RCLCPP_DEBUG(
    get_logger(), "[calcClosestWaypoint] base_resampled.size() = %lu, prev_planned_closest_ = %d",
    traj_resampled.points.size(), prev_output_closest);

  /* Optimize velocity */
  output =
    optimizeVelocity(traj_resampled, traj_resampled_closest, prev_output_, prev_output_closest);

  /* Max velocity filter for safety */
  vpu::maximumVelocityFilter(planning_param_.max_velocity, output);

  /* for negative velocity */
  if (is_reverse) {
    vpu::multiplyConstantToTrajectoryVelocity(-1.0, output);
  }

  /* Insert behind velocity for output's consistency */
  insertBehindVelocity(prev_output_closest, prev_output_, traj_resampled_closest, output);

  /* for debug */
  publishFloat(output.points.at(traj_resampled_closest).twist.linear.x, debug_closest_velocity_);
  publishFloat(output.points.at(traj_resampled_closest).accel.linear.x, debug_closest_acc_);
  publishStopDistance(output, traj_resampled_closest);
  if (publish_debug_trajs_) {
    pub_trajectory_raw_->publish(traj_extracted);
    pub_trajectory_vel_lim_->publish(traj_vel_limtted);
    pub_trajectory_latcc_filtered_->publish(traj_latacc_filtered);
    pub_trajectory_resampled_->publish(traj_resampled);
  }

  return output;
}

void MotionVelocityOptimizer::insertBehindVelocity(
  const int prev_output_closest, const autoware_planning_msgs::msg::Trajectory & prev_output,
  const int output_closest, autoware_planning_msgs::msg::Trajectory & output) const
{
  int j = std::max(prev_output_closest - 1, 0);
  for (int i = output_closest - 1; i >= 0; --i) {
    if (
      initialize_type_ == InitializeType::INIT ||
      initialize_type_ == InitializeType::LARGE_DEVIATION_REPLAN ||
      initialize_type_ == InitializeType::ENGAGING)
    {
      output.points.at(i).twist.linear.x = output.points.at(output_closest).twist.linear.x;
    } else {
      output.points.at(i).twist.linear.x = prev_output.points.at(j).twist.linear.x;
    }

    j = std::max(j - 1, 0);
  }
}

void MotionVelocityOptimizer::publishStopDistance(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const int closest) const
{
  /* stop distance calculation */
  int stop_idx = 0;
  const double stop_dist_lim = 50.0;
  double stop_dist = stop_dist_lim;
  if (vpu::searchZeroVelocityIdx(trajectory, stop_idx)) {
    stop_dist = vpu::calcLengthOnWaypoints(trajectory, closest, stop_idx);
  }
  stop_dist = closest > stop_idx ? stop_dist : -stop_dist;
  autoware_debug_msgs::msg::Float32Stamped dist_to_stopline;
  dist_to_stopline.stamp = this->now();
  dist_to_stopline.data = std::max(-stop_dist_lim, std::min(stop_dist_lim, stop_dist));
  pub_dist_to_stopline_->publish(dist_to_stopline);
}

bool MotionVelocityOptimizer::resampleTrajectory(
  const autoware_planning_msgs::msg::Trajectory & input,
  autoware_planning_msgs::msg::Trajectory & output) const
{
  std::vector<double> in_arclength;
  vpu::calcTrajectoryArclength(input, in_arclength);
  const double Nt = planning_param_.resample_time / std::max(planning_param_.resample_dt, 0.001);
  const double ds_nominal = std::max(
    current_velocity_ptr_->twist.linear.x * planning_param_.resample_dt,
    planning_param_.min_trajectory_interval_distance);
  const double Ns = planning_param_.min_trajectory_length / std::max(ds_nominal, 0.001);
  const double N = std::max(Nt, Ns);
  std::vector<double> out_arclength;
  double dist_i = 0.0;
  out_arclength.push_back(dist_i);
  bool is_endpoint_included = false;
  for (int i = 1; i <= N; ++i) {
    double ds = ds_nominal;
    if (i > Nt) {
      if (dist_i > planning_param_.min_trajectory_length) {
        break;  // planning time is enough and planning distance is over min length.
      }
      // if the planning time is not enough to see the desired distance,
      // change the interval distance to see far.
      ds = std::max(1.0, ds_nominal);
    }
    dist_i += ds;
    if (dist_i > planning_param_.max_trajectory_length) {
      break;  // distance is over max.
    }
    if (dist_i >= in_arclength.back()) {
      is_endpoint_included = true;  // distance is over input endpoint.
      break;
    }
    out_arclength.push_back(dist_i);
  }
  if (!vpu::linearInterpTrajectory(in_arclength, input, out_arclength, output)) {
    RCLCPP_WARN(
      get_logger(),
      "[motion_velocity_optimizer]: fail trajectory interpolation. size : in_arclength = %lu, "
      "input = %lu, out_arclength = %lu, output = %lu",
      in_arclength.size(), input.points.size(), out_arclength.size(), output.points.size());
    return false;
  }

  // add end point directly to consider the endpoint velocity.
  if (is_endpoint_included) {
    constexpr double ep_dist = 1.0E-3;
    if (vpu::calcDist2d(output.points.back(), input.points.back()) < ep_dist) {
      output.points.back() = input.points.back();
    } else {
      output.points.push_back(input.points.back());
    }
  }
  return true;
}

void MotionVelocityOptimizer::calcInitialMotion(
  const double & target_vel, const autoware_planning_msgs::msg::Trajectory & reference_traj,
  const int reference_traj_closest, const autoware_planning_msgs::msg::Trajectory & prev_output,
  const int prev_output_closest, double & initial_vel, double & initial_acc)
{
  const double vehicle_speed = std::fabs(current_velocity_ptr_->twist.linear.x);

  /* first time */
  if (prev_output.points.empty() || prev_output_closest == -1) {
    initial_vel = vehicle_speed;
    initial_acc = 0.0;  // if possible, use actual vehicle acc & jerk value;
    initialize_type_ = InitializeType::INIT;
    return;
  }

  /* when velocity tracking deviation is large */
  const double desired_vel = prev_output.points.at(prev_output_closest).twist.linear.x;
  const double vel_error = vehicle_speed - std::fabs(desired_vel);
  if (std::fabs(vel_error) > planning_param_.replan_vel_deviation) {
    initialize_type_ = InitializeType::LARGE_DEVIATION_REPLAN;
    initial_vel = vehicle_speed;  // use current vehicle speed
    initial_acc = prev_output.points.at(prev_output_closest).accel.linear.x;
    RCLCPP_DEBUG(
      get_logger(),
      "[calcInitialMotion] : Large deviation error for speed control. Use current speed for "
      "initial value, desired_vel = %f, vehicle_speed = %f, vel_error = %f, error_thr = %f",
      desired_vel, vehicle_speed, vel_error, planning_param_.replan_vel_deviation);
    return;
  }

  /*
   * if current vehicle velocity is low && base_desired speed is high,
   * use engage_velocity for engage vehicle
   */
  const double engage_vel_thr =
    planning_param_.engage_velocity * planning_param_.engage_exit_ratio;
  if (vehicle_speed < engage_vel_thr) {
    if (target_vel >= planning_param_.engage_velocity) {
      int idx = 0;
      const bool ret = vpu::searchZeroVelocityIdx(reference_traj, idx);
      const bool exist_stop_point = (idx >= reference_traj_closest) ? ret : false;

      const double stop_dist = vpu::calcDist2d(
        reference_traj.points.at(idx), reference_traj.points.at(reference_traj_closest));
      if (!exist_stop_point || stop_dist > planning_param_.stop_dist_to_prohibit_engage) {
        initialize_type_ = InitializeType::ENGAGING;
        initial_vel = planning_param_.engage_velocity;
        initial_acc = planning_param_.engage_acceleration;
        RCLCPP_DEBUG(
          get_logger(),
          "[calcInitialMotion]: vehicle speed is low (%.3f), and desired speed is high (%.3f). Use "
          "engage speed (%.3f) until vehicle speed reaches engage_vel_thr (%.3f). stop_dist = %.3f",
          vehicle_speed, target_vel, planning_param_.engage_velocity, engage_vel_thr, stop_dist);
        return;
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000 /*ms*/,
          "[calcInitialMotion]: stop point is close (%.3f[m]). no engage.", stop_dist);
      }
    } else if (target_vel > 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000 /*ms*/,
        "[calcInitialMotion]: target velocity(%.3f[m/s]) is lower than engage "
        "velocity(%.3f[m/s]). ",
        target_vel, planning_param_.engage_velocity);
    }
  }

  /* normal update: use closest in prev_output */
  initialize_type_ = InitializeType::NORMAL;
  initial_vel = prev_output.points.at(prev_output_closest).twist.linear.x;
  initial_acc = prev_output.points.at(prev_output_closest).accel.linear.x;
  RCLCPP_DEBUG(
    get_logger(),
    "[calcInitialMotion]: normal update. v0 = %f, a0 = %f, vehicle_speed = %f, target_vel = %f",
    initial_vel, initial_acc, vehicle_speed, target_vel);
}

autoware_planning_msgs::msg::Trajectory MotionVelocityOptimizer::optimizeVelocity(
  const autoware_planning_msgs::msg::Trajectory & input, const int input_closest,
  const autoware_planning_msgs::msg::Trajectory & prev_output, const int prev_output_closest)
{
  const double target_vel = std::fabs(input.points.at(input_closest).twist.linear.x);

  /* calculate initial motion for planning */
  double initial_vel = 0.0;
  double initial_acc = 0.0;
  calcInitialMotion(
    target_vel, input, input_closest, prev_output, prev_output_closest,
    /* out */ initial_vel, initial_acc);

  autoware_planning_msgs::msg::Trajectory optimized_traj;
  bool res = optimizer_->solve(initial_vel, initial_acc, input_closest, input, &optimized_traj);
  if (!res) {
    // RCLCPP_WARN(get_logger(), "[optimizeVelocity] fail to solve optimization.");
  }

  /* find stop point for stopVelocityFilter */
  int stop_idx = -1;
  bool stop_point_exists = vpu::searchZeroVelocityIdx(input, stop_idx);
  RCLCPP_DEBUG(
    get_logger(), "[replan]: target_vel = %f, stop_idx = %d, closest = %d, stop_point_exists = %d",
    target_vel, stop_idx, input_closest, static_cast<int>(stop_point_exists));

  /* for the endpoint of the trajectory */
  if (optimized_traj.points.size() > 0) {
    optimized_traj.points.back().twist.linear.x = 0.0;
  }

  /* set output trajectory */
  RCLCPP_DEBUG(
    get_logger(), "[optimizeVelocity]: optimized_traj.size() = %lu", optimized_traj.points.size());
  return optimized_traj;
}

bool MotionVelocityOptimizer::lateralAccelerationFilter(
  const autoware_planning_msgs::msg::Trajectory & input,
  autoware_planning_msgs::msg::Trajectory & output) const
{
  if (input.points.size() == 0) {
    return false;
  }

  output = input;  // initialize

  if (input.points.size() < 3) {
    return true;  // cannot calculate lateral acc. do nothing.
  }

  /* Interpolate with constant interval distance for lateral acceleration calculation. */
  constexpr double points_interval = 0.1;  // [m]
  std::vector<double> in_arclength, out_arclength;
  vpu::calcTrajectoryArclength(input, in_arclength);
  for (double s = 0; s < in_arclength.back(); s += points_interval) {
    out_arclength.push_back(s);
  }
  if (!vpu::linearInterpTrajectory(in_arclength, input, out_arclength, output)) {
    RCLCPP_WARN(
      get_logger(),
      "[motion_velocity_optimizer]: interpolation failed at lateral acceleraion filter.");
    return false;
  }
  output.points.back().twist = input.points.back().twist;  // keep the final speed.

  constexpr double curvature_calc_dist = 3.0;  // [m] calc curvature with 3m away points
  const unsigned int idx_dist =
    std::max(static_cast<int>(curvature_calc_dist / points_interval), 1);

  /* Calculate curvature assuming the trajectory points interval is constant */
  std::vector<double> curvature_v;
  vpu::calcTrajectoryCurvatureFrom3Points(output, idx_dist, curvature_v);

  /*  Decrease speed according to lateral G */
  const int before_decel_index =
    static_cast<int>(std::round(planning_param_.decel_distance_before_curve / points_interval));
  const int after_decel_index =
    static_cast<int>(std::round(planning_param_.decel_distance_after_curve / points_interval));
  const double max_lateral_accel_abs = std::fabs(planning_param_.max_lateral_accel);

  const int output_size = static_cast<int>(output.points.size());
  for (int i = 0; i < output_size; ++i) {
    double curvature = 0.0;
    const int start = std::max(i - after_decel_index, 0);
    const int end = std::min(output_size, i + before_decel_index);
    for (int j = start; j < end; ++j) {
      curvature = std::max(curvature, std::fabs(curvature_v.at(j)));
    }
    double v_curvature_max = std::sqrt(max_lateral_accel_abs / std::max(curvature, 1.0E-5));
    v_curvature_max = std::max(v_curvature_max, planning_param_.min_curve_velocity);
    if (output.points.at(i).twist.linear.x > v_curvature_max) {
      output.points.at(i).twist.linear.x = v_curvature_max;
    }
  }
  return true;
}

bool MotionVelocityOptimizer::externalVelocityLimitFilter(
  const autoware_planning_msgs::msg::Trajectory & input,
  autoware_planning_msgs::msg::Trajectory & output) const
{
  output = input;
  if (!external_velocity_limit_filtered_) {return false;}

  vpu::maximumVelocityFilter(*external_velocity_limit_filtered_, output);
  RCLCPP_DEBUG(
    get_logger(), "[externalVelocityLimit]: limit_vel = %.3f", *external_velocity_limit_filtered_);
  return true;
}

void MotionVelocityOptimizer::preventMoveToCloseStopLine(
  const int closest, autoware_planning_msgs::msg::Trajectory & traj) const
{
  if (std::fabs(current_velocity_ptr_->twist.linear.x) > 0.1) {
    return;
  }

  int stop_idx = 0;
  if (!vpu::searchZeroVelocityIdx(traj, stop_idx)) {
    return;  // no obstacle.
  }
  /* if the desired stop line is ahead of ego-vehicle */
  double dist_to_stopline = vpu::calcDist2d(traj.points.at(stop_idx), traj.points.at(closest));
  std::string debug_msg;
  if (stop_idx >= closest && dist_to_stopline < planning_param_.stop_dist_to_prohibit_engage) {
    vpu::setZeroVelocity(traj);
    debug_msg = "curr_vel is low, and stop line is close. keep stopping.";
  } else {
    debug_msg = "curr_vel is low, but stop point is far. move.";
  }

  RCLCPP_DEBUG(
    get_logger(),
    "[preventMoveToCloseStopLine] %s curr_vel = %.3f, dist_to_stopline = %.3f, move_dist_min = "
    "%.3f, stop_idx = %d, closest = %d",
    debug_msg.c_str(), current_velocity_ptr_->twist.linear.x, dist_to_stopline,
    planning_param_.stop_dist_to_prohibit_engage, stop_idx, closest);
}

bool MotionVelocityOptimizer::extractPathAroundIndex(
  const autoware_planning_msgs::msg::Trajectory & input, const int index,
  autoware_planning_msgs::msg::Trajectory & output) const
{
  const double ahead_length = planning_param_.extract_ahead_dist;
  const double behind_length = planning_param_.extract_behind_dist;

  if (input.points.size() == 0 || index < 0 || static_cast<int>(input.points.size()) - 1 < index) {
    RCLCPP_WARN(
      get_logger(), "extractPathAroundIndex failed. input.points.size() = %lu, base_index = %d",
      input.points.size(), index);
    return false;
  }

  double dist_sum_tmp = 0.0;

  // calc ahead distance
  int ahead_index = input.points.size() - 1;
  for (int i = index; i < static_cast<int>(input.points.size()) - 1; ++i) {
    dist_sum_tmp += vpu::calcDist2d(input.points.at(i), input.points.at(i + 1));
    if (dist_sum_tmp > ahead_length) {
      ahead_index = i + 1;
      break;
    }
  }

  // calc behind distance
  dist_sum_tmp = 0.0;
  int behind_index = 0;
  for (int i = index; i > 0; --i) {
    dist_sum_tmp += vpu::calcDist2d(input.points.at(i), input.points[i - 1]);
    if (dist_sum_tmp > behind_length) {
      behind_index = i - 1;
      break;
    }
  }

  // extruct trajectory
  output.points.clear();
  for (int i = behind_index; i < ahead_index + 1; ++i) {
    output.points.push_back(input.points.at(i));
  }
  output.header = input.header;

  RCLCPP_DEBUG(
    get_logger(),
    "[extractPathAroundIndex] : input.size() = %lu, extract_base_index = %d, output.size() = %lu",
    input.points.size(), index, output.points.size());
  return true;
}

void MotionVelocityOptimizer::publishFloat(
  const double & data,
  const rclcpp::Publisher<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr pub) const
{
  autoware_debug_msgs::msg::Float32Stamped msg;
  msg.stamp = this->now();
  msg.data = data;
  pub->publish(msg);
}

void MotionVelocityOptimizer::updateExternalVelocityLimit(const double dt)
{
  if (!external_velocity_limit_ptr_) {return;}

  if (external_velocity_limit_ptr_->data < -1.0e-5) {
    RCLCPP_WARN(get_logger(), "external velocity limit is negative. The command is ignored");
    return;
  }

  double v_lim = std::min<double>(external_velocity_limit_ptr_->data, planning_param_.max_velocity);

  if (!external_velocity_limit_filtered_) {
    external_velocity_limit_filtered_ = std::make_shared<double>(v_lim);
    return;
  }

  double dv_raw = (v_lim - *external_velocity_limit_filtered_);
  double dv_filtered =
    std::max(std::min(dv_raw, planning_param_.max_accel * dt), planning_param_.min_decel * dt);
  *external_velocity_limit_filtered_ += dv_filtered;

  if (!prev_output_.points.empty() && dv_raw < -0.1) {
    double traj_v_max = vpu::getMaxAbsVelocity(prev_output_);
    *external_velocity_limit_filtered_ = std::min(traj_v_max, *external_velocity_limit_filtered_);
  }
}

void MotionVelocityOptimizer::timerCallback()
{
  const double dt = 1.0 / external_velocity_limit_update_rate_;
  updateExternalVelocityLimit(dt);
}

void MotionVelocityOptimizer::blockUntilVehiclePositionAvailable(const tf2::Duration & duration)
{
  static constexpr auto input = "map", output = "base_link";
  while (!tf_buffer_.canTransform(input, output, tf2::TimePointZero) &&
    rclcpp::ok())
  {
    RCLCPP_INFO(
      get_logger(), "waiting %d ms for %s->%s transform to become available",
      std::chrono::duration_cast<std::chrono::milliseconds>(duration * 1000).count(), input, output);
    rclcpp::sleep_for(duration);
  }
  RCLCPP_INFO(get_logger(), "transform available");
}

rcl_interfaces::msg::SetParametersResult MotionVelocityOptimizer::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    UPDATE_PARAM(max_velocity);
    UPDATE_PARAM(max_accel);
    UPDATE_PARAM(min_decel);
    UPDATE_PARAM(max_lateral_accel);
    UPDATE_PARAM(min_curve_velocity);
    UPDATE_PARAM(decel_distance_before_curve);
    UPDATE_PARAM(decel_distance_after_curve);
    UPDATE_PARAM(replan_vel_deviation);
    UPDATE_PARAM(engage_velocity);
    UPDATE_PARAM(engage_acceleration);
    UPDATE_PARAM(engage_exit_ratio);
    UPDATE_PARAM(extract_ahead_dist);
    UPDATE_PARAM(extract_behind_dist);
    UPDATE_PARAM(stop_dist_to_prohibit_engage);
    UPDATE_PARAM(delta_yaw_threshold);

    UPDATE_PARAM(resample_time);
    UPDATE_PARAM(resample_dt);
    UPDATE_PARAM(max_trajectory_length);
    UPDATE_PARAM(min_trajectory_length);
    UPDATE_PARAM(min_trajectory_interval_distance);

    optimizer_->setAccel(planning_param_.max_accel);
    optimizer_->setDecel(planning_param_.min_decel);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionVelocityOptimizer>());
  rclcpp::shutdown();
  return 0;
}
