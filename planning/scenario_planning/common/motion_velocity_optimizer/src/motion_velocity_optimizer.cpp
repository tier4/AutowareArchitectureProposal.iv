/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include <chrono>
#include <motion_velocity_optimizer/motion_velocity_optimizer.hpp>
#include <motion_velocity_optimizer/optimizer/l2_pseudo_jerk_optimizer.hpp>
#include <motion_velocity_optimizer/optimizer/linf_pseudo_jerk_optimizer.hpp>

// clang-format on
MotionVelocityOptimizer::MotionVelocityOptimizer() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  auto & p = planning_param_;
  pnh_.param<double>("max_velocity", p.max_velocity, 20.0);  // 72.0 kmph
  pnh_.param<double>("max_accel", p.max_accel, 2.0);         // 0.11G
  pnh_.param<double>("min_decel", p.min_decel, -3.0);        // -0.2G

  pnh_.param<double>("max_lateral_accel", p.max_lateral_accel, 0.2);  //
  pnh_.param<double>("decel_distance_before_curve", p.decel_distance_before_curve, 3.5);
  pnh_.param<double>("decel_distance_after_curve", p.decel_distance_after_curve, 0.0);
  pnh_.param<double>("min_curve_velocity", p.min_curve_velocity, 1.38);

  pnh_.param<double>("replan_vel_deviation", p.replan_vel_deviation, 3.0);
  pnh_.param<double>("engage_velocity", p.engage_velocity, 0.3);
  pnh_.param<double>("engage_acceleration", p.engage_acceleration, 0.1);
  pnh_.param<double>("engage_exit_ratio", p.engage_exit_ratio, 0.5);
  p.engage_exit_ratio = std::min(std::max(p.engage_exit_ratio, 0.0), 1.0);

  pnh_.param<double>("stopping_velocity", p.stopping_velocity, 2.778);  // 10kmph
  pnh_.param<double>("stopping_distance", p.stopping_distance, 0.0);

  pnh_.param<double>("extract_ahead_dist", p.extract_ahead_dist, 200.0);
  pnh_.param<double>("extract_behind_dist", p.extract_behind_dist, 3.0);
  pnh_.param<double>("max_trajectory_length", p.max_trajectory_length, 200.0);
  pnh_.param<double>("min_trajectory_length", p.min_trajectory_length, 30.0);
  pnh_.param<double>("resample_time", p.resample_time, 10.0);
  pnh_.param<double>("resample_dt", p.resample_dt, 0.1);
  pnh_.param<double>("min_trajectory_interval_distance", p.min_trajectory_interval_distance, 0.1);
  pnh_.param<double>("stop_dist_to_prohibit_engage", p.stop_dist_to_prohibit_engage, 1.5);
  pnh_.param<double>("delta_yaw_threshold", p.delta_yaw_threshold, M_PI / 3.0);

  pnh_.param<double>("over_stop_velocity_warn_thr", over_stop_velocity_warn_thr_, 1.389);  // 5kmph

  pnh_.param<std::string>("algorithm_type", p.algorithm_type, "L2");
  if (p.algorithm_type != "L2" && p.algorithm_type != "Linf") {
    ROS_WARN("[MotionVelocityOptimizer] undesired algorithm is selected. set L2.");
    p.algorithm_type = "L2";
  }

  pnh_.param<bool>("publish_debug_trajs", publish_debug_trajs_, false);

  pub_trajectory_ = pnh_.advertise<autoware_planning_msgs::Trajectory>("output/trajectory", 1);
  pub_velocity_limit_ =
    pnh_.advertise<std_msgs::Float32>("output/current_velocity_limit_mps", 1, true);
  pub_velocity_limit_.publish(createFloat32Msg(p.max_velocity));  // publish default max velocity
  pub_dist_to_stopline_ = pnh_.advertise<std_msgs::Float32>("distance_to_stopline", 1);
  pub_over_stop_velocity_ = pnh_.advertise<std_msgs::Bool>("stop_speed_exceeded", 1);
  sub_current_trajectory_ = pnh_.subscribe(
    "input/trajectory", 1, &MotionVelocityOptimizer::callbackCurrentTrajectory, this);
  sub_current_velocity_ = pnh_.subscribe(
    "/localization/twist", 1, &MotionVelocityOptimizer::callbackCurrentVelocity, this);
  sub_external_velocity_limit_ = pnh_.subscribe(
    "input/external_velocity_limit_mps", 1, &MotionVelocityOptimizer::callbackExternalVelocityLimit,
    this);

  if (p.algorithm_type == "L2") {
    OptimizerParam param;
    param.max_accel = p.max_accel;
    param.min_decel = p.min_decel;
    pnh_.param<double>("pseudo_jerk_weight", param.pseudo_jerk_weight, 100.0);
    pnh_.param<double>("over_v_weight", param.over_v_weight, 100000.0);
    pnh_.param<double>("over_a_weight", param.over_a_weight, 1000.0);
    optimizer_ = boost::make_shared<L2PseudoJerkOptimizer>(param);
  } else if (p.algorithm_type == "Linf") {
    OptimizerParam param;
    param.max_accel = p.max_accel;
    param.min_decel = p.min_decel;
    pnh_.param<double>("pseudo_jerk_weight", param.pseudo_jerk_weight, 200.0);
    pnh_.param<double>("over_v_weight", param.over_v_weight, 100000.0);
    pnh_.param<double>("over_a_weight", param.over_a_weight, 5000.0);
    optimizer_ = boost::make_shared<LinfPseudoJerkOptimizer>(param);
  } else {
    ROS_ERROR("unknown velocity optimizer.");
  }

  /* dynamic reconfigure */
  dynamic_reconfigure::Server<
    motion_velocity_optimizer::MotionVelocityOptimizerConfig>::CallbackType dyncon_f =
    boost::bind(&MotionVelocityOptimizer::dynamicRecofCallback, this, _1, _2);
  dyncon_server_.setCallback(dyncon_f);

  /* debug */
  debug_closest_velocity_ = pnh_.advertise<std_msgs::Float32>("closest_velocity", 1);
  debug_closest_acc_ = pnh_.advertise<std_msgs::Float32>("closest_acceleration", 1);
  debug_closest_jerk_ = pnh_.advertise<std_msgs::Float32>("closest_jerk", 1);
  pub_trajectory_raw_ =
    pnh_.advertise<autoware_planning_msgs::Trajectory>("debug/trajectory_raw", 1);
  pub_trajectory_vel_lim_ = pnh_.advertise<autoware_planning_msgs::Trajectory>(
    "debug/trajectory_external_velocity_limited", 1);
  pub_trajectory_latcc_filtered_ =
    pnh_.advertise<autoware_planning_msgs::Trajectory>("debug/trajectory_lateral_acc_filtered", 1);
  pub_trajectory_resampled_ =
    pnh_.advertise<autoware_planning_msgs::Trajectory>("debug/trajectory_time_resampled", 1);

  const double timer_dt = 0.1;
  timer_ = nh_.createTimer(ros::Duration(timer_dt), &MotionVelocityOptimizer::timerCallback, this);

  /* wait to get vehicle position */
  while (ros::ok()) {
    try {
      tf_buffer_.lookupTransform("map", "base_link", ros::Time::now(), ros::Duration(5.0));
      break;
    } catch (tf2::TransformException & ex) {
      ROS_INFO(
        "[MotionVelocityOptimizer] is waiting to get map to base_link transform. %s", ex.what());
      continue;
    }
  }
}
MotionVelocityOptimizer::~MotionVelocityOptimizer() {}

void MotionVelocityOptimizer::publishTrajectory(
  const autoware_planning_msgs::Trajectory & trajectory) const
{
  pub_trajectory_.publish(trajectory);
}

void MotionVelocityOptimizer::callbackCurrentVelocity(
  const geometry_msgs::TwistStamped::ConstPtr msg)
{
  current_velocity_ptr_ = msg;
}
void MotionVelocityOptimizer::callbackCurrentTrajectory(
  const autoware_planning_msgs::Trajectory::ConstPtr msg)
{
  base_traj_raw_ptr_ = msg;
  run();
}
void MotionVelocityOptimizer::callbackExternalVelocityLimit(const std_msgs::Float32::ConstPtr msg)
{
  external_velocity_limit_ptr_ = msg;
  pub_velocity_limit_.publish(*msg);
}

void MotionVelocityOptimizer::updateCurrentPose()
{
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN("[MotionVelocityOptimizer] cannot get map to base_link transform. %s", ex.what());
    return;
  }

  geometry_msgs::PoseStamped ps;
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  current_pose_ptr_ = boost::make_shared<geometry_msgs::PoseStamped>(ps);
}

void MotionVelocityOptimizer::run()
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto t_start = std::chrono::system_clock::now();
  ROS_DEBUG("============================== run() start ==============================");

  updateCurrentPose();

  /* guard */
  if (!current_pose_ptr_ || !current_velocity_ptr_ || !base_traj_raw_ptr_) {
    ROS_DEBUG(
      "wait topics : current_pose = %d, current_vel = %d, base_traj = %d", (bool)current_pose_ptr_,
      (bool)current_velocity_ptr_, (bool)base_traj_raw_ptr_);
    return;
  }
  if (base_traj_raw_ptr_->points.empty()) {
    ROS_DEBUG("received trajectory is empty");
    return;
  }

  /* calculate trajectory velocity */
  autoware_planning_msgs::Trajectory output = calcTrajectoryVelocity(*base_traj_raw_ptr_);

  /* publish message */
  output.header = base_traj_raw_ptr_->header;
  publishTrajectory(output);

  prev_output_ = output;

  auto t_end = std::chrono::system_clock::now();
  double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start).count();
  ROS_DEBUG("run: calculation time = %f [ms]", elapsed * 1.0e-6);
  ROS_DEBUG("============================== run() end ==============================\n\n");
}

autoware_planning_msgs::Trajectory MotionVelocityOptimizer::calcTrajectoryVelocity(
  const autoware_planning_msgs::Trajectory & traj_input)
{
  /* Find the nearest point to reference_traj */
  int input_closest = vpu::calcClosestWaypoint(
    traj_input, current_pose_ptr_->pose, planning_param_.delta_yaw_threshold);
  if (input_closest < 0) {
    ROS_WARN("[velocity planner] cannot find closest waypoint for input trajectory");
    return prev_output_;
  }

  autoware_planning_msgs::Trajectory traj_extracted;        // extracted around current_position
  autoware_planning_msgs::Trajectory traj_vel_limited;      // external velocity limited
  autoware_planning_msgs::Trajectory traj_latacc_filtered;  // max lateral acceleration limited
  autoware_planning_msgs::Trajectory traj_resampled;  // resampled depending on the current_velocity
  autoware_planning_msgs::Trajectory output;          // velocity is optimized by qp solver

  /* Extract trajectory around self-position with desired forward-backward length*/
  if (!extractPathAroundIndex(traj_input, input_closest, /* out */ traj_extracted)) {
    return prev_output_;
  }

  /* Apply external velocity limit */
  externalVelocityLimitFilter(traj_extracted, /* out */ traj_vel_limited);

  /* Lateral acceleration limit */
  if (!lateralAccelerationFilter(traj_vel_limited, /* out */ traj_latacc_filtered)) {
    return prev_output_;
  }

  /* Resample trajectory with ego-velocity based interval distance */
  if (!resampleTrajectory(traj_latacc_filtered, /* out */ traj_resampled)) {
    return prev_output_;
  }
  int traj_resampled_closest = vpu::calcClosestWaypoint(
    traj_resampled, current_pose_ptr_->pose, planning_param_.delta_yaw_threshold);
  if (traj_resampled_closest < 0) {
    ROS_WARN("[velocity planner] cannot find closest waypoint for resampled trajectory");
    return prev_output_;
  }

  /* Change trajectory velocity to zero when current_velocity == 0 & stop_dist is close */
  preventMoveToCloseStopLine(traj_resampled_closest, traj_resampled);

  /* for reverse velocity */
  const bool is_reverse = (traj_resampled.points.at(traj_resampled_closest).twist.linear.x < 0.0);
  if (is_reverse) {
    vpu::multiplyConstantToTrajectoryVelocity(-1.0, /* out */ traj_resampled);
  }

  /* Calculate the closest index on the previously planned traj (used to get initial planning speed) */
  int prev_output_closest = vpu::calcClosestWaypoint(
    prev_output_, current_pose_ptr_->pose, planning_param_.delta_yaw_threshold);
  ROS_DEBUG(
    "[calcClosestWaypoint] base_resampled.size() = %lu, prev_planned_closest_ = %d",
    traj_resampled.points.size(), prev_output_closest);

  /* Calculate the closest trajectory point on the previously planned traj*/
  const auto prev_output_closest_point = vpu::calcClosestTrajecotoryPointWithIntepolation(
    prev_output_, traj_resampled.points.at(traj_resampled_closest).pose);

  /* Apply stopping velocity */
  applyStoppingVelocity(&traj_resampled);

  /* Optimize velocity */
  output = optimizeVelocity(
    traj_resampled, traj_resampled_closest, prev_output_, prev_output_closest_point);

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
  publishClosestJerk(output.points.at(traj_resampled_closest).accel.linear.x);
  if (publish_debug_trajs_) {
    pub_trajectory_raw_.publish(traj_extracted);
    pub_trajectory_vel_lim_.publish(traj_vel_limited);
    pub_trajectory_latcc_filtered_.publish(traj_latacc_filtered);
    pub_trajectory_resampled_.publish(traj_resampled);
  }

  return output;
}

void MotionVelocityOptimizer::insertBehindVelocity(
  const int prev_output_closest, const autoware_planning_msgs::Trajectory & prev_output,
  const int output_closest, autoware_planning_msgs::Trajectory & output) const
{
  const bool keep_closest_vel_for_behind =
    (initialize_type_ == InitializeType::INIT ||
     initialize_type_ == InitializeType::LARGE_DEVIATION_REPLAN ||
     initialize_type_ == InitializeType::ENGAGING);

  for (int i = output_closest - 1; i >= 0; --i) {
    if (keep_closest_vel_for_behind) {
      output.points.at(i).twist.linear.x = output.points.at(output_closest).twist.linear.x;
      output.points.at(i).accel.linear.x = output.points.at(output_closest).accel.linear.x;
    } else {
      const auto prev_output_closest_point =
        vpu::calcClosestTrajecotoryPointWithIntepolation(prev_output, output.points.at(i).pose);
      output.points.at(i).twist.linear.x = prev_output_closest_point.twist.linear.x;
      output.points.at(i).accel.linear.x = prev_output_closest_point.accel.linear.x;
    }
  }
}

void MotionVelocityOptimizer::publishStopDistance(
  const autoware_planning_msgs::Trajectory & trajectory, const int closest) const
{
  /* stop distance calculation */
  int stop_idx = 0;
  const double stop_dist_lim = 50.0;
  double stop_dist = stop_dist_lim;
  if (vpu::searchZeroVelocityIdx(trajectory, stop_idx)) {
    stop_dist = vpu::calcLengthOnWaypoints(trajectory, closest, stop_idx);
  }
  stop_dist = closest > stop_idx ? stop_dist : -stop_dist;
  std_msgs::Float32 dist_to_stopline;
  dist_to_stopline.data = std::max(-stop_dist_lim, std::min(stop_dist_lim, stop_dist));
  pub_dist_to_stopline_.publish(dist_to_stopline);
}

bool MotionVelocityOptimizer::resampleTrajectory(
  const autoware_planning_msgs::Trajectory & input,
  autoware_planning_msgs::Trajectory & output) const
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
      if (dist_i > planning_param_.min_trajectory_length)
        break;  // planning time is enough and planning distance is over min length.
      // if the planning time is not enough to see the desired distance, change the interval distance to see far.
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
    ROS_WARN(
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
  const double & target_vel, const autoware_planning_msgs::Trajectory & reference_traj,
  const int reference_traj_closest, const autoware_planning_msgs::Trajectory & prev_output,
  const autoware_planning_msgs::TrajectoryPoint & prev_output_point, double & initial_vel,
  double & initial_acc)
{
  const double vehicle_speed = std::fabs(current_velocity_ptr_->twist.linear.x);

  /* first time */
  if (prev_output.points.empty()) {
    initial_vel = vehicle_speed;
    initial_acc = 0.0;  // if possible, use actual vehicle acc & jerk value;
    initialize_type_ = InitializeType::INIT;
    return;
  }

  /* when velocity tracking deviation is large */
  const double desired_vel = prev_output_point.twist.linear.x;
  const double vel_error = vehicle_speed - std::fabs(desired_vel);
  if (std::fabs(vel_error) > planning_param_.replan_vel_deviation) {
    initialize_type_ = InitializeType::LARGE_DEVIATION_REPLAN;
    initial_vel = vehicle_speed;  // use current vehicle speed
    initial_acc = prev_output_point.accel.linear.x;
    ROS_DEBUG(
      "[calcInitialMotion] : Large deviation error for speed control. Use current speed for "
      "initial value, desired_vel = %f, vehicle_speed = %f, vel_error = %f, error_thr = %f",
      desired_vel, vehicle_speed, vel_error, planning_param_.replan_vel_deviation);
    return;
  }

  /* if current vehicle velocity is low && base_desired speed is high, use engage_velocity for engage vehicle */
  const double engage_vel_thr = planning_param_.engage_velocity * planning_param_.engage_exit_ratio;
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
        ROS_DEBUG(
          "[calcInitialMotion]: vehicle speed is low (%.3f), and desired speed is high (%.3f). Use "
          "engage speed (%.3f) until vehicle speed reaches engage_vel_thr (%.3f). stop_dist = %.3f",
          vehicle_speed, target_vel, planning_param_.engage_velocity, engage_vel_thr, stop_dist);
        return;
      } else {
        ROS_WARN_THROTTLE(
          3.0, "[calcInitialMotion]: stop point is close (%.3f[m]). no engage.", stop_dist);
      }
    } else if (target_vel > 0.0) {
      ROS_WARN_THROTTLE(
        3.0,
        "[calcInitialMotion]: target velocity(%.3f[m/s]) is lower than engage "
        "velocity(%.3f[m/s]). ",
        target_vel, planning_param_.engage_velocity);
    }
  }

  /* normal update: use closest in prev_output */
  initialize_type_ = InitializeType::NORMAL;
  initial_vel = prev_output_point.twist.linear.x;
  initial_acc = prev_output_point.accel.linear.x;
  ROS_DEBUG(
    "[calcInitialMotion]: normal update. v0 = %f, a0 = %f, vehicle_speed = %f, target_vel = %f",
    initial_vel, initial_acc, vehicle_speed, target_vel);
  return;
}

autoware_planning_msgs::Trajectory MotionVelocityOptimizer::optimizeVelocity(
  const autoware_planning_msgs::Trajectory & input, const int input_closest,
  const autoware_planning_msgs::Trajectory & prev_output,
  const autoware_planning_msgs::TrajectoryPoint & prev_output_point)
{
  const double target_vel = std::fabs(input.points.at(input_closest).twist.linear.x);

  /* calculate initial motion for planning */
  double initial_vel = 0.0;
  double initial_acc = 0.0;
  calcInitialMotion(
    target_vel, input, input_closest, prev_output, prev_output_point,
    /* out */ initial_vel, initial_acc);

  autoware_planning_msgs::Trajectory optimized_traj;
  bool res = optimizer_->solve(initial_vel, initial_acc, input_closest, input, &optimized_traj);
  if (!res) {
    // ROS_WARN("[optimizeVelocity] fail to solve optimization.");
  }

  /* set 0 velocity after input-stop-point */
  overwriteStopPoint(input, &optimized_traj);

  /* for the endpoint of the trajectory */
  if (optimized_traj.points.size() > 0) {
    optimized_traj.points.back().twist.linear.x = 0.0;
  }

  /* set output trajectory */
  ROS_DEBUG("[optimizeVelocity]: optimized_traj.size() = %lu", optimized_traj.points.size());
  return optimized_traj;
}

void MotionVelocityOptimizer::overwriteStopPoint(
  const autoware_planning_msgs::Trajectory & input,
  autoware_planning_msgs::Trajectory * output) const
{
  int stop_idx = -1;
  bool stop_point_exists = vpu::searchZeroVelocityIdx(input, stop_idx);

  // check over velocity
  bool is_stop_velocity_exceeded = false;
  if (stop_point_exists) {
    double optimized_stop_point_vel = output->points.at(stop_idx).twist.linear.x;
    is_stop_velocity_exceeded = (optimized_stop_point_vel > over_stop_velocity_warn_thr_);
  }
  {
    std_msgs::Bool msg;
    msg.data = is_stop_velocity_exceeded;
    pub_over_stop_velocity_.publish(msg);
  }

  {
    double input_stop_vel = stop_point_exists ? input.points.at(stop_idx).twist.linear.x : -1.0;
    double output_stop_vel = stop_point_exists ? output->points.at(stop_idx).twist.linear.x : -1.0;
    ROS_DEBUG(
      "[replan]: input_stop_idx = %d, stop velocity : input = %f, output = %f, thr = %f", stop_idx,
      input_stop_vel, output_stop_vel, over_stop_velocity_warn_thr_);
  }

  // keep stop point at the same position
  vpu::insertZeroVelocityAfterIdx(stop_idx, *output);
}

bool MotionVelocityOptimizer::lateralAccelerationFilter(
  const autoware_planning_msgs::Trajectory & input,
  autoware_planning_msgs::Trajectory & output) const
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
    ROS_WARN("[motion_velocity_optimizer]: interpolation failed at lateral acceleration filter.");
    return false;
  }
  output.points.back().twist = input.points.back().twist;  // keep the final speed.

  constexpr double curvature_calc_dist = 3.0;  // [m] calc curvature with 3m away points
  const unsigned int idx_dist = std::max((int)(curvature_calc_dist / points_interval), 1);

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
  const autoware_planning_msgs::Trajectory & input,
  autoware_planning_msgs::Trajectory & output) const
{
  output = input;
  if (!external_velocity_limit_filtered_) return false;

  vpu::maximumVelocityFilter(*external_velocity_limit_filtered_, output);
  ROS_DEBUG("[externalVelocityLimit]: limit_vel = %.3f", *external_velocity_limit_filtered_);
  return true;
}

void MotionVelocityOptimizer::preventMoveToCloseStopLine(
  const int closest, autoware_planning_msgs::Trajectory & traj) const
{
  if (std::fabs(current_velocity_ptr_->twist.linear.x) > 0.1) {
    return;
  }

  int stop_idx = 0;
  if (!vpu::searchZeroVelocityIdx(traj, stop_idx)) return;  // no obstacle.

  /* if the desired stop line is ahead of ego-vehicle */
  double dist_to_stopline = vpu::calcDist2d(traj.points.at(stop_idx), traj.points.at(closest));
  std::string debug_msg;
  if (stop_idx >= closest && dist_to_stopline < planning_param_.stop_dist_to_prohibit_engage) {
    vpu::setZeroVelocity(traj);
    debug_msg = "curr_vel is low, and stop line is close. keep stopping.";
  } else {
    debug_msg = "curr_vel is low, but stop point is far. move.";
  }

  ROS_DEBUG(
    "[preventMoveToCloseStopLine] %s curr_vel = %.3f, dist_to_stopline = %.3f, move_dist_min = "
    "%.3f, stop_idx = %d, closest = %d",
    debug_msg.c_str(), current_velocity_ptr_->twist.linear.x, dist_to_stopline,
    planning_param_.stop_dist_to_prohibit_engage, stop_idx, closest);
}

bool MotionVelocityOptimizer::extractPathAroundIndex(
  const autoware_planning_msgs::Trajectory & input, const int index,
  autoware_planning_msgs::Trajectory & output) const
{
  const double ahead_length = planning_param_.extract_ahead_dist;
  const double behind_length = planning_param_.extract_behind_dist;

  if (input.points.size() == 0 || index < 0 || (int)input.points.size() - 1 < index) {
    ROS_WARN(
      "extractPathAroundIndex failed. input.points.size() = %lu, base_index = %d",
      input.points.size(), index);
    return false;
  }

  double dist_sum_tmp = 0.0;

  // calc ahead distance
  int ahead_index = input.points.size() - 1;
  for (int i = index; i < (int)input.points.size() - 1; ++i) {
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

  // extract trajectory
  output.points.clear();
  for (int i = behind_index; i < ahead_index + 1; ++i) {
    output.points.push_back(input.points.at(i));
  }
  output.header = input.header;

  ROS_DEBUG(
    "[extractPathAroundIndex] : input.size() = %lu, extract_base_index = %d, output.size() = %lu",
    input.points.size(), index, output.points.size());
  return true;
}

void MotionVelocityOptimizer::applyStoppingVelocity(autoware_planning_msgs::Trajectory * traj) const
{
  int stop_idx;
  if (!vpu::searchZeroVelocityIdx(*traj, stop_idx)) return;  // no stop point.

  double distance_sum = 0.0;
  for (int i = stop_idx - 1; i >= 0; --i) {  // search backward
    distance_sum += vpu::calcDist2d(traj->points.at(i), traj->points.at(i + 1));
    if (distance_sum > planning_param_.stopping_distance) break;
    if (traj->points.at(i).twist.linear.x > planning_param_.stopping_velocity) {
      traj->points.at(i).twist.linear.x = planning_param_.stopping_velocity;
    }
  }
  return;
}

void MotionVelocityOptimizer::publishFloat(const double & data, const ros::Publisher & pub) const
{
  std_msgs::Float32 msg;
  msg.data = data;
  pub.publish(msg);
}

void MotionVelocityOptimizer::updateExternalVelocityLimit(const double dt)
{
  if (!external_velocity_limit_ptr_) return;

  if (external_velocity_limit_ptr_->data < -1.0e-5) {
    ROS_WARN("external velocity limit is negative. The command is ignored");
    return;
  }

  double v_lim = std::min<double>(external_velocity_limit_ptr_->data, planning_param_.max_velocity);

  if (!external_velocity_limit_filtered_) {
    external_velocity_limit_filtered_ = boost::make_shared<double>(v_lim);
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

void MotionVelocityOptimizer::timerCallback(const ros::TimerEvent & e)
{
  const double dt = (e.current_expected - e.last_expected).toSec();
  updateExternalVelocityLimit(dt);
}

void MotionVelocityOptimizer::publishClosestJerk(const double curr_acc)
{
  if (!prev_time_) {
    prev_time_ = std::make_shared<ros::Time>(ros::Time::now());
    prev_acc_ = curr_acc;
    return;
  }
  ros::Time curr_time = ros::Time::now();
  double dt = (curr_time - *prev_time_).toSec();
  double curr_jerk = (curr_acc - prev_acc_) / dt;
  publishFloat(curr_jerk, debug_closest_jerk_);
  prev_acc_ = curr_acc;
  *prev_time_ = curr_time;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "motion_velocity_optimizer");
  MotionVelocityOptimizer obj;

  ros::spin();

  return 0;
}
