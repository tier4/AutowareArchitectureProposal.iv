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

#include "mpc_follower/mpc_follower_core.h"

#define DEG2RAD 3.1415926535 / 180.0
#define RAD2DEG 180.0 / 3.1415926535

MPCFollower::MPCFollower() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  pnh_.param<double>("ctrl_period", ctrl_period_, 0.03);
  pnh_.param<bool>("enable_path_smoothing", enable_path_smoothing_, true);
  pnh_.param<bool>("enable_yaw_recalculation", enable_yaw_recalculation_, false);
  pnh_.param<bool>("use_steer_prediction", use_steer_prediction_, false);
  pnh_.param<int>("path_filter_moving_ave_num", path_filter_moving_ave_num_, 35);
  pnh_.param<int>("curvature_smoothing_num", curvature_smoothing_num_, 35);
  pnh_.param<double>("traj_resample_dist", traj_resample_dist_, 0.1);  // [m]
  pnh_.param<double>("admisible_position_error", admisible_position_error_, 5.0);
  pnh_.param<double>("admisible_yaw_error", admisible_yaw_error_, M_PI_2);
  pnh_.param<double>("vehicle_model_steer_tau", mpc_param_.steer_tau, 0.1);

  /* stop state parameters */
  pnh_.param<double>("stop_state_entry_ego_speed", stop_state_entry_ego_speed_, 0.2);  // [m/s]
  pnh_.param<double>(
    "stop_state_entry_target_speed", stop_state_entry_target_speed_, 0.1);  // [m/s]
  pnh_.param<double>(
    "stop_state_keep_stopping_dist", stop_state_keep_stopping_dist_, 0.5);  // [m/s]

  /* mpc parameters */
  pnh_.param<double>("/vehicle_info/wheel_base", wheelbase_, 2.9);

  /* vehicle model setup */
  pnh_.param("vehicle_model_type", vehicle_model_type_, std::string("kinematics"));
  if (vehicle_model_type_ == "kinematics") {
    vehicle_model_ptr_ =
      std::make_shared<KinematicsBicycleModel>(wheelbase_, steer_lim_, mpc_param_.steer_tau);
  } else if (vehicle_model_type_ == "kinematics_no_delay") {
    vehicle_model_ptr_ = std::make_shared<KinematicsBicycleModelNoDelay>(wheelbase_, steer_lim_);
  } else if (vehicle_model_type_ == "dynamics") {
    double mass_fl, mass_fr, mass_rl, mass_rr, cf, cr;
    pnh_.param<double>("mass_fl", mass_fl, 600);
    pnh_.param<double>("mass_fr", mass_fr, 600);
    pnh_.param<double>("mass_rl", mass_rl, 600);
    pnh_.param<double>("mass_rr", mass_rr, 600);
    pnh_.param<double>("cf", cf, 155494.663);
    pnh_.param<double>("cr", cr, 155494.663);
    vehicle_model_ptr_ = std::make_shared<DynamicsBicycleModel>(
      wheelbase_, mass_fl, mass_fr, mass_rl, mass_rr, cf, cr);
  } else {
    ROS_ERROR("[MPC] vehicle_model_type is undefined");
  }

  /* QP solver setup */
  std::string qp_solver_type;
  pnh_.param("qp_solver_type", qp_solver_type, std::string("unconstraint_fast"));
  if (qp_solver_type == "unconstraint_fast") {
    qpsolver_ptr_ = std::make_shared<QPSolverEigenLeastSquareLLT>();
  } else if (qp_solver_type == "osqp") {
    qpsolver_ptr_ = std::make_shared<QPSolverOSQP>();
  } else {
    ROS_ERROR("[MPC] qp_solver_type is undefined");
  }

  /* delay compensation */
  {
    double delay_tmp;
    pnh_.param<double>("input_delay", delay_tmp, 0.0);
    const int delay_step = std::round(delay_tmp / ctrl_period_);
    mpc_param_.input_delay = delay_step * ctrl_period_;
    input_buffer_ = std::deque<double>(delay_step, 0.0);
  }

  /* initialize lowpass filter */
  {
    double steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz;
    pnh_.param<double>("steering_lpf_cutoff_hz", steering_lpf_cutoff_hz, 3.0);
    pnh_.param<double>("error_deriv_lpf_cutoff_hz", error_deriv_lpf_cutoff_hz, 5.0);
    lpf_steering_cmd_.initialize(ctrl_period_, steering_lpf_cutoff_hz);
    lpf_lateral_error_.initialize(ctrl_period_, error_deriv_lpf_cutoff_hz);
    lpf_yaw_error_.initialize(ctrl_period_, error_deriv_lpf_cutoff_hz);
  }

  /* set up ros system */
  timer_control_ = nh_.createTimer(ros::Duration(ctrl_period_), &MPCFollower::onTimer, this);
  sub_ref_path_ = pnh_.subscribe("input/reference_trajectory", 1, &MPCFollower::onTrajectory, this);
  sub_current_vel_ = pnh_.subscribe("input/current_velocity", 1, &MPCFollower::onVelocity, this);
  sub_steering_ = pnh_.subscribe("input/current_steering", 1, &MPCFollower::onSteering, this);
  pub_ctrl_cmd_ =
    pnh_.advertise<autoware_control_msgs::ControlCommandStamped>("output/control_raw", 1);
  pub_predicted_traj_ =
    pnh_.advertise<autoware_planning_msgs::Trajectory>("output/predicted_trajectory", 1);

  /* wait to get vehicle position */
  while (ros::ok()) {
    try {
      tf_buffer_.lookupTransform("map", "base_link", ros::Time::now(), ros::Duration(5.0));
      break;
    } catch (tf2::TransformException & ex) {
      ROS_INFO("[mpc_follower] is waiting to get map to base_link transform. %s", ex.what());
      continue;
    }
  }

  /* for debug */
  pub_debug_marker_ = pnh_.advertise<visualization_msgs::MarkerArray>("debug/markers", 10);
  pub_debug_mpc_calc_time_ = pnh_.advertise<std_msgs::Float32>("debug/mpc_calc_time", 1);
  pub_debug_values_ = pnh_.advertise<std_msgs::Float32MultiArray>("debug/debug_values", 1);

  /* dynamic reconfigure */
  auto dyncon_f = boost::bind(&MPCFollower::dynamicRecofCallback, this, _1, _2);
  dyncon_server_.setCallback(dyncon_f);
}

MPCFollower::~MPCFollower()
{
  autoware_control_msgs::ControlCommand stop_cmd = getStopControlCommand();
  publishCtrlCmd(stop_cmd);
}

void MPCFollower::onTimer(const ros::TimerEvent & te)
{
  updateCurrentPose();

  if (!checkData()) {
    publishCtrlCmd(getStopControlCommand());
    return;
  }

  autoware_control_msgs::ControlCommand ctrl_cmd;

  if (!is_ctrl_cmd_prev_initialized_) {
    ctrl_cmd_prev_ = getInitialControlCommand();
    is_ctrl_cmd_prev_initialized_ = true;
  }

  const bool is_mpc_solved = calculateMPC(&ctrl_cmd);

  if (isStoppedState()) {
    // Reset input buffer
    for (auto & value : input_buffer_) {
      value = ctrl_cmd_prev_.steering_angle;
    }
    // Use previous command value as previous raw steer command
    raw_steer_cmd_prev_ = ctrl_cmd_prev_.steering_angle;

    publishCtrlCmd(ctrl_cmd_prev_);
    return;
  }

  if (!is_mpc_solved) {
    ROS_WARN_DELAYED_THROTTLE(5.0, "[MPC] MPC is not solved. publish 0 velocity.");
    ctrl_cmd = getStopControlCommand();
  }

  ctrl_cmd_prev_ = ctrl_cmd;
  publishCtrlCmd(ctrl_cmd);
}

bool MPCFollower::checkData()
{
  if (!vehicle_model_ptr_ || !qpsolver_ptr_) {
    ROS_DEBUG(
      "[MPC] vehicle_model = %d, qp_solver = %d", vehicle_model_ptr_ != nullptr,
      qpsolver_ptr_ != nullptr);
    return false;
  }

  if (!current_pose_ptr_ || !current_velocity_ptr_ || !current_steer_ptr_) {
    ROS_DEBUG(
      "[MPC] waiting data. pose = %d, velocity = %d, steer = %d", current_pose_ptr_ != nullptr,
      current_velocity_ptr_ != nullptr, current_steer_ptr_ != nullptr);
    return false;
  }

  if (ref_traj_.size() == 0) {
    ROS_DEBUG("[MPC] trajectory size is zero.");
    return false;
  }

  return true;
}

bool MPCFollower::calculateMPC(autoware_control_msgs::ControlCommand * ctrl_cmd)
{
  auto start = std::chrono::system_clock::now();

  if (!ctrl_cmd) {
    return false;
  }

  /* recalculate velocity from ego-velocity with dynamics */
  MPCTrajectory reference_trajectory =
    applyVelocityDynamicsFilter(ref_traj_, current_velocity_ptr_->twist.linear.x);

  MPCData mpc_data = {};
  if (!getData(reference_trajectory, &mpc_data)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[MPC] fail to get Data.");
    return false;
  }

  /* define initial state for error dynamics */
  Eigen::VectorXd x0 = getInitialState(mpc_data);

  /* delay compensation */
  if (!updateStateForDelayCompensation(reference_trajectory, mpc_data.nearest_time, &x0)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[MPC] delay compensation failed.");
    return false;
  }

  /* resample ref_traj with mpc sampling time */
  MPCTrajectory mpc_resampled_ref_traj;
  const double mpc_start_time = mpc_data.nearest_time + mpc_param_.input_delay;
  if (!resampleMPCTrajectoryByTime(mpc_start_time, reference_trajectory, &mpc_resampled_ref_traj)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[MPC] trajectory resampling failed.");
    return false;
  }

  /* generate mpc matrix : predict equation Xec = Aex * x0 + Bex * Uex + Wex */
  MPCMatrix mpc_matrix = generateMPCMatrix(mpc_resampled_ref_traj);

  /* solve quadratic optimization */
  Eigen::VectorXd Uex;
  if (!executeOptimization(mpc_matrix, x0, &Uex)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[MPC] optimization failed.");
    return false;
  }

  /* apply saturation and filter */
  const double u_saturated = std::max(std::min(Uex(0), steer_lim_), -steer_lim_);
  const double u_filtered = lpf_steering_cmd_.filter(u_saturated);

  /* set control command */
  {
    const auto & dt = mpc_param_.prediction_dt;
    const int prev_idx = std::max(0, static_cast<int>(mpc_data.nearest_idx) - 1);
    ctrl_cmd->steering_angle = u_filtered;
    ctrl_cmd->steering_angle_velocity = (Uex(1) - Uex(0)) / dt;
    ctrl_cmd->velocity = ref_traj_.vx[mpc_data.nearest_idx];
    ctrl_cmd->acceleration = (ref_traj_.vx[mpc_data.nearest_idx] - ref_traj_.vx[prev_idx]) / dt;
  }

  storeSteerCmd(u_filtered);

  /* save input to buffer for delay compensation*/
  input_buffer_.push_back(ctrl_cmd->steering_angle);
  input_buffer_.pop_front();
  raw_steer_cmd_pprev_ = raw_steer_cmd_prev_;
  raw_steer_cmd_prev_ = Uex(0);

  /* publish predicted trajectory */
  {
    Eigen::VectorXd Xex = mpc_matrix.Aex * x0 + mpc_matrix.Bex * Uex + mpc_matrix.Wex;
    MPCTrajectory mpc_predicted_traj;
    const auto & traj = mpc_resampled_ref_traj;
    for (int i = 0; i < mpc_param_.prediction_horizon; ++i) {
      const int DIM_X = vehicle_model_ptr_->getDimX();
      const double lat_error = Xex(i * DIM_X);
      const double yaw_error = Xex(i * DIM_X + 1);
      const double x = traj.x[i] - std::sin(traj.yaw[i]) * lat_error;
      const double y = traj.y[i] + std::cos(traj.yaw[i]) * lat_error;
      const double z = traj.z[i];
      const double yaw = traj.yaw[i] + yaw_error;
      const double vx = traj.vx[i];
      const double k = traj.k[i];
      const double smooth_k = traj.smooth_k[i];
      const double relative_time = traj.relative_time[i];
      mpc_predicted_traj.push_back(x, y, z, yaw, vx, k, smooth_k, relative_time);
    }

    autoware_planning_msgs::Trajectory predicted_traj;
    predicted_traj.header.stamp = current_trajectory_ptr_->header.stamp;
    predicted_traj.header.frame_id = current_trajectory_ptr_->header.frame_id;
    MPCUtils::convertToAutowareTrajectory(mpc_predicted_traj, &predicted_traj);
    pub_predicted_traj_.publish(predicted_traj);

    visualization_msgs::MarkerArray markers = MPCUtils::convertTrajToMarker(
      mpc_predicted_traj, "predicted_trajectory", 0.99, 0.99, 0.99, 0.2,
      current_trajectory_ptr_->header.frame_id);
    pub_debug_marker_.publish(markers);
  }

  /* publish debug values */
  {
    double curr_v = current_velocity_ptr_->twist.linear.x;
    double nearest_k = 0.0;
    if (!LinearInterpolate::interpolate(
          reference_trajectory.relative_time, reference_trajectory.k, mpc_data.nearest_time,
          nearest_k)) {
      ROS_WARN("[mpc] interpolate error in debug. ignore.");
      return true;
    }

    MPCTrajectory tmp_traj = reference_trajectory;
    MPCUtils::calcTrajectoryCurvature(1, &tmp_traj);
    double curvature_raw = tmp_traj.k[mpc_data.nearest_idx];
    double steer_cmd = ctrl_cmd->steering_angle;

    std_msgs::Float32MultiArray d;
    const auto & ps = mpc_data.predicted_steer;
    const auto & W = wheelbase_;
    d.data.push_back(steer_cmd);                 // [0] final steering command (MPC + LPF)
    d.data.push_back(Uex(0));                    // [1] mpc calculation result
    d.data.push_back(mpc_matrix.Urefex(0));      // [2] feedforward steering value
    d.data.push_back(std::atan(nearest_k * W));  // [3] feedforward steering value raw
    d.data.push_back(mpc_data.steer);            // [4] current steering angle
    d.data.push_back(mpc_data.lateral_err);      // [5] lateral error
    d.data.push_back(tf2::getYaw(current_pose_ptr_->pose.orientation));  // [6] current_pose yaw
    d.data.push_back(tf2::getYaw(mpc_data.nearest_pose.orientation));    // [7] nearest_pose yaw
    d.data.push_back(mpc_data.yaw_err);                                  // [8] yaw error
    d.data.push_back(ctrl_cmd->velocity);                                // [9] command velocities
    d.data.push_back(current_velocity_ptr_->twist.linear.x);             // [10] measured velocity
    d.data.push_back(curr_v * tan(steer_cmd) / W);       // [11] angvel from steer command
    d.data.push_back(curr_v * tan(mpc_data.steer) / W);  // [12] angvel from measured steer
    d.data.push_back(curr_v * nearest_k);    // [13] angvel from path curvature (Path angvel)
    d.data.push_back(nearest_k);             // [14] nearest path curvature (used for control)
    d.data.push_back(curvature_raw);         // [15] nearest path curvature (not smoothed)
    d.data.push_back(ps);                    // [16] predicted steer
    d.data.push_back(curr_v * tan(ps) / W);  // [17] angvel from predicted steer
    pub_debug_values_.publish(d);
  }

  /* publish computing time */
  {
    auto end = std::chrono::system_clock::now();
    auto t = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() * 1.0e-6;
    std_msgs::Float32 calc_time;
    calc_time.data = t;  // [ms]
    pub_debug_mpc_calc_time_.publish(calc_time);
  }

  return true;
}

bool MPCFollower::getData(const MPCTrajectory & traj, MPCData * data)
{
  if (!MPCUtils::calcNearestPoseInterp(
        traj, current_pose_ptr_->pose, &(data->nearest_pose), &(data->nearest_idx),
        &(data->nearest_time))) {
    ROS_WARN_DELAYED_THROTTLE(
      5.0, "[MPC] calculateMPC: error in calculating nearest pose. stop mpc.");
    return false;
  }

  /* get data */
  data->steer = current_steer_ptr_->data;
  data->lateral_err = MPCUtils::calcLateralError(current_pose_ptr_->pose, data->nearest_pose);
  data->yaw_err = MPCUtils::normalizeRadian(
    tf2::getYaw(current_pose_ptr_->pose.orientation) - tf2::getYaw(data->nearest_pose.orientation));

  /* get predicted steer */
  if (!steer_prediction_prev_) {
    steer_prediction_prev_ = std::make_shared<double>(current_steer_ptr_->data);
  }
  data->predicted_steer = calcSteerPrediction();
  *steer_prediction_prev_ = data->predicted_steer;

  /* check error limit */
  const double dist_err = MPCUtils::calcDist2d(current_pose_ptr_->pose, data->nearest_pose);
  if (dist_err > admisible_position_error_) {
    ROS_WARN_DELAYED_THROTTLE(
      5.0, "[MPC] position error is over limit. error = %fm, limit: %fm", dist_err,
      admisible_position_error_);
    return false;
  }
  /* check yaw error limit */
  if (std::fabs(data->yaw_err) > admisible_yaw_error_) {
    ROS_WARN_DELAYED_THROTTLE(
      5.0, "[MPC] yaw error is over limit. error = %fdeg, limit %fdeg", RAD2DEG * (data->yaw_err),
      RAD2DEG * admisible_yaw_error_);
    return false;
  }
  /* check trajectory time length */
  if (
    data->nearest_time + mpc_param_.input_delay + getPredictionTime() > traj.relative_time.back()) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[MPC] path is too short for prediction.");
    return false;
  }
  return true;
}

double MPCFollower::calcSteerPrediction()
{
  const double t_start = time_prev_;
  const double t_end = ros::Time::now().toSec();
  time_prev_ = t_end;

  const double duration = t_end - t_start;
  const double time_constant = mpc_param_.steer_tau;

  const double initial_response = std::exp(-duration / time_constant) * (*steer_prediction_prev_);

  if (ctrl_cmd_vec_.size() <= 2) return initial_response;

  return initial_response + getSteerCmdSum(t_start, t_end, time_constant);
}

double MPCFollower::getSteerCmdSum(
  const double t_start, const double t_end, const double time_constant)
{
  if (ctrl_cmd_vec_.size() <= 2) return 0.0;

  // Find first index of control command container
  size_t idx = 1;
  while (t_start > ctrl_cmd_vec_.at(idx).header.stamp.toSec()) {
    if ((idx + 1) >= ctrl_cmd_vec_.size()) return 0.0;
    ++idx;
  }

  // Compute steer command input response
  double steer_sum = 0.0;
  double t = t_start;
  while (t_end > ctrl_cmd_vec_.at(idx).header.stamp.toSec()) {
    const double duration = ctrl_cmd_vec_.at(idx).header.stamp.toSec() - t;
    t = ctrl_cmd_vec_.at(idx).header.stamp.toSec();
    steer_sum +=
      (1 - std::exp(-duration / time_constant)) * ctrl_cmd_vec_.at(idx - 1).control.steering_angle;
    ++idx;
    if (idx >= ctrl_cmd_vec_.size()) break;
  }

  const double duration = t_end - t;
  steer_sum +=
    (1 - std::exp(-duration / time_constant)) * ctrl_cmd_vec_.at(idx - 1).control.steering_angle;

  return steer_sum;
}

void MPCFollower::storeSteerCmd(const double steer)
{
  const auto time_delayed = ros::Time::now() + ros::Duration(mpc_param_.input_delay);
  autoware_control_msgs::ControlCommandStamped cmd;
  cmd.header.stamp = time_delayed;
  cmd.control.steering_angle = steer;

  // store published ctrl cmd
  ctrl_cmd_vec_.emplace_back(cmd);

  if (ctrl_cmd_vec_.size() <= 2) {
    return;
  }

  // remove unused ctrl cmd
  constexpr double store_time = 0.3;
  if (
    (time_delayed - ctrl_cmd_vec_.at(1).header.stamp).toSec() >
    mpc_param_.input_delay + store_time) {
    ctrl_cmd_vec_.erase(ctrl_cmd_vec_.begin());
  }
}

bool MPCFollower::resampleMPCTrajectoryByTime(
  double ts, const MPCTrajectory & input, MPCTrajectory * output) const
{
  std::vector<double> mpc_time_v;
  for (int i = 0; i < mpc_param_.prediction_horizon; ++i) {
    mpc_time_v.push_back(ts + i * mpc_param_.prediction_dt);
  }
  if (!MPCUtils::linearInterpMPCTrajectory(input.relative_time, input, mpc_time_v, output)) {
    ROS_WARN_DELAYED_THROTTLE(
      1.0, "[MPC] calculateMPC: mpc resample error. stop mpc calculation. check code!");
    return false;
  }
  return true;
}

Eigen::VectorXd MPCFollower::getInitialState(const MPCData & data)
{
  const int DIM_X = vehicle_model_ptr_->getDimX();
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(DIM_X);

  const auto & lat_err = data.lateral_err;
  const auto & steer = use_steer_prediction_ ? data.predicted_steer : data.steer;
  const auto & yaw_err = data.yaw_err;

  if (vehicle_model_type_ == "kinematics") {
    x0 << lat_err, yaw_err, steer;
  } else if (vehicle_model_type_ == "kinematics_no_delay") {
    x0 << lat_err, yaw_err;
  } else if (vehicle_model_type_ == "dynamics") {
    double dot_lat_err = (lat_err - lateral_error_prev_) / ctrl_period_;
    double dot_yaw_err = (yaw_err - yaw_error_prev_) / ctrl_period_;
    lateral_error_prev_ = lat_err;
    yaw_error_prev_ = yaw_err;
    dot_lat_err = lpf_lateral_error_.filter(dot_lat_err);
    dot_yaw_err = lpf_yaw_error_.filter(dot_yaw_err);
    x0 << lat_err, dot_lat_err, yaw_err, dot_yaw_err;
    ROS_DEBUG("[MPC] (before lpf) dot_lat_err = %f, dot_yaw_err = %f", dot_lat_err, dot_yaw_err);
    ROS_DEBUG("[MPC] (after lpf) dot_lat_err = %f, dot_yaw_err = %f", dot_lat_err, dot_yaw_err);
  } else {
    ROS_ERROR("vehicle_model_type is undefined");
  }
  return x0;
}

bool MPCFollower::updateStateForDelayCompensation(
  const MPCTrajectory & traj, const double & start_time, Eigen::VectorXd * x)
{
  const int DIM_X = vehicle_model_ptr_->getDimX();
  const int DIM_U = vehicle_model_ptr_->getDimU();
  const int DIM_Y = vehicle_model_ptr_->getDimY();

  Eigen::MatrixXd Ad(DIM_X, DIM_X);
  Eigen::MatrixXd Bd(DIM_X, DIM_U);
  Eigen::MatrixXd Wd(DIM_X, 1);
  Eigen::MatrixXd Cd(DIM_Y, DIM_X);

  Eigen::MatrixXd x_curr = *x;
  double mpc_curr_time = start_time;
  for (unsigned int i = 0; i < input_buffer_.size(); ++i) {
    double k = 0.0;
    double v = 0.0;
    if (
      !LinearInterpolate::interpolate(traj.relative_time, traj.k, mpc_curr_time, k) ||
      !LinearInterpolate::interpolate(traj.relative_time, traj.vx, mpc_curr_time, v)) {
      ROS_ERROR(
        "[MPC] mpc resample error at delay compensation, stop mpc calculation. check code!");
      return false;
    }

    /* get discrete state matrix A, B, C, W */
    vehicle_model_ptr_->setVelocity(v);
    vehicle_model_ptr_->setCurvature(k);
    vehicle_model_ptr_->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, ctrl_period_);
    Eigen::MatrixXd ud = Eigen::MatrixXd::Zero(DIM_U, 1);
    ud(0, 0) = input_buffer_.at(i);  // for steering input delay
    x_curr = Ad * x_curr + Bd * ud + Wd;
    mpc_curr_time += ctrl_period_;
  }
  *x = x_curr;
  return true;
}

MPCTrajectory MPCFollower::applyVelocityDynamicsFilter(const MPCTrajectory & input, const double v0)
{
  int nearest_idx = MPCUtils::calcNearestIndex(input, current_pose_ptr_->pose);
  if (nearest_idx < 0) return input;

  const double alim = mpc_param_.acceleration_limit;
  const double tau = mpc_param_.velocity_time_constant;

  MPCTrajectory output = input;
  MPCUtils::dynamicSmoothingVelocity(nearest_idx, v0, alim, tau, &output);
  const double t_ext = 100.0;  // extra time to prevent mpc calculation failure due to short time
  const double t_end = output.relative_time.back() + getPredictionTime() + t_ext;
  const double v_end = 0.0;
  output.vx.back() = v_end;  // set for end point
  output.push_back(
    output.x.back(), output.y.back(), output.z.back(), output.yaw.back(), v_end, output.k.back(),
    output.smooth_k.back(), t_end);
  return output;
}

/*
 * predict equation: Xec = Aex * x0 + Bex * Uex + Wex
 * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Urefex) + Uex' * R2ex * Uex
 * Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
 */
MPCFollower::MPCMatrix MPCFollower::generateMPCMatrix(const MPCTrajectory & reference_trajectory)
{
  using Eigen::MatrixXd;

  const int N = mpc_param_.prediction_horizon;
  const double DT = mpc_param_.prediction_dt;
  const int DIM_X = vehicle_model_ptr_->getDimX();
  const int DIM_U = vehicle_model_ptr_->getDimU();
  const int DIM_Y = vehicle_model_ptr_->getDimY();

  MPCMatrix m;
  m.Aex = MatrixXd::Zero(DIM_X * N, DIM_X);
  m.Bex = MatrixXd::Zero(DIM_X * N, DIM_U * N);
  m.Wex = MatrixXd::Zero(DIM_X * N, 1);
  m.Cex = MatrixXd::Zero(DIM_Y * N, DIM_X * N);
  m.Qex = MatrixXd::Zero(DIM_Y * N, DIM_Y * N);
  m.R1ex = MatrixXd::Zero(DIM_U * N, DIM_U * N);
  m.R2ex = MatrixXd::Zero(DIM_U * N, DIM_U * N);
  m.Urefex = MatrixXd::Zero(DIM_U * N, 1);

  /* weight matrix depends on the vehicle model */
  MatrixXd Q = MatrixXd::Zero(DIM_Y, DIM_Y);
  MatrixXd R = MatrixXd::Zero(DIM_U, DIM_U);
  MatrixXd Q_adaptive = MatrixXd::Zero(DIM_Y, DIM_Y);
  MatrixXd R_adaptive = MatrixXd::Zero(DIM_U, DIM_U);

  MatrixXd Ad(DIM_X, DIM_X);
  MatrixXd Bd(DIM_X, DIM_U);
  MatrixXd Wd(DIM_X, 1);
  MatrixXd Cd(DIM_Y, DIM_X);
  MatrixXd Uref(DIM_U, 1);

  constexpr double ep = 1.0e-3;  // large enough to ignore velocity noise

  /* predict dynamics for N times */
  for (int i = 0; i < N; ++i) {
    const double ref_vx = reference_trajectory.vx[i];
    const double ref_vx_squared = ref_vx * ref_vx;

    // curvature will be 0 when vehicle stops
    const double ref_k = reference_trajectory.k[i] * sign_vx_;
    const double ref_smooth_k = reference_trajectory.smooth_k[i] * sign_vx_;

    /* get discrete state matrix A, B, C, W */
    vehicle_model_ptr_->setVelocity(ref_vx);
    vehicle_model_ptr_->setCurvature(ref_k);
    vehicle_model_ptr_->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, DT);

    Q = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
    R = Eigen::MatrixXd::Zero(DIM_U, DIM_U);
    Q(0, 0) = getWeightLatError(ref_k);
    Q(1, 1) = getWeightHeadingError(ref_k);
    R(0, 0) = getWeightSteerInput(ref_k);

    Q_adaptive = Q;
    R_adaptive = R;
    if (i == N - 1) {
      Q_adaptive(0, 0) = mpc_param_.weight_terminal_lat_error;
      Q_adaptive(1, 1) = mpc_param_.weight_terminal_heading_error;
    }
    Q_adaptive(1, 1) += ref_vx_squared * getWeightHeadingErrorSqVel(ref_k);
    R_adaptive(0, 0) += ref_vx_squared * getWeightSteerInputSqVel(ref_k);

    /* update mpc matrix */
    int idx_x_i = i * DIM_X;
    int idx_x_i_prev = (i - 1) * DIM_X;
    int idx_u_i = i * DIM_U;
    int idx_y_i = i * DIM_Y;
    if (i == 0) {
      m.Aex.block(0, 0, DIM_X, DIM_X) = Ad;
      m.Bex.block(0, 0, DIM_X, DIM_U) = Bd;
      m.Wex.block(0, 0, DIM_X, 1) = Wd;
    } else {
      m.Aex.block(idx_x_i, 0, DIM_X, DIM_X) = Ad * m.Aex.block(idx_x_i_prev, 0, DIM_X, DIM_X);
      for (int j = 0; j < i; ++j) {
        int idx_u_j = j * DIM_U;
        m.Bex.block(idx_x_i, idx_u_j, DIM_X, DIM_U) =
          Ad * m.Bex.block(idx_x_i_prev, idx_u_j, DIM_X, DIM_U);
      }
      m.Wex.block(idx_x_i, 0, DIM_X, 1) = Ad * m.Wex.block(idx_x_i_prev, 0, DIM_X, 1) + Wd;
    }
    m.Bex.block(idx_x_i, idx_u_i, DIM_X, DIM_U) = Bd;
    m.Cex.block(idx_y_i, idx_x_i, DIM_Y, DIM_X) = Cd;
    m.Qex.block(idx_y_i, idx_y_i, DIM_Y, DIM_Y) = Q_adaptive;
    m.R1ex.block(idx_u_i, idx_u_i, DIM_U, DIM_U) = R_adaptive;

    /* get reference input (feed-forward) */
    vehicle_model_ptr_->setCurvature(ref_smooth_k);
    vehicle_model_ptr_->calculateReferenceInput(Uref);
    if (std::fabs(Uref(0, 0)) < DEG2RAD * mpc_param_.zero_ff_steer_deg) {
      Uref(0, 0) = 0.0;  // ignore curvature noise
    }
    m.Urefex.block(i * DIM_U, 0, DIM_U, 1) = Uref;
  }

  /* add lateral jerk : weight for (v * {u(i) - u(i-1)} )^2 */
  for (int i = 0; i < N - 1; ++i) {
    const double ref_vx = reference_trajectory.vx[i];
    sign_vx_ = ref_vx > ep ? 1 : (ref_vx < -ep ? -1 : sign_vx_);
    const double ref_k = reference_trajectory.k[i] * sign_vx_;
    const double j = ref_vx * ref_vx * getWeightLatJerk(ref_k) / (DT * DT);  // lateral jerk weight
    const Eigen::Matrix2d J = (Eigen::Matrix2d() << j, -j, -j, j).finished();
    m.R2ex.block(i, i, 2, 2) += J;
  }

  addSteerWeightR(&m.R1ex);

  return m;
}

/*
 * solve quadratic optimization.
 * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Urefex) + Uex' * R2ex * Uex
 *                , Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
 * constraint matrix : lb < U < ub, lbA < A*U < ubA
 * current considered constraint
 *  - steering limit
 *  - steering rate limit
 *
 * (1)lb < u < ub && (2)lbA < Au < ubA --> (3)[lb, lbA] < [I, A]u < [ub, ubA]
 * (1)lb < u < ub ...
 * [-u_lim] < [ u0 ] < [u_lim]
 * [-u_lim] < [ u1 ] < [u_lim]
 *              ~~~
 * [-u_lim] < [ uN ] < [u_lim] (*N... DIM_U)
 * (2)lbA < Au < ubA ...
 * [prev_u0 - au_lim*ctp] < [   u0  ] < [prev_u0 + au_lim*ctp] (*ctp ... ctrl_period)
 * [    -au_lim * dt    ] < [u1 - u0] < [     au_lim * dt    ]
 * [    -au_lim * dt    ] < [u2 - u1] < [     au_lim * dt    ]
 *                            ~~~
 * [    -au_lim * dt    ] < [uN-uN-1] < [     au_lim * dt    ] (*N... DIM_U)
 */
bool MPCFollower::executeOptimization(
  const MPCMatrix & m, const Eigen::VectorXd & x0, Eigen::VectorXd * Uex)
{
  using Eigen::MatrixXd;
  using Eigen::VectorXd;

  if (!isValid(m)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[MPC] model matrix includes NaN, stop MPC.");
    return false;
  }

  const int DIM_U_N = mpc_param_.prediction_horizon * vehicle_model_ptr_->getDimU();

  // cost function: 1/2 * Uex' * H * Uex + f' * Uex,  H = B' * C' * Q * C * B + R
  const MatrixXd CB = m.Cex * m.Bex;
  const MatrixXd QCB = m.Qex * CB;
  // MatrixXd H = CB.transpose() * QCB + m.R1ex + m.R2ex; // This calculation is heavy. looking for a good way.
  MatrixXd H = MatrixXd::Zero(DIM_U_N, DIM_U_N);
  H.triangularView<Eigen::Upper>() = CB.transpose() * QCB;
  H.triangularView<Eigen::Upper>() += m.R1ex + m.R2ex;
  H.triangularView<Eigen::Lower>() = H.transpose();
  MatrixXd f = (m.Cex * (m.Aex * x0 + m.Wex)).transpose() * QCB - m.Urefex.transpose() * m.R1ex;
  addSteerWeightF(&f);

  MatrixXd A = MatrixXd::Identity(DIM_U_N, DIM_U_N);
  for (int i = 1; i < DIM_U_N; i++) {
    A(i, i - 1) = -1.0;
  }

  VectorXd lb = VectorXd::Constant(DIM_U_N, -steer_lim_);  // min steering angle
  VectorXd ub = VectorXd::Constant(DIM_U_N, steer_lim_);   // max steering angle
  VectorXd lbA = VectorXd::Constant(DIM_U_N, -steer_rate_lim_ * mpc_param_.prediction_dt);
  VectorXd ubA = VectorXd::Constant(DIM_U_N, steer_rate_lim_ * mpc_param_.prediction_dt);
  lbA(0, 0) = raw_steer_cmd_prev_ - steer_rate_lim_ * ctrl_period_;
  ubA(0, 0) = raw_steer_cmd_prev_ + steer_rate_lim_ * ctrl_period_;

  auto t_start = std::chrono::system_clock::now();
  bool solve_result = qpsolver_ptr_->solve(H, f.transpose(), A, lb, ub, lbA, ubA, *Uex);
  auto t_end = std::chrono::system_clock::now();
  if (!solve_result) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[MPC] qp solver error");
    return false;
  }

  {
    auto t = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start).count() * 1.0e-6;
    ROS_DEBUG("[MPC] qp solver calculation time = %f [ms]", t);
  }

  if (Uex->array().isNaN().any()) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[MPC] model Uex includes NaN, stop MPC. ");
    return false;
  }
  return true;
}

void MPCFollower::addSteerWeightR(Eigen::MatrixXd * R_ptr) const
{
  const int N = mpc_param_.prediction_horizon;
  const double DT = mpc_param_.prediction_dt;

  auto & R = *R_ptr;

  /* add steering rate : weight for (u(i) - u(i-1) / dt )^2 */
  {
    const double steer_rate_r = mpc_param_.weight_steer_rate / (DT * DT);
    const Eigen::Matrix2d D = steer_rate_r * (Eigen::Matrix2d() << 1.0, -1.0, -1.0, 1.0).finished();
    for (int i = 0; i < N - 1; ++i) {
      R.block(i, i, 2, 2) += D;
    }
    if (N > 1) {
      // steer rate i = 0
      R(0, 0) += mpc_param_.weight_steer_rate / (ctrl_period_ * ctrl_period_);
    }
  }

  /* add steering acceleration : weight for { (u(i+1) - 2*u(i) + u(i-1)) / dt^2 }^2 */
  {
    const double w = mpc_param_.weight_steer_acc;
    const double steer_acc_r = w / std::pow(DT, 4);
    const double steer_acc_r_cp1 = w / (std::pow(DT, 3) * ctrl_period_);
    const double steer_acc_r_cp2 = w / (std::pow(DT, 2) * std::pow(ctrl_period_, 2));
    const double steer_acc_r_cp4 = w / std::pow(ctrl_period_, 4);
    const Eigen::Matrix3d D =
      steer_acc_r *
      (Eigen::Matrix3d() << 1.0, -2.0, 1.0, -2.0, 4.0, -2.0, 1.0, -2.0, 1.0).finished();
    for (int i = 1; i < N - 1; ++i) {
      R.block(i - 1, i - 1, 3, 3) += D;
    }
    if (N > 1) {
      // steer acc i = 1
      R(0, 0) += steer_acc_r * 1.0 + steer_acc_r_cp2 * 1.0 + steer_acc_r_cp1 * 2.0;
      R(1, 0) += steer_acc_r * -1.0 + steer_acc_r_cp1 * -1.0;
      R(0, 1) += steer_acc_r * -1.0 + steer_acc_r_cp1 * -1.0;
      R(1, 1) += steer_acc_r * 1.0;
      // steer acc i = 0
      R(0, 0) += steer_acc_r_cp4 * 1.0;
    }
  }
}

void MPCFollower::addSteerWeightF(Eigen::MatrixXd * f_ptr) const
{
  if (f_ptr->rows() < 2) {
    return;
  }

  const double DT = mpc_param_.prediction_dt;
  auto & f = *f_ptr;

  // steer rate for i = 0
  f(0, 0) += -2.0 * mpc_param_.weight_steer_rate / (std::pow(DT, 2)) * 0.5;

  // const double steer_acc_r = mpc_param_.weight_steer_acc / std::pow(DT, 4);
  const double steer_acc_r_cp1 = mpc_param_.weight_steer_acc / (std::pow(DT, 3) * ctrl_period_);
  const double steer_acc_r_cp2 =
    mpc_param_.weight_steer_acc / (std::pow(DT, 2) * std::pow(ctrl_period_, 2));
  const double steer_acc_r_cp4 = mpc_param_.weight_steer_acc / std::pow(ctrl_period_, 4);

  // steer acc  i = 0
  f(0, 0) += ((-2.0 * raw_steer_cmd_prev_ + raw_steer_cmd_pprev_) * steer_acc_r_cp4) * 0.5;

  // steer acc for i = 1
  f(0, 0) += (-2.0 * raw_steer_cmd_prev_ * (steer_acc_r_cp1 + steer_acc_r_cp2)) * 0.5;
  f(0, 1) += (2.0 * raw_steer_cmd_prev_ * steer_acc_r_cp1) * 0.5;
}

double MPCFollower::getPredictionTime() const
{
  return (mpc_param_.prediction_horizon - 1) * mpc_param_.prediction_dt + mpc_param_.input_delay +
         ctrl_period_;
}

bool MPCFollower::isValid(const MPCMatrix & m) const
{
  if (
    m.Aex.array().isNaN().any() || m.Bex.array().isNaN().any() || m.Cex.array().isNaN().any() ||
    m.Wex.array().isNaN().any() || m.Qex.array().isNaN().any() || m.R1ex.array().isNaN().any() ||
    m.R2ex.array().isNaN().any() || m.Urefex.array().isNaN().any()) {
    return false;
  }

  if (
    m.Aex.array().isInf().any() || m.Bex.array().isInf().any() || m.Cex.array().isInf().any() ||
    m.Wex.array().isInf().any() || m.Qex.array().isInf().any() || m.R1ex.array().isInf().any() ||
    m.R2ex.array().isInf().any() || m.Urefex.array().isInf().any()) {
    return false;
  }

  return true;
}

void MPCFollower::onTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr & msg)
{
  current_trajectory_ptr_ = msg;

  if (msg->points.size() < 3) {
    ROS_DEBUG("[MPC] received path size is < 3, not enough.");
    return;
  }

  if (!isValidTrajectory(*msg)) {
    ROS_ERROR("[MPC] Trajectory is invalid!! stop computing.");
    return;
  }

  MPCTrajectory mpc_traj_raw;        // received raw trajectory
  MPCTrajectory mpc_traj_resampled;  // resampled trajectory
  MPCTrajectory mpc_traj_smoothed;   // smooth filtered trajectory

  /* resampling */
  MPCUtils::convertToMPCTrajectory(*current_trajectory_ptr_, &mpc_traj_raw);
  if (!MPCUtils::resampleMPCTrajectoryByDistance(
        mpc_traj_raw, traj_resample_dist_, &mpc_traj_resampled)) {
    ROS_WARN("spline error!!!!!!");
    return;
  }

  /* path smoothing */
  mpc_traj_smoothed = mpc_traj_resampled;
  int mpc_traj_resampled_size = static_cast<int>(mpc_traj_resampled.size());
  if (enable_path_smoothing_ && mpc_traj_resampled_size > 2 * path_filter_moving_ave_num_) {
    if (
      !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, mpc_traj_smoothed.x) ||
      !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, mpc_traj_smoothed.y) ||
      !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, mpc_traj_smoothed.yaw) ||
      !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, mpc_traj_smoothed.vx)) {
      ROS_DEBUG("[MPC] path callback: filtering error. stop filtering.");
      mpc_traj_smoothed = mpc_traj_resampled;
    }
  }

  /* calculate yaw angle */
  if (enable_yaw_recalculation_) {
    MPCUtils::calcTrajectoryYawFromXY(&mpc_traj_smoothed);
    MPCUtils::convertEulerAngleToMonotonic(&mpc_traj_smoothed.yaw);
  }

  /* calculate curvature */
  MPCUtils::calcTrajectoryCurvature(curvature_smoothing_num_, &mpc_traj_smoothed);

  /* add end point with vel=0 on traj for mpc prediction */
  {
    auto & t = mpc_traj_smoothed;
    const double t_ext = 100.0;  // extra time to prevent mpc calculation failure due to short time
    const double t_end = t.relative_time.back() + getPredictionTime() + t_ext;
    const double v_end = 0.0;
    t.vx.back() = v_end;  // set for end point
    t.push_back(
      t.x.back(), t.y.back(), t.z.back(), t.yaw.back(), v_end, t.k.back(), t.smooth_k.back(),
      t_end);
  }

  if (!mpc_traj_smoothed.size()) {
    ROS_DEBUG("[MPC] path callback: trajectory size is undesired.");
    return;
  }

  ref_traj_ = mpc_traj_smoothed;

  /* publish debug marker */
  {
    using MPCUtils::convertTrajToMarker;
    visualization_msgs::MarkerArray m;
    std::string frame = msg->header.frame_id;
    m = convertTrajToMarker(mpc_traj_raw, "trajectory raw", 0.9, 0.5, 0.0, 0.05, frame);
    pub_debug_marker_.publish(m);
    m = convertTrajToMarker(mpc_traj_resampled, "trajectory spline", 0.5, 0.1, 1.0, 0.05, frame);
    pub_debug_marker_.publish(m);
    m = convertTrajToMarker(mpc_traj_smoothed, "trajectory smoothed", 0.0, 1.0, 0.0, 0.05, frame);
    pub_debug_marker_.publish(m);
  }
}

void MPCFollower::updateCurrentPose()
{
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN_DELAYED_THROTTLE(
      5.0, "[mpc_follower] cannot get map to base_link transform. %s", ex.what());
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

void MPCFollower::onSteering(const autoware_vehicle_msgs::Steering::ConstPtr & msg)
{
  current_steer_ptr_ = msg;
}

void MPCFollower::onVelocity(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
  current_velocity_ptr_ = msg;
}

autoware_control_msgs::ControlCommand MPCFollower::getStopControlCommand() const
{
  autoware_control_msgs::ControlCommand cmd;
  cmd.steering_angle = steer_cmd_prev_;
  cmd.steering_angle_velocity = 0.0;
  cmd.velocity = 0.0;
  cmd.acceleration = -1.5;
  return cmd;
}

autoware_control_msgs::ControlCommand MPCFollower::getInitialControlCommand() const
{
  autoware_control_msgs::ControlCommand cmd;
  cmd.steering_angle = current_steer_ptr_->data;
  cmd.steering_angle_velocity = 0.0;
  cmd.velocity = 0.0;
  cmd.acceleration = -1.5;
  return cmd;
}

bool MPCFollower::isStoppedState() const
{
  const int nearest = MPCUtils::calcNearestIndex(*current_trajectory_ptr_, current_pose_ptr_->pose);
  // If the nearest index is not found, return false
  if (nearest < 0) return false;
  const double dist = calcStopDistance(nearest);
  if (dist < stop_state_keep_stopping_dist_) {
    ROS_DEBUG(
      "stop_dist = %f < %f : stop_state_keep_stopping_dist_. keep stopping.", dist,
      stop_state_keep_stopping_dist_);
    return true;
  }
  ROS_DEBUG("stop_dist = %f release stopping.", dist);

  const double current_vel = current_velocity_ptr_->twist.linear.x;
  const double target_vel = current_trajectory_ptr_->points.at(nearest).twist.linear.x;
  if (
    std::fabs(current_vel) < stop_state_entry_ego_speed_ &&
    std::fabs(target_vel) < stop_state_entry_target_speed_) {
    return true;
  } else {
    return false;
  }
}

double MPCFollower::calcStopDistance(const int origin) const
{
  constexpr double zero_velocity = std::numeric_limits<double>::epsilon();
  const double origin_velocity = current_trajectory_ptr_->points.at(origin).twist.linear.x;
  double stop_dist = 0.0;

  // search forward
  if (std::fabs(origin_velocity) > zero_velocity) {
    for (int i = origin + 1; i < static_cast<int>(current_trajectory_ptr_->points.size()) - 1;
         ++i) {
      const auto & p0 = current_trajectory_ptr_->points.at(i);
      const auto & p1 = current_trajectory_ptr_->points.at(i - 1);
      stop_dist += MPCUtils::calcDist2d(p0.pose, p1.pose);
      if (std::fabs(p0.twist.linear.x) < zero_velocity) {
        break;
      }
    }
    return stop_dist;
  }

  // search backward
  for (int i = origin - 1; 0 < i; --i) {
    const auto & p0 = current_trajectory_ptr_->points.at(i);
    const auto & p1 = current_trajectory_ptr_->points.at(i + 1);
    if (std::fabs(p0.twist.linear.x) > zero_velocity) {
      break;
    }
    stop_dist -= MPCUtils::calcDist2d(p0.pose, p1.pose);
  }
  return stop_dist;
}

void MPCFollower::publishCtrlCmd(const autoware_control_msgs::ControlCommand & ctrl_cmd)
{
  autoware_control_msgs::ControlCommandStamped cmd;
  cmd.header.frame_id = "base_link";
  cmd.header.stamp = ros::Time::now();
  cmd.control = ctrl_cmd;
  pub_ctrl_cmd_.publish(cmd);

  steer_cmd_prev_ = ctrl_cmd.steering_angle;
}

bool MPCFollower::isValidTrajectory(const autoware_planning_msgs::Trajectory & traj) const
{
  for (const auto & points : traj.points) {
    const auto & p = points.pose.position;
    const auto & o = points.pose.orientation;
    const auto & t = points.twist.linear;
    const auto & a = points.accel.linear;
    if (
      !isfinite(p.x) || !isfinite(p.y) || !isfinite(p.z) || !isfinite(o.x) || !isfinite(o.y) ||
      !isfinite(o.z) || !isfinite(o.w) || !isfinite(t.x) || !isfinite(t.y) || !isfinite(t.z) ||
      !isfinite(a.x) || !isfinite(a.y) || !isfinite(a.z)) {
      return false;
    }
  }
  return true;
}
