// Copyright 2021 The Autoware Foundation
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

#include <algorithm>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "trajectory_follower_nodes/lateral_controller_node.hpp"

#include "tf2_ros/create_timer_ros.h"


#define UPDATE_MPC_PARAM(PARAM_STRUCT, NAME) \
  update_param(parameters, "m_mpc" #NAME, PARAM_STRUCT.NAME)

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower_nodes
{
namespace
{
using namespace std::chrono_literals;

template<typename T>
void update_param(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & name, T & value)
{
  auto it = std::find_if(
    parameters.cbegin(), parameters.cend(),
    [&name](const rclcpp::Parameter & parameter) {return parameter.get_name() == name;});
  if (it != parameters.cend()) {
    value = static_cast<T>(it->template get_value<T>());
  }
}
}  // namespace

LateralController::LateralController(const rclcpp::NodeOptions & node_options)
: Node("lateral_controller", node_options),
  m_tf_buffer(this->get_clock()), m_tf_listener(m_tf_buffer)
{
  using std::placeholders::_1;

  m_mpc.m_ctrl_period = declare_parameter("ctrl_period").get<float64_t>();
  m_enable_path_smoothing = declare_parameter("enable_path_smoothing").get<bool8_t>();
  m_enable_yaw_recalculation = declare_parameter("enable_yaw_recalculation").get<bool8_t>();
  m_path_filter_moving_ave_num = declare_parameter("path_filter_moving_ave_num").get<int64_t>();
  m_curvature_smoothing_num = declare_parameter("curvature_smoothing_num").get<int64_t>();
  m_traj_resample_dist = declare_parameter("traj_resample_dist").get<float64_t>();
  m_mpc.m_admissible_position_error =
    declare_parameter("admissible_position_error").get<float64_t>();
  m_mpc.m_admissible_yaw_error_rad = declare_parameter("admissible_yaw_error_rad").get<float64_t>();
  m_mpc.m_use_steer_prediction = declare_parameter("use_steer_prediction").get<bool8_t>();
  m_mpc.m_param.steer_tau = declare_parameter("vehicle_model_steer_tau").get<float64_t>();

  /* stop state parameters */
  m_stop_state_entry_ego_speed = declare_parameter("stop_state_entry_ego_speed").get<float64_t>();
  m_stop_state_entry_target_speed =
    declare_parameter("stop_state_entry_target_speed").get<float64_t>();
  m_stop_state_keep_stopping_dist =
    declare_parameter("stop_state_keep_stopping_dist").get<float64_t>();

  /* mpc parameters */
  const float64_t steer_lim_deg = declare_parameter("steer_lim_deg").get<float64_t>();
  const float64_t steer_rate_lim_degs = declare_parameter("steer_rate_lim_dps").get<float64_t>();
  constexpr float64_t deg2rad = static_cast<float64_t>(autoware::common::types::PI) / 180.0;
  m_mpc.m_steer_lim = steer_lim_deg * deg2rad;
  m_mpc.m_steer_rate_lim = steer_rate_lim_degs * deg2rad;
  const float64_t cg_to_front_m = declare_parameter("vehicle.cg_to_front_m").get<float64_t>();
  const float64_t cg_to_rear_m = declare_parameter("vehicle.cg_to_rear_m").get<float64_t>();
  const float64_t wheelbase = cg_to_front_m + cg_to_rear_m;

  /* vehicle model setup */
  const std::string vehicle_model_type = declare_parameter("vehicle_model_type").get<std::string>();
  std::shared_ptr<trajectory_follower::VehicleModelInterface> vehicle_model_ptr;
  if (vehicle_model_type == "kinematics") {
    vehicle_model_ptr =
      std::make_shared<trajectory_follower::KinematicsBicycleModel>(
      wheelbase, m_mpc.m_steer_lim,
      m_mpc.m_param.steer_tau);
  } else if (vehicle_model_type == "kinematics_no_delay") {
    vehicle_model_ptr = std::make_shared<trajectory_follower::KinematicsBicycleModelNoDelay>(
      wheelbase, m_mpc.m_steer_lim);
  } else if (vehicle_model_type == "dynamics") {
    const float64_t mass_fl = declare_parameter("vehicle.mass_fl").get<float64_t>();
    const float64_t mass_fr = declare_parameter("vehicle.mass_fr").get<float64_t>();
    const float64_t mass_rl = declare_parameter("vehicle.mass_rl").get<float64_t>();
    const float64_t mass_rr = declare_parameter("vehicle.mass_rr").get<float64_t>();
    const float64_t cf = declare_parameter("vehicle.cf").get<float64_t>();
    const float64_t cr = declare_parameter("vehicle.cr").get<float64_t>();

    // vehicle_model_ptr is only assigned in ctor, so parameter value have to be passed at init time  // NOLINT
    vehicle_model_ptr = std::make_shared<trajectory_follower::DynamicsBicycleModel>(
      wheelbase, mass_fl, mass_fr, mass_rl, mass_rr, cf, cr);
  } else {
    RCLCPP_ERROR(get_logger(), "vehicle_model_type is undefined");
  }

  /* QP solver setup */
  const std::string qp_solver_type = declare_parameter("qp_solver_type").get<std::string>();
  std::shared_ptr<trajectory_follower::QPSolverInterface> qpsolver_ptr;
  if (qp_solver_type == "unconstraint_fast") {
    qpsolver_ptr = std::make_shared<trajectory_follower::QPSolverEigenLeastSquareLLT>();
  } else if (qp_solver_type == "osqp") {
    qpsolver_ptr = std::make_shared<trajectory_follower::QPSolverOSQP>(get_logger());
  } else {
    RCLCPP_ERROR(get_logger(), "qp_solver_type is undefined");
  }

  /* delay compensation */
  {
    const float64_t delay_tmp = declare_parameter("input_delay").get<float64_t>();
    const float64_t delay_step = std::round(delay_tmp / m_mpc.m_ctrl_period);
    m_mpc.m_param.input_delay = delay_step * m_mpc.m_ctrl_period;
    m_mpc.m_input_buffer = std::deque<float64_t>(static_cast<size_t>(delay_step), 0.0);
  }

  /* initialize lowpass filter */
  {
    const float64_t steering_lpf_cutoff_hz =
      declare_parameter("steering_lpf_cutoff_hz").get<float64_t>();
    const float64_t error_deriv_lpf_cutoff_hz =
      declare_parameter("error_deriv_lpf_cutoff_hz").get<float64_t>();
    m_mpc.initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);
  }

  /* set up ros system */
  initTimer(m_mpc.m_ctrl_period);

  m_pub_ctrl_cmd =
    create_publisher<autoware_auto_msgs::msg::AckermannLateralCommand>(
    "output/lateral/control_cmd", 1);
  m_pub_predicted_traj =
    create_publisher<autoware_auto_msgs::msg::Trajectory>("output/lateral/predicted_trajectory", 1);
  m_pub_diagnostic =
    create_publisher<autoware_auto_msgs::msg::Float32MultiArrayDiagnostic>(
    "output/lateral/diagnostic", 1);
  m_sub_ref_path = create_subscription<autoware_auto_msgs::msg::Trajectory>(
    "input/reference_trajectory", rclcpp::QoS{1},
    std::bind(&LateralController::onTrajectory, this, _1));
  m_sub_steering = create_subscription<autoware_auto_msgs::msg::VehicleKinematicState>(
    "input/current_kinematic_state", rclcpp::QoS{1}, std::bind(
      &LateralController::onState, this, _1));

  // TODO(Frederik.Beaujean) ctor is too long, should factor out parameter declarations
  declareMPCparameters();

  /* get parameter updates */
  m_set_param_res =
    this->add_on_set_parameters_callback(std::bind(&LateralController::paramCallback, this, _1));

  m_mpc.setQPSolver(qpsolver_ptr);
  m_mpc.setVehicleModel(vehicle_model_ptr, vehicle_model_type);

  m_mpc.setLogger(get_logger());
  m_mpc.setClock(get_clock());
}

LateralController::~LateralController()
{
  autoware_auto_msgs::msg::AckermannLateralCommand stop_cmd = getStopControlCommand();
  publishCtrlCmd(stop_cmd);
}

void LateralController::onTimer()
{
  updateCurrentPose();

  if (!checkData()) {
    publishCtrlCmd(getStopControlCommand());
    return;
  }

  autoware_auto_msgs::msg::AckermannLateralCommand ctrl_cmd;
  autoware_auto_msgs::msg::Trajectory predicted_traj;
  autoware_auto_msgs::msg::Float32MultiArrayDiagnostic diagnostic;

  if (!m_is_ctrl_cmd_prev_initialized) {
    m_ctrl_cmd_prev = getInitialControlCommand();
    m_is_ctrl_cmd_prev_initialized = true;
  }

  const bool8_t is_mpc_solved = m_mpc.calculateMPC(
    *m_current_state_ptr, m_current_state_ptr->state.longitudinal_velocity_mps,
    m_current_pose_ptr->pose, ctrl_cmd, predicted_traj, diagnostic);

  if (isStoppedState()) {
    // Reset input buffer
    for (auto & value : m_mpc.m_input_buffer) {
      value = m_ctrl_cmd_prev.steering_tire_angle;
    }
    // Use previous command value as previous raw steer command
    m_mpc.m_raw_steer_cmd_prev = m_ctrl_cmd_prev.steering_tire_angle;

    publishCtrlCmd(m_ctrl_cmd_prev);
    publishDiagnostic(diagnostic);
    return;
  }

  if (!is_mpc_solved) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/,
      "MPC is not solved. publish 0 velocity.");
    ctrl_cmd = getStopControlCommand();
  }

  m_ctrl_cmd_prev = ctrl_cmd;
  publishCtrlCmd(ctrl_cmd);
  publishPredictedTraj(predicted_traj);
  publishDiagnostic(diagnostic);
}

bool8_t LateralController::checkData() const
{
  if (!m_mpc.hasVehicleModel()) {
    RCLCPP_DEBUG(
      get_logger(), "MPC does not have a vehicle model");
    return false;
  }
  if (!m_mpc.hasQPSolver()) {
    RCLCPP_DEBUG(
      get_logger(), "MPC does not have a QP solver");
    return false;
  }

  if (!m_current_pose_ptr || !m_current_state_ptr) {
    RCLCPP_DEBUG(
      get_logger(), "waiting data. pose = %d, current_state = %d",
      m_current_pose_ptr != nullptr, m_current_state_ptr != nullptr);
    return false;
  }

  if (m_mpc.m_ref_traj.size() == 0) {
    RCLCPP_DEBUG(get_logger(), "trajectory size is zero.");
    return false;
  }

  return true;
}

void LateralController::onTrajectory(const autoware_auto_msgs::msg::Trajectory::SharedPtr msg)
{
  m_current_trajectory_ptr = msg;

  if (msg->points.size() < 3) {
    RCLCPP_DEBUG(get_logger(), "received path size is < 3, not enough.");
    return;
  }

  if (!isValidTrajectory(*msg)) {
    RCLCPP_ERROR(get_logger(), "Trajectory is invalid!! stop computing.");
    return;
  }

  m_mpc.setReferenceTrajectory(
    *msg, m_traj_resample_dist, m_enable_path_smoothing, m_path_filter_moving_ave_num,
    m_enable_yaw_recalculation, m_curvature_smoothing_num);
}

void LateralController::updateCurrentPose()
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = m_tf_buffer.lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/,
      "cannot get map to base_link transform. %s", ex.what());
    return;
  }

  geometry_msgs::msg::PoseStamped ps;
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  m_current_pose_ptr = std::make_shared<geometry_msgs::msg::PoseStamped>(ps);
}

void LateralController::onState(const autoware_auto_msgs::msg::VehicleKinematicState::SharedPtr msg)
{
  m_current_state_ptr = msg;
}

autoware_auto_msgs::msg::AckermannLateralCommand LateralController::getStopControlCommand() const
{
  autoware_auto_msgs::msg::AckermannLateralCommand cmd;
  cmd.steering_tire_angle = static_cast<decltype(cmd.steering_tire_angle)>(m_steer_cmd_prev);
  cmd.steering_tire_rotation_rate = 0.0;
  return cmd;
}

autoware_auto_msgs::msg::AckermannLateralCommand LateralController::getInitialControlCommand() const
{
  autoware_auto_msgs::msg::AckermannLateralCommand cmd;
  cmd.steering_tire_angle = m_current_state_ptr->state.front_wheel_angle_rad;
  cmd.steering_tire_rotation_rate = 0.0;
  return cmd;
}

bool8_t LateralController::isStoppedState() const
{
  const int64_t nearest = trajectory_follower::MPCUtils::calcNearestIndex(
    *m_current_trajectory_ptr,
    m_current_pose_ptr->pose);
  // If the nearest index is not found, return false
  if (nearest < 0) {return false;}
  const float64_t dist = trajectory_follower::MPCUtils::calcStopDistance(
    *m_current_trajectory_ptr,
    nearest);
  if (dist < m_stop_state_keep_stopping_dist) {
    RCLCPP_DEBUG(
      get_logger(),
      "stop_dist = %f < %f : m_stop_state_keep_stopping_dist. keep stopping.", dist,
      m_stop_state_keep_stopping_dist);
    return true;
  }
  RCLCPP_DEBUG(get_logger(), "stop_dist = %f release stopping.", dist);

  const float64_t current_vel = m_current_state_ptr->state.longitudinal_velocity_mps;
  const float64_t target_vel =
    m_current_trajectory_ptr->points.at(static_cast<size_t>(nearest)).longitudinal_velocity_mps;
  if (
    std::fabs(current_vel) < m_stop_state_entry_ego_speed &&
    std::fabs(target_vel) < m_stop_state_entry_target_speed)
  {
    return true;
  } else {
    return false;
  }
}

void LateralController::publishCtrlCmd(autoware_auto_msgs::msg::AckermannLateralCommand ctrl_cmd)
{
  ctrl_cmd.stamp = this->now();
  m_pub_ctrl_cmd->publish(ctrl_cmd);
  m_steer_cmd_prev = ctrl_cmd.steering_tire_angle;
}

void LateralController::publishPredictedTraj(autoware_auto_msgs::msg::Trajectory & predicted_traj)
const
{
  predicted_traj.header.stamp = this->now();
  predicted_traj.header.frame_id = m_current_trajectory_ptr->header.frame_id;
  m_pub_predicted_traj->publish(predicted_traj);
}

void LateralController::publishDiagnostic(
  autoware_auto_msgs::msg::Float32MultiArrayDiagnostic & diagnostic)
const
{
  diagnostic.diag_header.data_stamp = this->now();
  diagnostic.diag_header.name = std::string("linear-MPC lateral controller");
  m_pub_diagnostic->publish(diagnostic);
}

void LateralController::initTimer(float64_t period_s)
{
  auto timer_callback = std::bind(&LateralController::onTimer, this);
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<float64_t>(
      period_s));
  m_timer = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period_ns, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(m_timer, nullptr);
}

void LateralController::declareMPCparameters()
{
  m_mpc.m_param.prediction_horizon = declare_parameter("mpc_prediction_horizon").get<int64_t>();
  m_mpc.m_param.prediction_dt = declare_parameter("mpc_prediction_dt").get<float64_t>();
  m_mpc.m_param.weight_lat_error = declare_parameter("mpc_weight_lat_error").get<float64_t>();
  m_mpc.m_param.weight_heading_error =
    declare_parameter("mpc_weight_heading_error").get<float64_t>();
  m_mpc.m_param.weight_heading_error_squared_vel = declare_parameter(
    "mpc_weight_heading_error_squared_vel").get<float64_t>();
  m_mpc.m_param.weight_steering_input =
    declare_parameter("mpc_weight_steering_input").get<float64_t>();
  m_mpc.m_param.weight_steering_input_squared_vel = declare_parameter(
    "mpc_weight_steering_input_squared_vel").get<float64_t>();
  m_mpc.m_param.weight_lat_jerk = declare_parameter("mpc_weight_lat_jerk").get<float64_t>();
  m_mpc.m_param.weight_steer_rate = declare_parameter("mpc_weight_steer_rate").get<float64_t>();
  m_mpc.m_param.weight_steer_acc = declare_parameter("mpc_weight_steer_acc").get<float64_t>();
  m_mpc.m_param.low_curvature_weight_lat_error = declare_parameter(
    "mpc_low_curvature_weight_lat_error").get<float64_t>();
  m_mpc.m_param.low_curvature_weight_heading_error = declare_parameter(
    "mpc_low_curvature_weight_heading_error").get<float64_t>();
  m_mpc.m_param.low_curvature_weight_heading_error_squared_vel = declare_parameter(
    "mpc_low_curvature_weight_heading_error_squared_vel").get<float64_t>();
  m_mpc.m_param.low_curvature_weight_steering_input = declare_parameter(
    "mpc_low_curvature_weight_steering_input").get<float64_t>();
  m_mpc.m_param.low_curvature_weight_steering_input_squared_vel = declare_parameter(
    "mpc_low_curvature_weight_steering_input_squared_vel").get<float64_t>();
  m_mpc.m_param.low_curvature_weight_lat_jerk = declare_parameter(
    "mpc_low_curvature_weight_lat_jerk").get<float64_t>();
  m_mpc.m_param.low_curvature_weight_steer_rate = declare_parameter(
    "mpc_low_curvature_weight_steer_rate").get<float64_t>();
  m_mpc.m_param.low_curvature_weight_steer_acc = declare_parameter(
    "mpc_low_curvature_weight_steer_acc").get<float64_t>();
  m_mpc.m_param.low_curvature_thresh_curvature = declare_parameter(
    "mpc_low_curvature_thresh_curvature").get<float64_t>();
  m_mpc.m_param.weight_terminal_lat_error =
    declare_parameter("mpc_weight_terminal_lat_error").get<float64_t>();
  m_mpc.m_param.weight_terminal_heading_error = declare_parameter(
    "mpc_weight_terminal_heading_error").get<float64_t>();
  m_mpc.m_param.zero_ff_steer_deg = declare_parameter("mpc_zero_ff_steer_deg").get<float64_t>();
  m_mpc.m_param.acceleration_limit = declare_parameter("mpc_acceleration_limit").get<float64_t>();
  m_mpc.m_param.velocity_time_constant =
    declare_parameter("mpc_velocity_time_constant").get<float64_t>();
}

rcl_interfaces::msg::SetParametersResult LateralController::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // strong exception safety wrt MPCParam
  trajectory_follower::MPCParam param = m_mpc.m_param;
  try {
    UPDATE_MPC_PARAM(param, prediction_horizon);
    UPDATE_MPC_PARAM(param, prediction_dt);
    UPDATE_MPC_PARAM(param, weight_lat_error);
    UPDATE_MPC_PARAM(param, weight_heading_error);
    UPDATE_MPC_PARAM(param, weight_heading_error_squared_vel);
    UPDATE_MPC_PARAM(param, weight_steering_input);
    UPDATE_MPC_PARAM(param, weight_steering_input_squared_vel);
    UPDATE_MPC_PARAM(param, weight_lat_jerk);
    UPDATE_MPC_PARAM(param, weight_steer_rate);
    UPDATE_MPC_PARAM(param, weight_steer_acc);
    UPDATE_MPC_PARAM(param, low_curvature_weight_lat_error);
    UPDATE_MPC_PARAM(param, low_curvature_weight_heading_error);
    UPDATE_MPC_PARAM(param, low_curvature_weight_heading_error_squared_vel);
    UPDATE_MPC_PARAM(param, low_curvature_weight_steering_input);
    UPDATE_MPC_PARAM(param, low_curvature_weight_steering_input_squared_vel);
    UPDATE_MPC_PARAM(param, low_curvature_weight_lat_jerk);
    UPDATE_MPC_PARAM(param, low_curvature_weight_steer_rate);
    UPDATE_MPC_PARAM(param, low_curvature_weight_steer_acc);
    UPDATE_MPC_PARAM(param, low_curvature_thresh_curvature);
    UPDATE_MPC_PARAM(param, weight_terminal_lat_error);
    UPDATE_MPC_PARAM(param, weight_terminal_heading_error);
    UPDATE_MPC_PARAM(param, zero_ff_steer_deg);
    UPDATE_MPC_PARAM(param, acceleration_limit);
    UPDATE_MPC_PARAM(param, velocity_time_constant);

    // initialize input buffer
    update_param(parameters, "input_delay", param.input_delay);
    const float64_t delay_step = std::round(param.input_delay / m_mpc.m_ctrl_period);
    const float64_t delay = delay_step * m_mpc.m_ctrl_period;
    if (param.input_delay != delay) {
      param.input_delay = delay;
      m_mpc.m_input_buffer = std::deque<float64_t>(static_cast<size_t>(delay_step), 0.0);
    }

    // transaction succeeds, now assign values
    m_mpc.m_param = param;
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

bool8_t LateralController::isValidTrajectory(const autoware_auto_msgs::msg::Trajectory & traj) const
{
  for (const auto & p : traj.points) {
    if (
      !isfinite(p.x) || !isfinite(p.y) || !isfinite(p.heading.imag) || !isfinite(p.heading.real) ||
      !isfinite(p.longitudinal_velocity_mps) ||
      !isfinite(p.lateral_velocity_mps) || !isfinite(p.lateral_velocity_mps) ||
      !isfinite(p.heading_rate_rps) || !isfinite(p.front_wheel_angle_rad) ||
      !isfinite(p.rear_wheel_angle_rad))
    {
      return false;
    }
  }
  return true;
}

}  // namespace trajectory_follower_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::motion::control::trajectory_follower_nodes::LateralController)
