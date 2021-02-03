#include "goal_distance_calculator/goal_distance_calculator_node.hpp"
#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include "autoware_utils/unit_conversion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

namespace goal_distance_calculator
{
GoalDistanceCalculatorNode::GoalDistanceCalculatorNode() : Node("goal_distance_calculator")
{
  using std::placeholders::_1;

  static constexpr std::size_t queue_size = 1;
  rclcpp::QoS durable_qos(queue_size);
  durable_qos.transient_local();

  // Node Parameter
  node_param_.update_rate = declare_parameter("update_rate", 10.0);
  node_param_.oneshot = declare_parameter("oneshot", true);

  // Core
  goal_distance_calculator_ = std::make_unique<GoalDistanceCalculator>();
  goal_distance_calculator_->setParam(param_);

  // Subscriber
  sub_route_ = create_subscription<autoware_planning_msgs::msg::Route>(
    "/planning/mission_planning/route", queue_size,
    [&](const autoware_planning_msgs::msg::Route::SharedPtr msg_ptr) { route_ = msg_ptr; });

  // Wait for first self pose
  //self_pose_listener_.waitForFirstPose();

  // Timer
  double delta_time = 1.0 / (double)node_param_.update_rate;
  auto timer_callback_ = std::bind(&GoalDistanceCalculatorNode::onTimer, this);
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(delta_time));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback_)>>(
    this->get_clock(), period_ns, std::move(timer_callback_),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
  goal_distance_calculator_ = std::make_unique<GoalDistanceCalculator>();
}

bool GoalDistanceCalculatorNode::isDataReady()
{
  if (!current_pose_) {
    auto & clk = *this->get_clock();
    RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 5000, "waiting for current_pose...");
    return false;
  }

  if (!route_) {
    auto & clk = *this->get_clock();
    RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 5000, "waiting for route msg...");
    return false;
  }

  return true;
}

bool GoalDistanceCalculatorNode::isDataTimeout()
{
  rclcpp::Clock system_clock(rcl_clock_type_t RCL_SYSTEM_TIME);
  //rclcpp::Time now = now();
  constexpr double th_pose_timeout = 1.0;
  //const auto pose_time_diff = current_pose_->header.stamp - now();
  if (true /*pose_time_diff.toSec() > th_pose_timeout*/) {
    auto & clk = *this->get_clock();
    RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "pose is timeout...");
    return true;
  }
  return false;
}

void GoalDistanceCalculatorNode::onTimer()
{
  //current_pose_ = self_pose_listener_.getCurrentPose();

  if (!isDataReady()) {
    return;
  }

  if (isDataTimeout()) {
    return;
  }

  input_.current_pose = current_pose_;
  input_.route = route_;

  output_ = goal_distance_calculator_->update(input_);

  {
    using autoware_utils::rad2deg;
    //const auto & deviation = output_.goal_deviation;
  }

  if (node_param_.oneshot) {
    rclcpp::shutdown();
  }
}
}  // namespace goal_distance_calculator
