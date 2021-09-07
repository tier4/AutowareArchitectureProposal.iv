// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include "gtest/gtest.h"

#include "simple_planning_simulator/simple_planning_simulator_core.hpp"
#include "motion_common/motion_common.hpp"

using autoware_auto_msgs::msg::VehicleControlCommand;
using autoware_auto_msgs::msg::VehicleStateCommand;
using autoware_auto_msgs::msg::VehicleKinematicState;
using geometry_msgs::msg::PoseWithCovarianceStamped;

using simulation::simple_planning_simulator::SimplePlanningSimulator;

std::string toStrInfo(const VehicleKinematicState & state)
{
  const auto & s = state.state;
  std::stringstream ss;
  ss << "state x: " << s.x << ", y: " << s.y << ", yaw: " << motion::motion_common::to_angle(
    s.heading) << ", vx = " << s.longitudinal_velocity_mps << ", vy: " << s.lateral_velocity_mps <<
    ", ax: " << s.acceleration_mps2 << ", steer: " << s.front_wheel_angle_rad;
  return ss.str();
}

static constexpr float32_t COM_TO_BASELINK = 1.5f;
class PubSubNode : public rclcpp::Node
{
public:
  PubSubNode()
  : Node{"test_simple_planning_simulator_pubsub"}
  {
    kinematic_state_sub_ = create_subscription<VehicleKinematicState>(
      "output/kinematic_state", rclcpp::QoS{1},
      [this](const VehicleKinematicState::SharedPtr msg) {
        current_state_ = msg;
      });
    pub_control_command_ = create_publisher<VehicleControlCommand>(
      "input/vehicle_control_command",
      rclcpp::QoS{1});
    pub_initialpose_ = create_publisher<PoseWithCovarianceStamped>(
      "/initialpose",
      rclcpp::QoS{1});
    pub_state_cmd_ = create_publisher<VehicleStateCommand>(
      "/input/vehicle_state_command",
      rclcpp::QoS{1});
  }

  rclcpp::Publisher<VehicleControlCommand>::SharedPtr pub_control_command_;
  rclcpp::Publisher<VehicleStateCommand>::SharedPtr pub_state_cmd_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_initialpose_;
  rclcpp::Subscription<VehicleKinematicState>::SharedPtr kinematic_state_sub_;

  VehicleKinematicState::SharedPtr current_state_;
};

VehicleControlCommand cmdGen(
  const builtin_interfaces::msg::Time & t, float32_t steer, float32_t vel, float32_t acc)
{
  VehicleControlCommand cmd;
  cmd.stamp = t;
  cmd.front_wheel_angle_rad = steer;
  cmd.velocity_mps = vel;
  cmd.long_accel_mps2 = acc;
  return cmd;
}

void resetInitialpose(rclcpp::Node::SharedPtr sim_node, std::shared_ptr<PubSubNode> pub_sub_node)
{
  PoseWithCovarianceStamped p;
  p.header.frame_id = "odom";
  p.header.stamp = sim_node->now();
  p.pose.pose.orientation.w = 1.0;  // yaw = 0
  for (int i = 0; i < 10; ++i) {
    pub_sub_node->pub_initialpose_->publish(p);
    rclcpp::spin_some(sim_node);
    rclcpp::spin_some(pub_sub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds{10LL});
  }
}

void sendGear(
  uint8_t gear, rclcpp::Node::SharedPtr sim_node,
  std::shared_ptr<PubSubNode> pub_sub_node)
{
  VehicleStateCommand cmd;
  cmd.stamp = sim_node->now();
  cmd.gear = gear;
  for (int i = 0; i < 10; ++i) {
    pub_sub_node->pub_state_cmd_->publish(cmd);
    rclcpp::spin_some(sim_node);
    rclcpp::spin_some(pub_sub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds{10LL});
  }
}

void sendCommand(
  const VehicleControlCommand & cmd, rclcpp::Node::SharedPtr sim_node,
  std::shared_ptr<PubSubNode> pub_sub_node)
{
  for (int i = 0; i < 150; ++i) {
    pub_sub_node->pub_control_command_->publish(cmd);
    rclcpp::spin_some(sim_node);
    rclcpp::spin_some(pub_sub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds{10LL});
  }
}

VehicleKinematicState comToBaselink(const VehicleKinematicState & com)
{
  auto baselink = com;
  float64_t yaw = motion::motion_common::to_angle(com.state.heading);
  baselink.state.x -= COM_TO_BASELINK * static_cast<float>(std::cos(yaw));
  baselink.state.y -= COM_TO_BASELINK * static_cast<float>(std::sin(yaw));
  return baselink;
}


// Check which direction the vehicle is heading on the baselink coordinates.
//                      y
//                      |
//                      |         (Fwd-Left)
//                      |
//  ---------(Bwd)------------------(Fwd)----------> x
//                      |
//        (Bwd-Right)   |
//                      |
//
void isOnForward(const VehicleKinematicState & _state, const VehicleKinematicState & _init)
{
  float32_t forward_thr = 1.0f;
  auto state = comToBaselink(_state);
  auto init = comToBaselink(_init);
  float32_t dx = state.state.x - init.state.x;
  EXPECT_GT(dx, forward_thr) << "[curr] " << toStrInfo(state) << ", [init] " << toStrInfo(init);
}

void isOnBackward(const VehicleKinematicState & _state, const VehicleKinematicState & _init)
{
  float32_t backward_thr = -1.0f;
  auto state = comToBaselink(_state);
  auto init = comToBaselink(_init);
  float32_t dx = state.state.x - init.state.x;
  EXPECT_LT(dx, backward_thr) << "[curr] " << toStrInfo(state) << ", [init] " << toStrInfo(init);
}

void isOnForwardLeft(const VehicleKinematicState & _state, const VehicleKinematicState & _init)
{
  float32_t forward_thr = 1.0f;
  float32_t left_thr = 0.1f;
  auto state = comToBaselink(_state);
  auto init = comToBaselink(_init);
  float32_t dx = state.state.x - init.state.x;
  float32_t dy = state.state.y - init.state.y;
  EXPECT_GT(dx, forward_thr) << "[curr] " << toStrInfo(state) << ", [init] " << toStrInfo(init);
  EXPECT_GT(dy, left_thr) << "[curr] " << toStrInfo(state) << ", [init] " << toStrInfo(init);
}

void isOnBackwardRight(const VehicleKinematicState & _state, const VehicleKinematicState & _init)
{
  float32_t backward_thr = -1.0f;
  float32_t right_thr = -0.1f;
  auto state = comToBaselink(_state);
  auto init = comToBaselink(_init);
  float32_t dx = state.state.x - init.state.x;
  float32_t dy = state.state.y - init.state.y;
  EXPECT_LT(dx, backward_thr) << "[curr] " << toStrInfo(state) << ", [init] " << toStrInfo(init);
  EXPECT_LT(dy, right_thr) << "[curr] " << toStrInfo(state) << ", [init] " << toStrInfo(init);
}

// Send a control command and run the simulation.
// Then check if the vehicle is moving in the desired direction.
TEST(TestSimplePlanningSimulatorIdealSteerVel, TestMoving)
{
  rclcpp::init(0, nullptr);

  std::vector<std::string> vehicle_model_type_vec = { // NOLINT
    "IDEAL_STEER_VEL",
    "IDEAL_STEER_ACC",
    "IDEAL_STEER_ACC_GEARED",
    "DELAY_STEER_ACC",
    "DELAY_STEER_ACC_GEARED",
  };

  for (const auto & vehicle_model_type : vehicle_model_type_vec) {
    std::cout << "\n\n vehicle model = " << vehicle_model_type << std::endl << std::endl;
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("initialize_source", "INITIAL_POSE_TOPIC");
    node_options.append_parameter_override("cg_to_rear_m", COM_TO_BASELINK);
    node_options.append_parameter_override("vehicle_model_type", vehicle_model_type);
    const auto sim_node = std::make_shared<SimplePlanningSimulator>(node_options);

    const auto pub_sub_node = std::make_shared<PubSubNode>();

    const float32_t target_vel = 5.0f;
    const float32_t target_acc = 5.0f;
    const float32_t target_steer = 0.2f;

    auto _resetInitialpose = [&]() {resetInitialpose(sim_node, pub_sub_node);};
    auto _sendFwdGear = [&]() {sendGear(VehicleStateCommand::GEAR_DRIVE, sim_node, pub_sub_node);};
    auto _sendBwdGear =
      [&]() {sendGear(VehicleStateCommand::GEAR_REVERSE, sim_node, pub_sub_node);};
    auto _sendCommand = [&](const VehicleControlCommand & _cmd) {
        sendCommand(_cmd, sim_node, pub_sub_node);
      };

    // check pub-sub connections
    {
      size_t expected = 1;
      EXPECT_EQ(pub_sub_node->pub_control_command_->get_subscription_count(), expected);
      EXPECT_EQ(pub_sub_node->pub_state_cmd_->get_subscription_count(), expected);
      EXPECT_EQ(pub_sub_node->pub_initialpose_->get_subscription_count(), expected);
      EXPECT_EQ(pub_sub_node->kinematic_state_sub_->get_publisher_count(), expected);
    }

    // check initial pose
    _resetInitialpose();
    const auto init_state = *(pub_sub_node->current_state_);


    // go forward
    _resetInitialpose();
    _sendFwdGear();
    _sendCommand(cmdGen(pub_sub_node->now(), 0.0f, target_vel, target_acc));
    isOnForward(*(pub_sub_node->current_state_), init_state);

    // go backward
    _resetInitialpose();
    _sendBwdGear();
    _sendCommand(cmdGen(pub_sub_node->now(), 0.0f, -target_vel, -target_acc));
    isOnBackward(*(pub_sub_node->current_state_), init_state);

    // go forward left
    _resetInitialpose();
    _sendFwdGear();
    _sendCommand(cmdGen(pub_sub_node->now(), target_steer, target_vel, target_acc));
    isOnForwardLeft(*(pub_sub_node->current_state_), init_state);

    // go backward right
    _resetInitialpose();
    _sendBwdGear();
    _sendCommand(cmdGen(pub_sub_node->now(), -target_steer, -target_vel, -target_acc));
    isOnBackwardRight(*(pub_sub_node->current_state_), init_state);
  }

  rclcpp::shutdown();
}
