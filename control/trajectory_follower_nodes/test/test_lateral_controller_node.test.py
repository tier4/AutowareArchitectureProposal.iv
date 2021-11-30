# Copyright 2021 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pytest
import launch_ros
import launch_testing
import launch
import os
from ament_index_python import get_package_share_directory
import unittest
import rclpy
from autoware_auto_planning_msgs.msg import Trajectory, TrajectoryPoint
from nav_msgs.msg import Odometry
from autoware_auto_vehicle_msgs.msg import SteeringReport
from autoware_auto_control_msgs.msg import AckermannLateralCommand
import tf2_ros
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from rclpy.clock import Clock, ClockType
import rclpy.time
import time
import copy


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="trajectory_follower_nodes",
                executable="lateral_controller_node_exe",
                parameters=[
                    os.path.join(
                    get_package_share_directory("trajectory_follower_nodes"),
                    "param",
                    "lateral_controller_defaults.yaml",
                    ),
                    os.path.join(
                    get_package_share_directory("trajectory_follower_nodes"),
                    "param",
                    "test_vehicle_info.yaml",
                    ),
                ],
                arguments=["--ros-args", "--log-level", "lateral_controller:=debug"]
            ),
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestLateralController(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.fake_node = rclpy.create_node("fake_node")
        self.traj_pub = self.fake_node.create_publisher(
            Trajectory,
            "lateral_controller/input/reference_trajectory",
            10,
        )
        self.odom_pub = self.fake_node.create_publisher(
            Odometry,
            "lateral_controller/input/current_odometry",
            10,
        )
        self.steer_pub = self.fake_node.create_publisher(
            SteeringReport,
            "lateral_controller/input/current_steering",
            10,
        )

    def tearDown(self) -> None:
        self.fake_node.destroy_node()

    @staticmethod
    def get_dummy_transform() -> TransformStamped:
        transform = TransformStamped()
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        return transform

    def wait_for_message(self, max_wait_time = 10.0, fail_on_timeout=True):
        end_time = time.time() + max_wait_time
        while not self.received_lateral_command:
            rclpy.spin_once(self.fake_node, timeout_sec=0.1)
            time.sleep(0.1)
            if time.time() > end_time:
                if fail_on_timeout:
                    assert "Did not receive a message soon enough"
                else:
                    break

    def test_no_input(self):
        cmd_msg = AckermannLateralCommand()
        self.received_lateral_command = False

        def callback(msg):
            nonlocal cmd_msg
            self.received_lateral_command = True
            cmd_msg = msg

        cmd_sub = self.fake_node.create_subscription(
            AckermannLateralCommand,
            "lateral_controller/output/control_cmd",
            lambda msg: callback(msg),
            10,
        )

        clock = Clock(clock_type=ClockType.ROS_TIME)

        br = tf2_ros.StaticTransformBroadcaster(self.fake_node)

        # Dummy transform: ego is at (0.0, 0.0) in map frame
        transform = self.get_dummy_transform()
        transform.header.stamp = clock.now().to_msg()
        br.sendTransform(transform)
        time.sleep(1.0)

        # Empty trajectory: expect a stopped command
        traj_msg = Trajectory()
        traj_msg.header.stamp = clock.now().to_msg()
        traj_msg.header.frame_id = "map"
        odom_msg = Odometry()
        odom_msg.header.stamp = clock.now().to_msg()
        odom_msg.twist.twist.linear.x = 0.0
        steer_msg = SteeringReport()
        steer_msg.stamp = clock.now().to_msg()
        steer_msg.steering_tire_angle = 0.0
        self.traj_pub.publish(traj_msg)
        self.odom_pub.publish(odom_msg)
        self.steer_pub.publish(steer_msg)
        print("publish")

        self.wait_for_message(1.0, False)
        self.assertFalse(self.received_lateral_command)

    def test_straight_trajectory(self):
        cmd_msg = AckermannLateralCommand()
        self.received_lateral_command = False

        def callback(msg):
            nonlocal cmd_msg
            self.received_lateral_command = True
            cmd_msg = msg
            print("received")

        cmd_sub = self.fake_node.create_subscription(
            AckermannLateralCommand,
            "lateral_controller/output/control_cmd",
            lambda msg: callback(msg),
            10,
        )

        clock = Clock(clock_type=ClockType.ROS_TIME)

        br = tf2_ros.StaticTransformBroadcaster(self.fake_node)

        # Dummy transform: ego is at (0.0, 0.0) in map frame
        transform = self.get_dummy_transform()
        transform.header.stamp = clock.now().to_msg()
        br.sendTransform(transform)
        time.sleep(1.0)

        # Empty trajectory: expect a stopped command
        traj_msg = Trajectory()
        traj_msg.header.stamp = clock.now().to_msg()
        traj_msg.header.frame_id = "map"
        p = TrajectoryPoint()
        p.pose.position.x = -1.0
        p.pose.position.y = 0.0
        p.longitudinal_velocity_mps = 1.0
        traj_msg.points.append(copy.deepcopy(p))
        p.pose.position.x = 0.0
        p.pose.position.y = 0.0
        p.longitudinal_velocity_mps = 1.0
        traj_msg.points.append(copy.deepcopy(p))
        p.pose.position.x = 1.0
        p.pose.position.y = 0.0
        p.longitudinal_velocity_mps = 1.0
        traj_msg.points.append(copy.deepcopy(p))
        p.pose.position.x = 2.0
        p.pose.position.y = 0.0
        p.longitudinal_velocity_mps = 1.0
        traj_msg.points.append(copy.deepcopy(p))
        self.traj_pub.publish(traj_msg)
        odom_msg = Odometry()
        odom_msg.header.stamp = clock.now().to_msg()
        odom_msg.twist.twist.linear.x = 1.0
        steer_msg = SteeringReport()
        steer_msg.stamp = clock.now().to_msg()
        steer_msg.steering_tire_angle = 0.0
        self.odom_pub.publish(odom_msg)
        self.steer_pub.publish(steer_msg)
        print("publish")

        self.wait_for_message()
        self.assertTrue(self.received_lateral_command)
        self.assertEqual(cmd_msg.steering_tire_angle, 0.0)
        self.assertEqual(cmd_msg.steering_tire_rotation_rate, 0.0)
        self.assertGreater(
            rclpy.time.Time().from_msg(cmd_msg.stamp),
            rclpy.time.Time().from_msg(traj_msg.header.stamp)
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        """
        Test process exit code.

        Check that all processes in the launch (in this case, there's just one) exit
        with code 0
        """
        launch_testing.asserts.assertExitCodes(proc_info)
