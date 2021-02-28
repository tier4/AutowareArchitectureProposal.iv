#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2020 Tier IV, Inc.
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

import rclpy
from rclpy.node import Node

import math
import sys
import time
import copy
import numpy as np

from autoware_planning_msgs.msg import Path, PathWithLaneId, Trajectory
from autoware_control_msgs.msg import ControlCommandStamped
from autoware_vehicle_msgs.msg import VehicleCommand
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist, TwistStamped
from std_msgs.msg import Bool, Float32, Header, Int32, Float32MultiArray

from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

REF_LINK = "map"
SELF_LINK = "base_link"


LANE_CHANGE = 0
BEHAVIOR_VELOCITY = 1
OBSTACLE_AVOID = 2
OBSTACLE_STOP = 3
LAT_ACC = 4
VELOCITY_OPTIMIZE = 5
CONTROL_CMD = 6
VEHICLE_CMD = 7
CONTROL_CMD_ACC = 8
VEHICLE_CMD_ACC = 9
DATA_NUM = 10


class VelocityChecker(Node):
    def __init__(self):
        super().__init__("velocity_checker")

        self.autoware_engage = None
        self.external_vlim = np.nan
        self.localization_twist = Twist()
        self.localization_twist.linear.x = np.nan
        self.vehicle_twist = Twist()
        self.vehicle_twist.linear.x = np.nan
        self.self_pose = Pose()
        self.data_arr = [np.nan] * DATA_NUM
        self.count = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # planning path and trajectories
        lane_drv = "/planning/scenario_planning/lane_driving"
        scenareio = "/planning/scenario_planning"
        self.sub0 = self.create_subscription(PathWithLaneId, lane_drv+"/behavior_planning/path_with_lane_id", self.CallBackBehaviorPathWLid, 1)
        self.sub1 = self.create_subscription(Path, lane_drv+"/behavior_planning/path", self.CallBackBehaviorPath, 1)
        self.sub2 = self.create_subscription(Trajectory, lane_drv+"/motion_planning/obstacle_avoidance_planner/trajectory", self.CallBackAvoidTrajectory, 1)
        self.sub3 = self.create_subscription(Trajectory, lane_drv+"/trajectory", self.CallBackLaneDriveTrajectory, 1)
        self.sub4 = self.create_subscription(Trajectory, scenareio+"/motion_velocity_optimizer/debug/trajectory_lateral_acc_filtered", self.CallBackLataccTrajectory, 1)
        self.sub5 = self.create_subscription(Trajectory, scenareio+"/trajectory", self.CallBackScenarioTrajectory, 1)

        # control commands
        self.sub6 = self.create_subscription(ControlCommandStamped, "/control/control_cmd", self.CallBackControlCmd, 1)
        self.sub7 = self.create_subscription(VehicleCommand, "/control/vehicle_cmd", self.CallBackVehicleCmd, 1)

        # others related to velocity
        self.sub8 = self.create_subscription(Bool, "/autoware/engage", self.CallBackAwEngage, 1)
        self.sub9 = self.create_subscription(Float32, "/planning/scenario_planning/max_velocity", self.CallBackExternalVelLim, 1)

        # self twist
        self.sub10 = self.create_subscription(TwistStamped, "/localization/twist", self.CallBackLocalizationTwist, 1)
        self.sub11 = self.create_subscription(TwistStamped, "/vehicle/status/twist", self.CallBackVehicleTwist, 1)

        # publish data
        self.pub_varr = self.create_publisher(Float32MultiArray, "closest_speeds", 1)

        time.sleep(1.0)  # wait for ready to publish/subscribe

        # for publish traffic signal image
        self.create_timer(0.1, self.timerCallback)


    def printInfo(self):
        self.count = self.count % 30
        if self.count == 0:
            self.get_logger().info("")
            self.get_logger().info("| Map Limit | Behavior | Obs Avoid | Obs Stop | External Lim | LatAcc Filtered | Optimized | Control VelCmd | Control AccCmd | Vehicle VelCmd | Vehicle AccCmd | Engage | Localization Vel | Vehicle Vel | [km/h]")
        mps2kmph = 3.6
        vel_map_lim = self.data_arr[LANE_CHANGE] * mps2kmph
        vel_behavior = self.data_arr[BEHAVIOR_VELOCITY] * mps2kmph
        vel_obs_avoid = self.data_arr[OBSTACLE_AVOID] * mps2kmph
        vel_obs_stop = self.data_arr[OBSTACLE_STOP] * mps2kmph
        vel_external_lim = self.external_vlim * mps2kmph
        vel_latacc_filtered = self.data_arr[LAT_ACC] * mps2kmph
        vel_optimized = self.data_arr[VELOCITY_OPTIMIZE] * mps2kmph
        vel_ctrl_cmd = self.data_arr[CONTROL_CMD] * mps2kmph
        acc_ctrl_cmd = self.data_arr[CONTROL_CMD_ACC]
        vel_vehicle_cmd = self.data_arr[VEHICLE_CMD] * mps2kmph
        acc_vehicle_cmd = self.data_arr[VEHICLE_CMD_ACC]
        vel_localization = self.localization_twist.linear.x * mps2kmph
        vel_vehicle = self.vehicle_twist.linear.x * mps2kmph
        engage = "None" if self.autoware_engage is None else ("True" if self.autoware_engage is True else "False")
        self.get_logger().info("| {0: 9.2f} | {1: 8.2f} | {2: 9.2f} | {3: 8.2f} | {4: 12.2f} | {5: 15.2f} | {6: 9.2f} | {7: 14.2f} | {8: 14.2f} | {9: 14.2f} | {10: 14.2f} | {11:>6s} | {12: 16.2f} | {13: 11.2f} |".format(
            vel_map_lim, vel_behavior, vel_obs_avoid, vel_obs_stop, vel_external_lim, vel_latacc_filtered, vel_optimized,
            vel_ctrl_cmd, acc_ctrl_cmd, vel_vehicle_cmd, acc_vehicle_cmd,  engage, vel_localization, vel_vehicle))
        self.count += 1

    def timerCallback(self):
        # self.get_logger().info("timer called")
        self.updatePose(REF_LINK, SELF_LINK)
        self.pub_varr.publish(Float32MultiArray(data=self.data_arr))
        self.printInfo()

    def CallBackAwEngage(self, msg):
        self.autoware_engage = msg.data

    def CallBackExternalVelLim(self, msg):
        self.external_vlim = msg.data

    def CallBackLocalizationTwist(self, msg):
        self.localization_twist = msg.twist

    def CallBackVehicleTwist(self, msg):
        self.vehicle_twist = msg.twist

    def CallBackBehaviorPathWLid(self, msg):
        # self.get_logger().info("LANE_CHANGE called")
        closest = self.calcClosestPathWLid(msg)
        self.data_arr[LANE_CHANGE] = msg.points[closest].point.twist.linear.x
        return

    def CallBackBehaviorPath(self, msg):
        # self.get_logger().info("BEHAVIOR_VELOCITY called")
        closest = self.calcClosestPath(msg)
        self.data_arr[BEHAVIOR_VELOCITY] = msg.points[closest].twist.linear.x
        return

    def CallBackAvoidTrajectory(self, msg):
        # self.get_logger().info("OBSTACLE_AVOID called")
        closest = self.calcClosestTrajectory(msg)
        self.data_arr[OBSTACLE_AVOID] = msg.points[closest].twist.linear.x
        return

    def CallBackLaneDriveTrajectory(self, msg):
        # self.get_logger().info("OBSTACLE_STOP called")
        closest = self.calcClosestTrajectory(msg)
        self.data_arr[OBSTACLE_STOP] = msg.points[closest].twist.linear.x
        return

    def CallBackLataccTrajectory(self, msg):
        # self.get_logger().info("LAT_ACC called")
        closest = self.calcClosestTrajectory(msg)
        self.data_arr[LAT_ACC] = msg.points[closest].twist.linear.x
        return

    def CallBackScenarioTrajectory(self, msg):
        # self.get_logger().info("VELOCITY_OPTIMIZE called")
        closest = self.calcClosestTrajectory(msg)
        self.data_arr[VELOCITY_OPTIMIZE] = msg.points[closest].twist.linear.x
        return

    def CallBackControlCmd(self, msg):
        # self.get_logger().info("CONTROL_CMD called")
        self.data_arr[CONTROL_CMD] = msg.control.velocity
        self.data_arr[CONTROL_CMD_ACC] = msg.control.acceleration
        return

    def CallBackVehicleCmd(self, msg):
        # self.get_logger().info("VEHICLE_CMD called")
        self.data_arr[VEHICLE_CMD] = msg.control.velocity
        self.data_arr[VEHICLE_CMD_ACC] = msg.control.acceleration
        return

    def calcClosestPath(self, path):
        closest = -1
        min_dist_squared = 1.0e10
        for i in range(0, len(path.points)):
            dist_sq = self.calcSquaredDist2d(self.self_pose, path.points[i].pose)
            if dist_sq < min_dist_squared:
                min_dist_squared = dist_sq
                closest = i
        return closest

    def calcClosestPathWLid(self, path):
        closest = -1
        min_dist_squared = 1.0e10
        for i in range(0, len(path.points)):
            dist_sq = self.calcSquaredDist2d(self.self_pose, path.points[i].point.pose)
            if dist_sq < min_dist_squared:
                min_dist_squared = dist_sq
                closest = i
        return closest

    def calcClosestTrajectory(self, path):
        closest = -1
        min_dist_squared = 1.0e10
        for i in range(0, len(path.points)):
            dist_sq = self.calcSquaredDist2d(self.self_pose, path.points[i].pose)
            if dist_sq < min_dist_squared:
                min_dist_squared = dist_sq
                closest = i
        return closest

    def calcSquaredDist2d(self, p1, p2):
        dx = p1.position.x - p2.position.x
        dy = p1.position.y - p2.position.y
        return dx * dx + dy * dy

    def updatePose(self, from_link, to_link):
        try:
            tf = self.tf_buffer.lookup_transform(from_link, to_link, rclpy.time.Time())
            self.self_pose.position.x = tf.transform.translation.x
            self.self_pose.position.y = tf.transform.translation.y
            self.self_pose.position.z = tf.transform.translation.z
            self.self_pose.orientation.x = tf.transform.rotation.x
            self.self_pose.orientation.y = tf.transform.rotation.y
            self.self_pose.orientation.z = tf.transform.rotation.z
            self.self_pose.orientation.w = tf.transform.rotation.w
            return
        except LookupException as e:
            self.get_logger().warn('No required transformation found: `{}`'.format(str(e)))
            return


def main(args=None):
    try:
        rclpy.init(args=args)
        node = VelocityChecker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
