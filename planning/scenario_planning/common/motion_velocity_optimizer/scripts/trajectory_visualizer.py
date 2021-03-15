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
import time
from autoware_planning_msgs.msg import Trajectory, PathWithLaneId, Path
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, Twist
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np
import argparse
import message_filters

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

parser = argparse.ArgumentParser()
parser.add_argument('-l', '--length', help='max arclength in plot')
parser.add_argument(
    '-t', '--type', help='Options  VEL(default): show velocity only, VEL_ACC_JERK: show vel & acc & jerk')

args = parser.parse_args()

if args.length is None:
    PLOT_MAX_ARCLENGTH = 200
else:
    PLOT_MAX_ARCLENGTH = int(args.length)
print('max arclength = ' + str(PLOT_MAX_ARCLENGTH))

if args.type is None:
    PLOT_TYPE = "VEL"
elif args.type == "VEL":
    PLOT_TYPE = "VEL"
elif args.type == "VEL_ACC_JERK":
    PLOT_TYPE = "VEL_ACC_JERK"
else:
    print("invalid type. set default VEL.")
    PLOT_TYPE = "VEL"
print('plot type = ' + PLOT_TYPE)

PATH_ORIGIN_FRAME = "map"
SELF_POSE_FRAME = "base_link"


class TrajectoryVisualizer(Node):

    def __init__(self):

        super().__init__("trajectory_visualizer")

        self.fig = plt.figure()

        self.max_vel = 0.0
        self.min_vel = 0.0
        self.min_accel = 0.0
        self.max_accel = 0.0
        self.min_jerk = 0.0
        self.max_jerk = 0.0

        # update flag
        self.update_ex_vel_lim = False
        self.update_lat_acc_fil = False
        self.update_traj_raw = False
        self.update_traj_resample = False
        self.update_traj_final = False
        self.update_lanechange_path = False
        self.update_behavior_path = False
        self.update_traj_ob_avoid = False
        self.update_traj_ob_stop = False

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=True)

        self.self_pose = Pose()
        self.self_pose_received = False
        self.localization_twist = Twist()
        self.vehicle_twist = Twist()

        self.trajectory_external_velocity_limited = Trajectory()
        self.trajectory_lateral_acc_filtered = Trajectory()
        self.trajectory_raw = Trajectory()
        self.trajectory_time_resampled = Trajectory()
        self.trajectory_final = Trajectory()

        self.lane_change_path = PathWithLaneId()
        self.behavior_path = Path()
        self.obstacle_avoid_traj = Trajectory()
        self.obstacle_stop_traj = Trajectory()

        self.plotted = [False] * 9
        self.sub_localization_twist = self.create_subscription(
            TwistStamped, "/localization/twist", self.CallbackLocalizationTwist, 1)
        self.sub_vehicle_twist = self.create_subscription(
            TwistStamped, "/vehicle/status/twist", self.CallbackVehicleTwist, 1)

        # BUFFER_SIZE = 65536*100
        optimizer_debug = "/planning/scenario_planning/debug/"
        # TODO: change optimizer_debug to following after fixing bug of autoware
        # optimizer_debug = "/planning/scenario_planning/motion_velocity_optimizer/debug/"
        self.sub1 = message_filters.Subscriber(
            self, Trajectory, optimizer_debug + "trajectory_external_velocity_limited")
        self.sub2 = message_filters.Subscriber(
            self, Trajectory, optimizer_debug + "trajectory_lateral_acc_filtered")
        self.sub3 = message_filters.Subscriber(
            self, Trajectory, optimizer_debug + "trajectory_raw")
        self.sub4 = message_filters.Subscriber(
            self, Trajectory, optimizer_debug + "trajectory_time_resampled")
        self.sub5 = message_filters.Subscriber(
            self, Trajectory, "/planning/scenario_planning/trajectory")

        lane_driving = "/planning/scenario_planning/lane_driving"
        self.sub6 = message_filters.Subscriber(
            self, PathWithLaneId, lane_driving + "/behavior_planning/path_with_lane_id")
        self.sub7 = message_filters.Subscriber(
            self, Path, lane_driving + "/behavior_planning/path")
        self.sub8 = message_filters.Subscriber(
            self,
            Trajectory,
            lane_driving +
            "/motion_planning/obstacle_avoidance_planner/trajectory")
        self.sub9 = message_filters.Subscriber(
            self, Trajectory, lane_driving + "/trajectory")

        self.ts1 = message_filters.ApproximateTimeSynchronizer(
            [self.sub1, self.sub2, self.sub3, self.sub4, self.sub5], 30, 0.5)
        self.ts1.registerCallback(self.CallbackMotionVelOptTraj)
        self.ts2 = message_filters.ApproximateTimeSynchronizer(
            [self.sub6, self.sub7, self.sub8, self.sub9], 30, 1, 0)
        self.ts2.registerCallback(self.CallBackLaneDrivingTraj)

        # main process
        if PLOT_TYPE == "VEL_ACC_JERK":
            self.ani = animation.FuncAnimation(
                self.fig, self.plotTrajectory, interval=100, blit=True)
            self.setPlotTrajectory()
        else:
            self.ani = animation.FuncAnimation(
                self.fig, self.plotTrajectoryVelocity, interval=100, blit=True)
            self.setPlotTrajectoryVelocity()

        plt.show()

        return

    def test(self):
        self.updatePose("map", "base_link")

    def CallbackLocalizationTwist(self, cmd):
        self.localization_twist = cmd.twist

    def CallbackVehicleTwist(self, cmd):
        self.vehicle_twist = cmd.twist

    def CallbackMotionVelOptTraj(self, cmd1, cmd2, cmd3, cmd4, cmd5):
        self.CallBackTrajExVelLim(cmd1)
        self.CallBackTrajLatAccFiltered(cmd2)
        self.CallBackTrajRaw(cmd3)
        self.CallBackTrajTimeResampled(cmd4)
        self.CallBackTrajFinal(cmd5)

    def CallBackTrajExVelLim(self, cmd):
        self.trajectory_external_velocity_limited = cmd
        self.update_ex_vel_lim = True

    def CallBackTrajLatAccFiltered(self, cmd):
        self.trajectory_lateral_acc_filtered = cmd
        self.update_lat_acc_fil = True

    def CallBackTrajRaw(self, cmd):
        self.trajectory_raw = cmd
        self.update_traj_raw = True

    def CallBackTrajTimeResampled(self, cmd):
        self.trajectory_time_resampled = cmd
        self.update_traj_resample = True

    def CallBackTrajFinal(self, cmd):
        self.trajectory_final = cmd
        self.update_traj_final = True

    def CallBackLaneDrivingTraj(self, cmd6, cmd7, cmd8, cmd9):
        self.CallBackLaneChangePath(cmd6)
        self.CallBackBehaviorPath(cmd7)
        self.CallbackObstacleAvoidTraj(cmd8)
        self.CallbackObstacleStopTraj(cmd9)

    def CallBackLaneChangePath(self, cmd):
        self.lane_change_path = cmd
        self.update_lanechange_path = True

    def CallBackBehaviorPath(self, cmd):
        self.behavior_path = cmd
        self.update_behavior_path = True

    def CallbackObstacleAvoidTraj(self, cmd):
        self.obstacle_avoid_traj = cmd
        self.update_traj_ob_avoid = True

    def CallbackObstacleStopTraj(self, cmd):
        self.obstacle_stop_traj = cmd
        self.update_traj_ob_stop = True

    def setPlotTrajectoryVelocity(self):
        self.ax1 = plt.subplot(1, 1, 1)  # row, col, index(<raw*col)
        self.im1, = self.ax1.plot(
            [], [], label="0: lane_change_path", marker="")
        self.im2, = self.ax1.plot(
            [], [], label="1: behavior_path", marker="", ls="--")
        self.im3, = self.ax1.plot(
            [], [], label="2: obstacle_avoid_traj", marker="", ls="-.")
        self.im4, = self.ax1.plot(
            [], [], label="3: obstacle_stop_traj", marker="", ls="--")
        self.im5, = self.ax1.plot(
            [], [], label="4-1: opt input", marker="", ls="--")
        self.im6, = self.ax1.plot(
            [], [], label="4-2: opt external_velocity_limited", marker="", ls="--")
        self.im7, = self.ax1.plot(
            [], [], label="4-3: opt lat_acc_filtered", marker="", ls="--")
        self.im8, = self.ax1.plot(
            [], [], label="4-4: opt time_resampled", marker="", ls="--")
        self.im9, = self.ax1.plot(
            [], [], label="4-5: opt final", marker="", ls="-")
        self.im10, = self.ax1.plot(
            [], [], label="localization twist vx", color="r", marker="*", ls=":", markersize=10)
        self.im11, = self.ax1.plot(
            [], [], label="vehicle twist vx", color="k", marker="+", ls=":", markersize=10)
        self.ax1.set_title("trajectorys velocity")
        self.ax1.legend()
        self.ax1.set_xlim([0, PLOT_MAX_ARCLENGTH])
        self.ax1.set_ylabel("vel [m/s]")

        return self.im1, self.im2, self.im3, self.im4, self.im5, \
            self.im6, self.im7, self.im8, self.im9, self.im10, self.im11

    def plotTrajectoryVelocity(self, data):
        self.updatePose(PATH_ORIGIN_FRAME, SELF_POSE_FRAME)
        if self.self_pose_received is False:
            self.get_logger().info("plot start but self pose is not received")
            return self.im1, self.im2, self.im3, self.im4, self.im5, \
                self.im6, self.im7, self.im8, self.im9, self.im10, self.im11
        self.get_logger().info("plot start")

        # copy
        lane_change_path = self.lane_change_path
        behavior_path = self.behavior_path
        obstacle_avoid_traj = self.obstacle_avoid_traj
        obstacle_stop_traj = self.obstacle_stop_traj
        trajectory_raw = self.trajectory_raw
        trajectory_external_velocity_limited = self.trajectory_external_velocity_limited
        trajectory_lateral_acc_filtered = self.trajectory_lateral_acc_filtered
        trajectory_time_resampled = self.trajectory_time_resampled
        trajectory_final = self.trajectory_final
        # self.get_logger().info("received : behavior_path {}, obstacle_avoid_traj {}, obstacle_stop_traj {}, trajectory_raw {}, \
        #     trajectory_external_velocity_limited {}, trajectory_lateral_acc_filtered {}, trajectory_time_resampled {}, trajectory_final {}"\
        #     .format(self.update_lanechange_path, self.update_behavior_path, self.update_traj_ob_avoid, self.update_traj_ob_stop, \
        # self.update_traj_raw, self.update_ex_vel_lim,
        # self.update_lat_acc_fil, self.update_traj_resample,
        # self.update_traj_final))

        if self.update_lanechange_path:
            x = self.CalcArcLengthPathWLid(lane_change_path)
            y = self.ToVelListPathWLid(lane_change_path)
            self.im1.set_data(x, y)
            self.update_lanechange_path = False
            if len(y) != 0:
                self.max_vel = max(10.0, np.max(y))
                self.min_vel = np.min(y)

        if self.update_behavior_path:
            x = self.CalcArcLengthPath(behavior_path)
            y = self.ToVelListPath(behavior_path)
            self.im2.set_data(x, y)
            self.update_behavior_path = False

        if self.update_traj_ob_avoid:
            x = self.CalcArcLength(obstacle_avoid_traj)
            y = self.ToVelList(obstacle_avoid_traj)
            self.im3.set_data(x, y)
            self.update_traj_ob_avoid = False

        if self.update_traj_ob_stop:
            x = self.CalcArcLength(obstacle_stop_traj)
            y = self.ToVelList(obstacle_stop_traj)
            self.im4.set_data(x, y)
            self.update_traj_ob_stop = False

        if self.update_traj_raw:
            x = self.CalcArcLength(trajectory_raw)
            y = self.ToVelList(trajectory_raw)
            self.im5.set_data(x, y)
            self.update_traj_raw = False

        if self.update_ex_vel_lim:
            x = self.CalcArcLength(trajectory_external_velocity_limited)
            y = self.ToVelList(trajectory_external_velocity_limited)
            self.im6.set_data(x, y)
            self.update_ex_vel_lim = False

        if self.update_lat_acc_fil:
            x = self.CalcArcLength(trajectory_lateral_acc_filtered)
            y = self.ToVelList(trajectory_lateral_acc_filtered)
            self.im7.set_data(x, y)
            self.update_lat_acc_fil = False

        if self.update_traj_resample:
            x = self.CalcArcLength(trajectory_time_resampled)
            y = self.ToVelList(trajectory_time_resampled)
            self.im8.set_data(x, y)
            self.update_traj_resample = False

        if self.update_traj_final:
            x = self.CalcArcLength(trajectory_final)
            y = self.ToVelList(trajectory_final)
            self.im9.set_data(x, y)
            self.update_traj_final = False

            closest = self.calcClosestTrajectory(trajectory_final)
            if closest >= 0:
                x_closest = x[closest]
                self.im10.set_data(x_closest, self.localization_twist.linear.x)
                self.im11.set_data(x_closest, self.vehicle_twist.linear.x)

        # change y-range
        self.ax1.set_ylim([self.min_vel - 1.0, self.max_vel + 1.0])

        return self.im1, self.im2, self.im3, self.im4, self.im5, \
            self.im6, self.im7, self.im8, self.im9, self.im10, self.im11

    def CalcArcLength(self, traj):
        s_arr = []
        ds = 0.0
        s_sum = 0.0

        if len(traj.points) > 0:
            s_arr.append(s_sum)

        for i in range(1, len(traj.points)):
            p0 = traj.points[i - 1]
            p1 = traj.points[i]
            dx = p1.pose.position.x - p0.pose.position.x
            dy = p1.pose.position.y - p0.pose.position.y
            ds = np.sqrt(dx**2 + dy**2)
            s_sum += ds
            s_arr.append(s_sum)
        return s_arr

    def CalcArcLengthPathWLid(self, traj):
        s_arr = []
        ds = 0.0
        s_sum = 0.0

        if len(traj.points) > 0:
            s_arr.append(s_sum)

        for i in range(1, len(traj.points)):
            p0 = traj.points[i - 1].point
            p1 = traj.points[i].point
            dx = p1.pose.position.x - p0.pose.position.x
            dy = p1.pose.position.y - p0.pose.position.y
            ds = np.sqrt(dx**2 + dy**2)
            s_sum += ds
            s_arr.append(s_sum)
        return s_arr

    def CalcArcLengthPath(self, traj):
        s_arr = []
        ds = 0.0
        s_sum = 0.0

        if len(traj.points) > 0:
            s_arr.append(s_sum)

        for i in range(1, len(traj.points)):
            p0 = traj.points[i - 1]
            p1 = traj.points[i]
            dx = p1.pose.position.x - p0.pose.position.x
            dy = p1.pose.position.y - p0.pose.position.y
            ds = np.sqrt(dx**2 + dy**2)
            s_sum += ds
            s_arr.append(s_sum)
        return s_arr

    def ToVelList(self, traj):
        v_list = []
        for p in traj.points:
            v_list.append(p.twist.linear.x)
        return v_list

    def ToVelListPathWLid(self, traj):
        v_list = []
        for p in traj.points:
            v_list.append(p.point.twist.linear.x)
        return v_list

    def ToVelListPath(self, traj):
        v_list = []
        for p in traj.points:
            v_list.append(p.twist.linear.x)
        return v_list

    def CalcAcceleration(self, traj):
        a_arr = []
        for i in range(1, len(traj.points) - 1):
            p0 = traj.points[i - 1]
            p1 = traj.points[i]
            v0 = p0.twist.linear.x
            v1 = p1.twist.linear.x
            v = 0.5 * (v1 + v0)
            dx = p1.pose.position.x - p0.pose.position.x
            dy = p1.pose.position.y - p0.pose.position.y
            ds = np.sqrt(dx**2 + dy**2)
            dt = ds / max(abs(v), 0.001)
            a = (v1 - v0) / dt
            a_arr.append(a)
        if len(traj.points) > 0:
            a_arr.append(0)
            a_arr.append(0)
        return a_arr

    def CalcJerk(self, traj):
        j_arr = []
        for i in range(1, len(traj.points) - 2):
            p0 = traj.points[i - 1]
            p1 = traj.points[i]
            p2 = traj.points[i + 1]
            v0 = p0.twist.linear.x
            v1 = p1.twist.linear.x
            v2 = p2.twist.linear.x

            dx0 = p1.pose.position.x - p0.pose.position.x
            dy0 = p1.pose.position.y - p0.pose.position.y
            ds0 = np.sqrt(dx0**2 + dy0**2)

            dx1 = p2.pose.position.x - p1.pose.position.x
            dy1 = p2.pose.position.y - p1.pose.position.y
            ds1 = np.sqrt(dx1**2 + dy1**2)

            dt0 = ds0 / max(abs(0.5 * (v1 + v0)), 0.001)
            dt1 = ds1 / max(abs(0.5 * (v2 + v1)), 0.001)

            a0 = (v1 - v0) / max(dt0, 0.001)
            a1 = (v2 - v1) / max(dt1, 0.001)
            j = (a1 - a0) / max(dt1, 0.001)
            j_arr.append(j)
        if len(traj.points) > 0:
            j_arr.append(0)
            j_arr.append(0)
            j_arr.append(0)
        return j_arr

    def setPlotTrajectory(self):
        self.ax1 = plt.subplot(3, 1, 1)  # row, col, index(<raw*col)
        self.im0, = self.ax1.plot([], [], label="0: raw", marker="")
        self.im1, = self.ax1.plot([], [], label="3: time_resampled", marker="")
        self.im2, = self.ax1.plot([], [], label="4: final velocity", marker="")
        self.ax1.set_title("trajectorys velocity")
        self.ax1.legend()
        self.ax1.set_xlim([0, PLOT_MAX_ARCLENGTH])
        self.ax1.set_ylabel("vel [m/s]")

        self.ax2 = plt.subplot(3, 1, 2)
        self.ax2.set_xlim([0, PLOT_MAX_ARCLENGTH])
        self.ax2.set_ylim([-1, 1])
        self.ax2.set_ylabel("acc [m/ss]")
        self.im3, = self.ax2.plot([], [], label="final accel")

        self.ax3 = plt.subplot(3, 1, 3)
        self.ax3.set_xlim([0, PLOT_MAX_ARCLENGTH])
        self.ax3.set_ylim([-2, 2])
        self.ax3.set_xlabel("arclength [m]")
        self.ax3.set_ylabel("jerk [m/sss]")
        self.im4, = self.ax3.plot([], [], label="final jerk")

        return self.im0, self.im1, self.im2, self.im3, self.im4

    def plotTrajectory(self, data):
        self.get_logger().info('plot called')
        self.updatePose(PATH_ORIGIN_FRAME, SELF_POSE_FRAME)

        # copy
        trajectory_raw = self.trajectory_raw
        trajectory_external_velocity_limited = self.trajectory_external_velocity_limited
        trajectory_lateral_acc_filtered = self.trajectory_lateral_acc_filtered
        trajectory_time_resampled = self.trajectory_time_resampled
        trajectory_final = self.trajectory_final

        # ax1
        if self.update_traj_raw:
            x = self.CalcArcLength(trajectory_raw)
            y = self.ToVelList(trajectory_raw)
            self.im0.set_data(x, y)
            self.update_traj_raw = False
            if len(y) != 0:
                self.max_vel = max(10.0, np.max(y))
                self.min_vel = np.min(y)
                # change y-range
                self.ax1.set_ylim([self.min_vel - 1.0, self.max_vel + 1.0])

        if self.update_traj_resample:
            x = self.CalcArcLength(trajectory_time_resampled)
            y = self.ToVelList(trajectory_time_resampled)
            self.im1.set_data(x, y)
            self.update_traj_resample = False

        if self.update_traj_final:
            x = self.CalcArcLength(trajectory_final)
            y = self.ToVelList(trajectory_final)
            self.im2.set_data(x, y)
            self.update_traj_final = False

            # ax2
            y = self.CalcAcceleration(trajectory_final)
            if len(y) != 0:
                self.max_accel = max(0.0, np.max(y))
                self.min_accel = min(0.0, np.min(y))
                # change y-range
                self.ax2.set_ylim([self.min_accel - 1.0, self.max_accel + 1.0])
                if len(x) == len(y):
                    self.im3.set_data(x, y)

            # ax3
            y = self.CalcJerk(trajectory_final)
            if len(y) != 0:
                self.max_jerk = max(0.0, np.max(y))
                self.min_jerk = min(0.0, np.min(y))
                # change y-range
                #self.ax3.set_ylim([self.min_jerk - 1.0, self.max_jerk + 1.0])
                self.ax3.set_ylim([-2.0, 2.0])  # fixed range
                if len(x) == len(y):
                    self.im4.set_data(x, y)

        return self.im0, self.im1, self.im2, self.im3, self.im4

    def calcClosestPath(self, path):
        closest = -1
        min_dist_squared = 1.0e10
        for i in range(0, len(path.points)):
            dist_sq = self.calcSquaredDist2d(
                self.self_pose, path.points[i].pose)
            if dist_sq < min_dist_squared:
                min_dist_squared = dist_sq
                closest = i
        return closest

    def calcClosestPathWLid(self, path):
        closest = -1
        min_dist_squared = 1.0e10
        for i in range(0, len(path.points)):
            dist_sq = self.calcSquaredDist2d(
                self.self_pose, path.points[i].point.pose)
            if dist_sq < min_dist_squared:
                min_dist_squared = dist_sq
                closest = i
        return closest

    def calcClosestTrajectory(self, path):
        closest = -1
        min_dist_squared = 1.0e10
        for i in range(0, len(path.points)):
            dist_sq = self.calcSquaredDist2d(
                self.self_pose, path.points[i].pose)
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
            tf = self.tf_buffer.lookup_transform(
                from_link, to_link, rclpy.time.Time())
            self.self_pose.position.x = tf.transform.translation.x
            self.self_pose.position.y = tf.transform.translation.y
            self.self_pose.position.z = tf.transform.translation.z
            self.self_pose.orientation.x = tf.transform.rotation.x
            self.self_pose.orientation.y = tf.transform.rotation.y
            self.self_pose.orientation.z = tf.transform.rotation.z
            self.self_pose.orientation.w = tf.transform.rotation.w
            self.get_logger().info("updatePose succeeded")
            self.self_pose_received = True
            return
        except BaseException:
            self.get_logger().warn(
                'lookup transform failed: from {} to {}'.format(
                    from_link, to_link))
            return

    def closeFigure(self):
        plt.close(self.fig)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = TrajectoryVisualizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
