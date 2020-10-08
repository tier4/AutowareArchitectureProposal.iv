#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import time
from autoware_planning_msgs.msg import Trajectory, PathWithLaneId, Path
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, Twist
import matplotlib.pyplot as plt
import numpy as np
import tf
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-l', '--length', help='max arclength in plot')
parser.add_argument('-t', '--type', help='Options  VEL(default): show velocity only, VEL_ACC_JERK: show vel & acc & jerk')

args = parser.parse_args()

if args.length == None:
    PLOT_MAX_ARCLENGTH = 200
else:
    PLOT_MAX_ARCLENGTH = int(args.length)
print('max arclength = ' + str(PLOT_MAX_ARCLENGTH))


if args.type == None:
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

class TrajectoryVisualizer():

    def __init__(self):
        self.tfl = tf.TransformListener()  # for get self-position
        self.self_pose = Pose()
        self.localization_twist = Twist()
        self.vehicle_twist = Twist()

        self.trajectory_external_velocity_limitted = Trajectory()
        self.trajectory_lateral_acc_filtered = Trajectory()
        self.trajectory_raw = Trajectory()
        self.trajectory_time_resamped = Trajectory()
        self.trajectory_final = Trajectory()

        self.lane_change_path = PathWithLaneId()
        self.behavior_path = Path()
        self.obstacle_avoid_traj = Trajectory()
        self.obstacle_stop_traj = Trajectory()

        self.plotted = [False] * 9
        self.sub_localization_twist = rospy.Subscriber("/localization/twist", TwistStamped, self.CallbackLocalizationTwist, queue_size=1, tcp_nodelay=True)
        self.sub_vehicle_twist = rospy.Subscriber("/vehicle/status/twist", TwistStamped, self.CallbackVehicleTwist, queue_size=1, tcp_nodelay=True)

        self.substatus1 = rospy.Subscriber("/planning/scenario_planning/motion_velocity_optimizer/debug/trajectory_external_velocity_limitted", Trajectory, self.CallBackTrajExVelLim, queue_size=1, tcp_nodelay=True)
        self.substatus2 = rospy.Subscriber("/planning/scenario_planning/motion_velocity_optimizer/debug/trajectory_lateral_acc_filtered", Trajectory, self.CallBackTrajLatAccFiltered, queue_size=1, tcp_nodelay=True)
        self.substatus3 = rospy.Subscriber("/planning/scenario_planning/motion_velocity_optimizer/debug/trajectory_raw", Trajectory, self.CallBackTrajRaw, queue_size=1, tcp_nodelay=True)
        self.substatus4 = rospy.Subscriber("/planning/scenario_planning/motion_velocity_optimizer/debug/trajectory_time_resampled", Trajectory, self.CallBackTrajTimeResampled, queue_size=1, tcp_nodelay=True)
        self.substatus5 = rospy.Subscriber("/planning/scenario_planning/trajectory", Trajectory, self.CallBackTrajFinal, queue_size=1, tcp_nodelay=True)

        self.substatus6 = rospy.Subscriber("/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", PathWithLaneId, self.CallBackLaneChangePath, queue_size=1, tcp_nodelay=True)
        self.substatus7 = rospy.Subscriber("/planning/scenario_planning/lane_driving/behavior_planning/path", Path, self.CallBackBehaviorPath, queue_size=1, tcp_nodelay=True)
        self.substatus8 = rospy.Subscriber("/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/trajectory", Trajectory, self.CallbackObstacleAvoidTraj, queue_size=1, tcp_nodelay=True)
        self.substatus8 = rospy.Subscriber("/planning/scenario_planning/lane_driving/trajectory", Trajectory, self.CallbackObstacleStopTraj, queue_size=1, tcp_nodelay=True)

        rospy.Timer(rospy.Duration(0.2), self.timerCallback)

    def CallbackLocalizationTwist(self, cmd):
        self.localization_twist = cmd.twist

    def CallbackVehicleTwist(self, cmd):
        self.vehicle_twist = cmd.twist

    def CallBackTrajExVelLim(self, cmd):
        self.trajectory_external_velocity_limitted = cmd

    def CallBackTrajLatAccFiltered(self, cmd):
        self.trajectory_lateral_acc_filtered = cmd

    def CallBackTrajRaw(self, cmd):
        self.trajectory_raw = cmd

    def CallBackTrajTimeResampled(self, cmd):
        self.trajectory_time_resamped = cmd

    def CallBackTrajFinal(self, cmd):
        self.trajectory_final = cmd

    def CallBackLaneChangePath(self, cmd):
        self.lane_change_path = cmd

    def CallBackBehaviorPath(self, cmd):
        self.behavior_path = cmd

    def CallbackObstacleAvoidTraj(self, cmd):
        self.obstacle_avoid_traj = cmd

    def CallbackObstacleStopTraj(self, cmd):
        self.obstacle_stop_traj = cmd

    def timerCallback(self, event):
        self.updatePose(PATH_ORIGIN_FRAME, SELF_POSE_FRAME)
        if PLOT_TYPE == "VEL_ACC_JERK":
            self.plotTrajectory()
        else:
            self.plotTrajectoryVelocity()

    def plotTrajectoryVelocity(self):
        plt.clf()
        ax1 = plt.subplot(1,1,1)  # row, col, index(<raw*col)

        rospy.loginfo("plot called")

        # copy
        lane_change_path = self.lane_change_path
        behavior_path = self.behavior_path
        obstacle_avoid_traj = self.obstacle_avoid_traj
        obstacle_stop_traj = self.obstacle_stop_traj
        trajectory_raw = self.trajectory_raw
        trajectory_external_velocity_limitted = self.trajectory_external_velocity_limitted
        trajectory_lateral_acc_filtered = self.trajectory_lateral_acc_filtered
        trajectory_time_resamped = self.trajectory_time_resamped
        trajectory_final = self.trajectory_final


        x = self.CalcArcLengthPathWLid(lane_change_path)
        y = self.ToVelListPathWLid(lane_change_path)
        ax1.plot(x, y, label="0: lane_change_path", marker="")

        x = self.CalcArcLengthPath(behavior_path)
        y = self.ToVelListPath(behavior_path)
        ax1.plot(x, y, label="1: behavior_path", marker="", ls="--")

        x = self.CalcArcLength(obstacle_avoid_traj)
        y = self.ToVelList(obstacle_avoid_traj)
        ax1.plot(x, y, label="2: obstacle_avoid_traj", marker="", ls="-.")

        x = self.CalcArcLength(obstacle_stop_traj)
        y = self.ToVelList(obstacle_stop_traj)
        ax1.plot(x, y, label="3: obstacle_stop_traj", marker="", ls="--")

        x = self.CalcArcLength(trajectory_raw)
        y = self.ToVelList(trajectory_raw)
        ax1.plot(x, y, label="4-1: opt input", marker="", ls="--")

        x = self.CalcArcLength(trajectory_external_velocity_limitted)
        y = self.ToVelList(trajectory_external_velocity_limitted)
        ax1.plot(x, y, label="4-2: opt external_velocity_limitted", marker="", ls="--")

        x = self.CalcArcLength(trajectory_lateral_acc_filtered)
        y = self.ToVelList(trajectory_lateral_acc_filtered)
        ax1.plot(x, y, label="4-3: opt lat_acc_filtered", marker="*", ls="--")

        x = self.CalcArcLength(trajectory_time_resamped)
        y = self.ToVelList(trajectory_time_resamped)
        ax1.plot(x, y, label="4-4: opt time_resamped", marker="*", ls="--")

        x = self.CalcArcLength(trajectory_final)
        y = self.ToVelList(trajectory_final)
        ax1.plot(x, y, label="4-5: opt final", marker="*", ls="--")

        closest = self.calcClosestTrajectory(trajectory_final)
        if closest >= 0:
            x_closest = x[closest]
            ax1.plot(x_closest, self.localization_twist.linear.x, label="localization twist vx", color="r", marker="*", ls=":", markersize=10)
            ax1.plot(x_closest, self.vehicle_twist.linear.x, label="vehicle twist vx", color="k", marker="+", ls=":", markersize=10)



        ax1.set_title("trajectorys velocity")
        ax1.legend()
        ax1.set_xlim([0, PLOT_MAX_ARCLENGTH])
        ax1.set_ylabel("vel [m/s]")

        plt.pause(0.01)


    def CalcArcLength(self, traj):
        s_arr = []
        ds = 0.0
        s_sum = 0.0

        if len(traj.points) > 0:
            s_arr.append(s_sum)

        for i in range(1, len(traj.points)):
            p0 = traj.points[i-1]
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
            p0 = traj.points[i-1].point
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
            p0 = traj.points[i-1]
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
            p0 = traj.points[i-1]
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
            p0 = traj.points[i-1]
            p1 = traj.points[i]
            p2 = traj.points[i+1]
            v0 = p0.twist.linear.x
            v1 = p1.twist.linear.x
            v2 = p2.twist.linear.x

            dx0 = p1.pose.position.x - p0.pose.position.x
            dy0 = p1.pose.position.y - p0.pose.position.y
            ds0 = np.sqrt(dx0**2 + dy0**2)

            dx1 = p2.pose.position.x - p1.pose.position.x
            dy1 = p2.pose.position.y - p1.pose.position.y
            ds1 = np.sqrt(dx1**2 + dy1**2)

            dt0 = ds0 / max(abs(0.5*(v1+v0)), 0.001)
            dt1 = ds1 / max(abs(0.5*(v2+v1)), 0.001)

            a0 = (v1 - v0) / max(dt0, 0.001)
            a1 = (v2 - v1) / max(dt1, 0.001)
            j = (a1 - a0) / max(dt1, 0.001)
            j_arr.append(j)
        if len(traj.points) > 0:
            j_arr.append(0)
            j_arr.append(0)
            j_arr.append(0)
        return j_arr

    def plotTrajectory(self):

        # copy
        trajectory_raw = self.trajectory_raw
        trajectory_external_velocity_limitted = self.trajectory_external_velocity_limitted
        trajectory_lateral_acc_filtered = self.trajectory_lateral_acc_filtered
        trajectory_time_resamped = self.trajectory_time_resamped
        trajectory_final = self.trajectory_final


        plt.clf()
        ax1 = plt.subplot(3,1,1)#row, col, index(<raw*col)
        x = self.CalcArcLength(trajectory_raw)
        y = self.ToVelList(trajectory_raw)
        ax1.plot(x, y, label="0: raw", marker="")

        x = self.CalcArcLength(trajectory_external_velocity_limitted)
        y = self.ToVelList(trajectory_external_velocity_limitted)
        ax1.plot(x, y, label="1: external_velocity_limitted", marker="")

        x = self.CalcArcLength(trajectory_lateral_acc_filtered)
        y = self.ToVelList(trajectory_lateral_acc_filtered)
        ax1.plot(x, y, label="2: lateral_acc_filtered", marker="*")

        x = self.CalcArcLength(trajectory_time_resamped)
        y = self.ToVelList(trajectory_time_resamped)
        ax1.plot(x, y, label="3: time_resamped", marker="*")

        x = self.CalcArcLength(trajectory_final)
        y = self.ToVelList(trajectory_final)
        ax1.plot(x, y, label="4: final", marker="*")
        ax1.set_title("trajectorys velocity")
        ax1.legend()
        ax1.set_xlim([0, PLOT_MAX_ARCLENGTH])
        ax1.set_ylabel("vel [m/s]")

        ax2 = plt.subplot(3,1,2)
        x = self.CalcArcLength(trajectory_final)
        y = self.CalcAcceleration(trajectory_final)
        if len(x) == len(y):
            ax2.plot(x, y, label="final")
            ax2.set_xlim([0, PLOT_MAX_ARCLENGTH])
            ax2.set_ylim([-5, 5])
        ax2.set_ylabel("acc [m/ss]")


        # ax2.legend()

        ax3 = plt.subplot(3,1,3)
        x = self.CalcArcLength(trajectory_final)
        y = self.CalcJerk(trajectory_final)
        if len(x) == len(y):
            ax3.plot(x, y, label="final")
            ax3.set_xlim([0, PLOT_MAX_ARCLENGTH])
            ax3.set_ylim([-2, 2])
        ax3.set_xlabel("arclength [m]")
        ax3.set_ylabel("jerk [m/sss]")

        # ax3.legend()

        #plt.show()
        plt.pause(.01)


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
            self.tfl.waitForTransform(from_link, to_link, rospy.Time(0), rospy.Duration(0.2))
            (trans, quat) = self.tfl.lookupTransform(from_link, to_link, rospy.Time(0))  # parent, child
            self.self_pose.position.x = trans[0]
            self.self_pose.position.y = trans[1]
            self.self_pose.position.z = trans[2]
            self.self_pose.orientation.x = quat[0]
            self.self_pose.orientation.y = quat[1]
            self.self_pose.orientation.z = quat[2]
            self.self_pose.orientation.w = quat[3]
            return
        except:
            trans = (0.0, 0.0, 0.0)
            quat = (0.0, 0.0, 0.0, 1.0)
            rospy.logwarn("cannot get position")
            return

def main():
    rospy.init_node("trajectory_visualizer")
    TrajectoryVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
