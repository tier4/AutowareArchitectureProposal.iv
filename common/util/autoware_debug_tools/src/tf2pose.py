#! /usr/bin/env python3
from __future__ import print_function

import argparse
import sys


import rclpy
from rclpy.node import Node
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped


class Tf2PoseNode(Node):
    def __init__(self, options):
        super().__init__("tf2pose")

        self._options = options
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._pub_pose = self.create_publisher(PoseStamped, "pose", 1)
        self.timer = self.create_timer((1.0 / self._options.hz), self._on_timer)

    def _on_timer(self):
        try:
            (trans, quat) = self.tf_buffer.lookup_transform(self._options.tf_from, self._options.tf_to, rclpy.time.Time())
            time = self._tf_listener.getLatestCommonTime(self._options.tf_from, self._options.tf_to)
            pose = Tf2PoseNode.create_pose(time, self._options.tf_from, trans, quat)
            self._pub_pose.publish(pose)
        except LookupException as e:
            print(e)

    @staticmethod
    def create_pose(time, frame_id, trans, quat):
        pose = PoseStamped()

        pose.header.stamp = time
        pose.header.frame_id = frame_id

        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        return pose


def main(args):
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument("tf_from", type=str)
    parser.add_argument("tf_to", type=str)
    parser.add_argument("hz", type=int, default=10)
    ns = parser.parse_args(args)

    tf2pose_node = Tf2PoseNode(ns)
    rclpy.spin(tf2pose_node)
    tf2pose_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
