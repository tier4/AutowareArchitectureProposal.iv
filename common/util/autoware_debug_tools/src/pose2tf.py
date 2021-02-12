#! /usr/bin/env python3
from __future__ import print_function

import argparse
import sys
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped


class Pose2TfNode(Node):
    def __init__(self, options):
        super().__init__('pose2tf')
        self._options = options
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._sub_pose = self.create_subscription(PoseStamped, self._options.topic_name, self._on_pose, 1)

    def _on_pose(self, msg):
        try:
            (trans, quat) = Pose2TfNode.create_transform(msg.pose)
            self._tf_broadcaster.sendTransform(
                trans, quat, msg.header.stamp, self._options.tf_name, msg.header.frame_id
            )
        except Exception as e:
            print(e)

    @staticmethod
    def create_transform(pose):
        trans = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        return (trans, quat)


def main(args):
    print("{}".format(args))
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument("topic_name", type=str)
    parser.add_argument("tf_name", type=str)
    ns = parser.parse_args(args)

    pose2tf_node = Pose2TfNode(ns)
    rclpy.spin(pose2tf_node)
    pose2tf_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv[1:])