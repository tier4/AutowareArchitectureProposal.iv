#! /usr/bin/env python3

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
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped


class SelfPoseListener(Node):
    def __init__(self):
        super().__init__('self_pose_listener')
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, self)

    def get_current_pose(self):
        try:
            (trans, quat) = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time())
            time = self._tf_listener.getLatestCommonTime("map", "base_link")
            return SelfPoseListener.create_pose(time, "map", trans, quat)
        except LookupException:
            return None

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
