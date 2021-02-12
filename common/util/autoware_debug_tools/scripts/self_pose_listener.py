import rclpy
from rclpy.node import Node
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped


class SelfPoseListener():
    def __init__(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_current_pose(self):
        try:
            (trans, quat) = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time()) #todo rospy.Time(0)
            time = self._tf_listener.getLatestCommonTime("map", "base_link")
            return SelfPoseListener.create_pose(time, "map", trans, quat)
        except Exception as e:
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
