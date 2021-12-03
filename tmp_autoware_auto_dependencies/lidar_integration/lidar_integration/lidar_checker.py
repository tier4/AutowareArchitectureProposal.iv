# Copyright 2019 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

import launch_ros.actions


def make_lidar_checker(*,
                       input_type,
                       topic,
                       size,
                       period,
                       period_tolerance=0.7,
                       size_tolerance=0.1,
                       runtime=10):
    """
    Make a lidar_integration_listener to check the specified topic.

    :param input_type: One of "cloud", "box", "track", or "block", specifying the listened topic
    type
    :param size: The expected size of the point cloud messages
    :param period: The expected period of the incoming messages
    :param period_tolerance: A percentage expressed as a number from 0 to 1 that controls
    how far off the period can be from the expected value.  Larger numbers = wider tolerance
    :param size_tolerance: A percentage expressed as a number from 0 to 1 that controls
    how far off the size can be from the expected value.  Larger numbers = wider tolerance
    :param runtime: The amount of time to run the checker for
    """
    checker = launch_ros.actions.Node(
        package="lidar_integration",
        executable="lidar_integration_listener_exe",
        namespace="lidar_front",
        arguments=[
            "--type", "{}".format(input_type),
            "--topic", "{}".format(topic),
            "--size", "{}".format(size),
            "--period", "{}".format(period),
            "--period_tolerance", "{}".format(period_tolerance),
            "--size_tolerance", "{}".format(size_tolerance),
            "--runtime", "{}".format(runtime),
        ]
    )

    # Glue a little more information on the checker for when we print diagnostic messages
    setattr(checker, "_checker_id", topic)

    return checker


def make_pcl_checker(*,
                     topic,
                     size,
                     period,
                     period_tolerance=0.7,
                     size_tolerance=0.1,
                     runtime=10):
    """
    Make a lidar_integration_listener to check the specified topic.

    This checker expects the topic to be of type sensor_msgs/PointCloud

    :param size: The expected size of the point cloud messages
    :param period: The expected period of the incoming messages
    :param period_tolerance: A percentage expressed as a number from 0 to 1 that controls
    how far off the period can be from the expected value.  Larger numbers = wider tolerance
    :param size_tolerance: A percentage expressed as a number from 0 to 1 that controls
    how far off the size can be from the expected value.  Larger numbers = wider tolerance
    :param runtime: The amount of time to run the checker for
    """
    return make_lidar_checker(input_type="cloud", topic=topic, size=size, period=period,
                              period_tolerance=period_tolerance, size_tolerance=size_tolerance,
                              runtime=runtime)


def make_box_checker(*,
                     topic,
                     size,
                     period,
                     period_tolerance=0.7,
                     size_tolerance=0.1,
                     runtime=10):
    """
    Make a lidar_integration_listener to check the specified topic.

    This checker expects the topic to be of type BoundingBoxArray

    :param size: The expected size of the bounding boxes
    :param period: The expected period of the incoming messages
    :param period_tolerance: A percentage expressed as a number from 0 to 1 that controls
    how far off the period can be from the expected value.  Larger numbers = wider tolerance
    :param size_tolerance: A percentage expressed as a number from 0 to 1 that controls
    how far off the size can be from the expected value.  Larger numbers = wider tolerance
    :param runtime: The amount of time to run the checker for
    """
    return make_lidar_checker(input_type="box", topic=topic, size=size, period=period,
                              period_tolerance=period_tolerance, size_tolerance=size_tolerance,
                              runtime=runtime)
