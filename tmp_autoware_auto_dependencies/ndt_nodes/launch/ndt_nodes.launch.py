# Copyright 2020 the Autoware Foundation
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

"""Launch P2D NDT localizer and map publisher nodes."""

import os

from ament_index_python import get_package_share_directory
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    """Launch P2D NDT and map publisher nodes."""
    # P2D NDT localizer parameter file definition.
    p2d_ndt_localizer_file_path = os.path.join(
        get_package_share_directory('ndt_nodes'),
        'param',
        'p2d_ndt_node.default.param.yaml')
    p2d_ndt_localizer_param_file = launch.substitutions.LaunchConfiguration(
        'params', default=[p2d_ndt_localizer_file_path])

    # P2D NDT localizer node execution definition.
    p2d_ndt_localizer_runner = launch_ros.actions.Node(
        package='ndt_nodes',
        executable='p2d_ndt_localizer_exe',
        parameters=[p2d_ndt_localizer_param_file],
        remappings=[("points_in", "points_nonground")])

    return launch.LaunchDescription([
        p2d_ndt_localizer_runner])
