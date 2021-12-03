# Copyright 2018 the Autoware Foundation
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

import os
import socket


def get_open_port_any():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("", 0))
    s.listen(1)
    port = s.getsockname()[1]
    s.close()
    return port


def get_open_port(range_start=3000):
    def bad_port(p):
        return (p < 1024) or (p > 7400)
    if (bad_port(range_start)):
        raise ValueError
    # Ports 1-1023 represent "Well-known ports"
    # Ports 7400 and up are ports that might possibly be used by DDS/RTPS implementations
    # ROS_DOMAIN_ID in the context of a test is "guaranteed" to be unique (and in 0-255)
    if 'ROS_DOMAIN_ID' not in os.environ:
        port = get_open_port_any()
        if (bad_port(port)):
            print("Warning, port {} might collide with DDS or well-known ports".format(port))
        return port
    domain_id = int(os.environ['ROS_DOMAIN_ID'])
    return domain_id + range_start
