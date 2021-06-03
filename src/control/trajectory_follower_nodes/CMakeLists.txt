# Copyright 2021 The Autoware Foundation
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

cmake_minimum_required(VERSION 3.5)
project(trajectory_follower_nodes)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(LATLON_MUXER_NODE latlon_muxer_node)
ament_auto_add_library(${LATLON_MUXER_NODE} SHARED
  include/trajectory_follower_nodes/latlon_muxer_node.hpp
  src/latlon_muxer_node.cpp
)

autoware_set_compile_options(${LATLON_MUXER_NODE})
# TODO(latlon_muxer) : RCLCPP_ERROR_THROTTLE() has built-in old-style casts.
# TODO(latlon_muxer) : Temporary workaround until this is fixed.
target_compile_options(${LATLON_MUXER_NODE} PRIVATE -Wno-error=old-style-cast)
rclcpp_components_register_node(${LATLON_MUXER_NODE} PLUGIN "autoware::motion::control::latlon_muxer::LatLonMuxer" EXECUTABLE ${LATLON_MUXER_NODE}_exe)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # Unit tests
  set(LATLON_MUXER_TEST test_latlon_muxer_node)
  ament_add_gtest(${LATLON_MUXER_TEST} test/test_latlon_muxer_node.cpp)
  target_compile_options(${LATLON_MUXER_TEST} PRIVATE -Wno-error=old-style-cast)
  autoware_set_compile_options(${LATLON_MUXER_TEST})
  target_link_libraries(${LATLON_MUXER_TEST} ${LATLON_MUXER_NODE})

  add_ros_test(
    test/test_latlon_muxer_node_launch.py
    TIMEOUT "30"
  )

endif()

ament_auto_package(
  INSTALL_TO_SHARE
  param
)
