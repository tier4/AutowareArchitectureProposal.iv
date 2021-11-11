# Copyright 2021 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(behavior_velocity_planner_nodes)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(BEHAVIOR_VELOCITY_PLANNER_NODE_SRC
  src/scene_module/blind_spot/debug.cpp
  src/scene_module/blind_spot/manager.cpp
  src/scene_module/blind_spot/scene.cpp

  src/scene_module/crosswalk/debug.cpp
  src/scene_module/crosswalk/manager.cpp
  src/scene_module/crosswalk/scene_crosswalk.cpp
  src/scene_module/crosswalk/scene_walkway.cpp
  src/scene_module/crosswalk/util.cpp

  src/scene_module/detection_area/debug.cpp
  src/scene_module/detection_area/manager.cpp
  src/scene_module/detection_area/scene.cpp

  src/scene_module/intersection/debug.cpp
  src/scene_module/intersection/manager.cpp
  src/scene_module/intersection/scene_intersection.cpp
  src/scene_module/intersection/scene_merge_from_private_road.cpp
  src/scene_module/intersection/util.cpp

  src/scene_module/stop_line/debug.cpp
  src/scene_module/stop_line/manager.cpp
  src/scene_module/stop_line/scene.cpp

  src/utilization/interpolate.cpp
  src/utilization/path_utilization.cpp
  src/utilization/util.cpp

  src/utilization/interpolation/spline_interpolation.cpp

  src/behavior_velocity_planner_node.cpp
  src/planner_data.cpp
  src/planner_manager.cpp
)

set(BEHAVIOR_VELOCITY_PLANNER_NODE_HEADERS
  include/behavior_velocity_planner_nodes/visibility_control.hpp
  include/behavior_velocity_planner_nodes/behavior_velocity_planner_node.hpp
  include/behavior_velocity_planner_nodes/planner_data.hpp
  include/behavior_velocity_planner_nodes/planner_manager.hpp

  include/scene_module/blind_spot/manager.hpp
  include/scene_module/blind_spot/scene.hpp

  include/scene_module/crosswalk/manager.hpp
  include/scene_module/crosswalk/scene_crosswalk.hpp
  include/scene_module/crosswalk/scene_walkway.hpp
  include/scene_module/crosswalk/util.hpp

  include/scene_module/detection_area/manager.hpp
  include/scene_module/detection_area/scene.hpp

  include/scene_module/intersection/manager.hpp
  include/scene_module/intersection/scene_intersection.hpp
  include/scene_module/intersection/scene_merge_from_private_road.hpp
  include/scene_module/intersection/util.hpp

  include/scene_module/stop_line/manager.hpp
  include/scene_module/stop_line/scene.hpp

  include/scene_module/scene_module_interface.hpp

  include/utilization/interpolation/cubic_spline.hpp
  include/utilization/interpolation/spline_interpolation.hpp
  include/utilization/boost_geometry_helper.hpp
  include/utilization/interpolate.hpp
  include/utilization/marker_helper.hpp
  include/utilization/path_utilization.hpp
  include/utilization/util.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${BEHAVIOR_VELOCITY_PLANNER_NODE_SRC}
  ${BEHAVIOR_VELOCITY_PLANNER_NODE_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-old-style-cast )

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::planning::behavior_velocity_planner_nodes::BehaviorVelocityPlannerNode"
  EXECUTABLE ${PROJECT_NAME}_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_behavior_velocity_planner_node.cpp)
  set(TEST_BEHAVIOR_VELOCITY_PLANNER_NODE_EXE test_behavior_velocity_planner_node)
  ament_add_gtest(${TEST_BEHAVIOR_VELOCITY_PLANNER_NODE_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_BEHAVIOR_VELOCITY_PLANNER_NODE_EXE})
  target_link_libraries(${TEST_BEHAVIOR_VELOCITY_PLANNER_NODE_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
