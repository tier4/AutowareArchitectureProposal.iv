# Copyright 2019 Apex.AI, Inc.
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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
cmake_minimum_required(VERSION 3.5)

### Export headers
project(autoware_auto_geometry)

## dependencies
find_package(ament_cmake REQUIRED)
find_package(autoware_auto_common REQUIRED)
find_package(autoware_auto_msgs REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)

# includes
add_library(${PROJECT_NAME} SHARED
  include/geometry/spatial_hash.hpp
  include/geometry/spatial_hash_config.hpp
  include/geometry/vehicle_bounding_box.hpp
  src/spatial_hash.cpp
  src/bounding_box.cpp
  src/vehicle_bounding_box.cpp)
ament_target_dependencies(${PROJECT_NAME}
  "autoware_auto_common"
  "autoware_auto_msgs"
  "geometry_msgs"
  "lidar_utils")

target_include_directories(${PROJECT_NAME} PUBLIC include)
autoware_set_compile_options(${PROJECT_NAME})

# static allocators from containers package need this
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wno-double-promotion)

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()
  # gtest
  find_package(geometry_msgs REQUIRED)
  find_package(ament_cmake_gtest)
  set(GEOMETRY_GTEST geometry_gtest)
  set(GEOMETRY_SRC test/src/test_geometry.cpp
    test/src/test_convex_hull.cpp
  )
  ament_add_gtest(${GEOMETRY_GTEST} ${GEOMETRY_SRC})
  target_include_directories(${GEOMETRY_GTEST} PRIVATE "test/include" "include")
  ament_target_dependencies(${GEOMETRY_GTEST}
    "autoware_auto_common"
    "autoware_auto_msgs"
    "geometry_msgs"
    "osrf_testing_tools_cpp")
  target_link_libraries(${GEOMETRY_GTEST} ${PROJECT_NAME})
endif()

# Ament Exporting
autoware_install(LIBRARIES ${PROJECT_NAME} HAS_INCLUDE)
ament_export_dependencies(
  "autoware_auto_common"
  "autoware_auto_msgs"
  "geometry_msgs"
)
ament_package()
