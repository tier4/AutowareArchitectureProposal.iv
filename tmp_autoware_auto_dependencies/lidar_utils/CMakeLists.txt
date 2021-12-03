# Copyright 2017-2018 Apex.AI, Inc.
# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(lidar_utils)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# Disable warnings due to external dependencies (Eigen)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/lidar_utils/point_cloud_utils.hpp
  include/lidar_utils/lidar_utils.hpp
  src/point_cloud_utils.cpp)

autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_lidar_utils
    test/src/test_fast_atan2.cpp
    test/src/test_point_cloud_utils.cpp
    test/src/test_cluster_view.cpp
  )
  autoware_set_compile_options(test_lidar_utils)
  target_include_directories(test_lidar_utils
    PRIVATE "include"
  )
  ament_target_dependencies(test_lidar_utils
    "autoware_auto_common"
    "autoware_auto_perception_msgs"
    "autoware_auto_geometry"
  )
  target_link_libraries(test_lidar_utils ${PROJECT_NAME})

  ament_add_google_benchmark(bench_point_cloud test/bench/bench_point_cloud.cpp)
  target_link_libraries(bench_point_cloud ${PROJECT_NAME})

endif()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
  "lidar_utils-extras.cmake"
)
ament_auto_package()
