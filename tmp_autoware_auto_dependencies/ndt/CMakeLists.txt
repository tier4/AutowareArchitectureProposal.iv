# Copyright 2019-2020 Apex.AI, Inc., Arm Limited
# All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(autoware_auto_ndt)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
#dependencies
find_package(Eigen3 REQUIRED)
find_package(GEOGRAPHICLIB REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS io)
ament_auto_find_build_dependencies()

# includes
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} ${YAML_CPP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

set(NDT_NODES_LIB_SRC
    src/ndt.cpp
    src/ndt_map.cpp
    src/ndt_map_publisher.cpp
    src/ndt_voxel.cpp
    src/ndt_voxel_view.cpp
)

set(NDT_NODES_LIB_HEADERS
    include/ndt/visibility_control.hpp
    include/ndt/ndt_config.hpp
    include/ndt/ndt_common.hpp
    include/ndt/ndt_optimization_problem.hpp
    include/ndt/ndt_voxel.hpp
    include/ndt/ndt_voxel_view.hpp
    include/ndt/ndt_map.hpp
    include/ndt/ndt_map_publisher.hpp
    include/ndt/ndt_scan.hpp
    include/ndt/ndt_localizer.hpp
    include/ndt/utils.hpp)

ament_auto_add_library(${PROJECT_NAME} SHARED ${NDT_NODES_LIB_SRC})
autoware_set_compile_options(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
  ${GeographicLib_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${PCL_LIBRARIES})

# TODO(yunus.caliskan): Remove once #978 is fixed.
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-Og")
else()
  set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-O3 -ftree-vectorize")
endif()

# Required for point_cloud_msg_wrapper
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-conversion -Wno-sign-conversion)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(PCL 1.8 REQUIRED COMPONENTS registration)
  include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

  # gtest
  set(NDT_TEST ndt_gtest)

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${NDT_TEST}
          test/test_ndt_utils.hpp
#           test/test_ndt_utils.cpp
          test/test_ndt_map.hpp
          test/test_ndt_scan.hpp
          test/test_ndt_map.cpp
          test/test_ndt_scan.cpp
          test/test_ndt_optimization.hpp
#          test/test_ndt_optimization.cpp
#          test/test_ndt_localizer.cpp
          )

  target_link_libraries(${NDT_TEST} ${PROJECT_NAME} ${PCL_LIBRARIES})
  autoware_set_compile_options(${NDT_TEST})
  target_compile_options(${NDT_TEST} PRIVATE -Wno-conversion -Wno-float-conversion -Wno-double-promotion)
endif()

ament_export_include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
ament_auto_package()
