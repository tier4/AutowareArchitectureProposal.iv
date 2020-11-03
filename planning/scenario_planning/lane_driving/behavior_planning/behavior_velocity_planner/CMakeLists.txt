cmake_minimum_required(VERSION 3.5)
project(behavior_velocity_planner)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lanelet2_extension REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(spline_interpolation REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vehicle_info_util REQUIRED)
find_package(visualization_msgs REQUIRED)

set(BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES
  ament_cmake
  autoware_perception_msgs
  autoware_planning_msgs
  Eigen3
  diagnostic_msgs
  geometry_msgs
  lanelet2_extension
  PCL
  pcl_conversions
  rclcpp
  sensor_msgs
  spline_interpolation
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  vehicle_info_util
  visualization_msgs
)


# Common
add_library(scene_module_lib STATIC
  src/utilization/path_utilization.cpp
  src/utilization/util.cpp
  src/utilization/interpolate.cpp
)

target_include_directories(scene_module_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_lib ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})


# Scene Module: Stop Line
add_library(scene_module_stop_line STATIC
  src/scene_module/stop_line/manager.cpp
  src/scene_module/stop_line/scene.cpp
  src/scene_module/stop_line/debug.cpp
)

target_include_directories(scene_module_stop_line
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_stop_line ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_stop_line scene_module_lib)


# Scene Module: Crosswalk
add_library(scene_module_crosswalk STATIC
  src/scene_module/crosswalk/manager.cpp
  src/scene_module/crosswalk/scene_crosswalk.cpp
  src/scene_module/crosswalk/scene_walkway.cpp
  src/scene_module/crosswalk/debug.cpp
  src/scene_module/crosswalk/util.cpp
)

target_include_directories(scene_module_crosswalk
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_crosswalk ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_crosswalk scene_module_lib)


# Scene Module: Intersection
add_library(scene_module_intersection STATIC
  src/scene_module/intersection/manager.cpp
  src/scene_module/intersection/scene_intersection.cpp
  src/scene_module/intersection/scene_merge_from_private_road.cpp
  src/scene_module/intersection/debug.cpp
  src/scene_module/intersection/util.cpp
)

target_include_directories(scene_module_intersection
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_intersection ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_intersection scene_module_lib)


# Scene Module: Traffic Light
add_library(scene_module_traffic_light STATIC
  src/scene_module/traffic_light/manager.cpp
  src/scene_module/traffic_light/scene.cpp
  src/scene_module/traffic_light/debug.cpp
)

target_include_directories(scene_module_traffic_light
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_traffic_light ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_traffic_light scene_module_lib)


# Scene Module: Blind Spot
add_library(scene_module_blind_spot STATIC
  src/scene_module/blind_spot/manager.cpp
  src/scene_module/blind_spot/scene.cpp
  src/scene_module/blind_spot/debug.cpp
)

target_include_directories(scene_module_blind_spot
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_blind_spot ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_blind_spot scene_module_lib)


# Scene Module: Detection Area
add_library(scene_module_detection_area STATIC
  src/scene_module/detection_area/manager.cpp
  src/scene_module/detection_area/scene.cpp
  src/scene_module/detection_area/debug.cpp
)

target_include_directories(scene_module_detection_area
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_detection_area ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_detection_area scene_module_lib)


# Scene Module Manager
add_library(scene_module_manager STATIC
  src/planner_manager.cpp
)

target_include_directories(scene_module_manager
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_manager ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_manager scene_module_lib)


# Node
add_executable(behavior_velocity_planner_node
  src/node.cpp
  src/main.cpp
)

target_include_directories(behavior_velocity_planner_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(behavior_velocity_planner_node ${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES} )

target_link_libraries(behavior_velocity_planner_node
  scene_module_lib
  scene_module_stop_line
  scene_module_crosswalk
  scene_module_intersection
  scene_module_traffic_light
  scene_module_blind_spot
  scene_module_detection_area
  scene_module_manager
)

ament_export_dependencies(${BEHAVIOR_VELOCITY_PLANNER_DEPENDENCIES})
ament_package()
