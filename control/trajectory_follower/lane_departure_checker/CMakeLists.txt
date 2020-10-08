cmake_minimum_required(VERSION 3.0.2)
project(lane_departure_checker)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  autoware_utils
  diagnostic_updater
  dynamic_reconfigure
  geometry_msgs
  lanelet2_extension
  roscpp
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/LaneDepartureChecker.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    autoware_planning_msgs
    autoware_utils
    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    lanelet2_extension
    std_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Target
## lane_departure_checker_node
add_executable(lane_departure_checker_node
  src/lane_departure_checker_node/main.cpp
  src/lane_departure_checker_node/lane_departure_checker.cpp
  src/lane_departure_checker_node/lane_departure_checker_node.cpp
)

target_link_libraries(lane_departure_checker_node
  ${Boost_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(lane_departure_checker_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

# Install
## executables and libraries
install(
  TARGETS
    lane_departure_checker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## directories
install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
