cmake_minimum_required(VERSION 3.0.2)
project(goal_distance_calculator)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  autoware_utils
  geometry_msgs
  roscpp
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    autoware_planning_msgs
    autoware_utils
    geometry_msgs
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
## goal_distance_calculator_node
add_executable(goal_distance_calculator_node
  src/goal_distance_calculator_node/main.cpp
  src/goal_distance_calculator_node/goal_distance_calculator.cpp
  src/goal_distance_calculator_node/goal_distance_calculator_node.cpp
)

target_link_libraries(goal_distance_calculator_node
  ${Boost_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(goal_distance_calculator_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Install
## executables and libraries
install(
  TARGETS
    goal_distance_calculator_node
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
    scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
