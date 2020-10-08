cmake_minimum_required(VERSION 3.0.2)
project(motion_velocity_optimizer)

add_compile_options(-std=c++14 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  autoware_planning_msgs
  geometry_msgs
  tf2
  tf2_ros
  dynamic_reconfigure
  osqp_interface
)

generate_dynamic_reconfigure_options(cfg/MotionVelocityOptimizer.cfg)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    autoware_planning_msgs
    geometry_msgs
    tf2
    tf2_ros
    osqp_interface
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


set(MOTION_VELOCITY_OPTIMIZER_SRC
  src/motion_velocity_optimizer_utils.cpp
  src/interpolate.cpp
  src/optimizer/pseudo_jerk_l2_optimizer.cpp
  src/optimizer/pseudo_jerk_linf_optimizer.cpp
)

add_executable(motion_velocity_optimizer src/motion_velocity_optimizer.cpp ${MOTION_VELOCITY_OPTIMIZER_SRC})
add_dependencies(motion_velocity_optimizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} motion_velocity_optimizer_gencfg)
target_link_libraries(motion_velocity_optimizer ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})

install(
  TARGETS
    motion_velocity_optimizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY
    scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
