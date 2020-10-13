cmake_minimum_required(VERSION 3.5)
project(velocity_controller)

add_compile_options(-std=c++14)


### Dependencies
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)

set(VELOCITY_CONTROLLER_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  autoware_planning_msgs
  autoware_control_msgs
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  velocity_controller
  src/velocity_controller_core.cpp
  src/velocity_controller_node.cpp
  src/pid.cpp
  src/velocity_controller_mathutils.cpp
)

ament_target_dependencies(velocity_controller ${VELOCITY_CONTROLLER_DEPENDENCIES})


#############
## Install ##
#############

install(
  TARGETS velocity_controller
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
