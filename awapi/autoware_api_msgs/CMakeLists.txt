cmake_minimum_required(VERSION 3.0.2)
project(autoware_api_msgs)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  diagnostic_msgs
  autoware_planning_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    Latlon.msg
    Euler.msg
    AwapiVehicleStatus.msg
    AwapiAutowareStatus.msg
    LaneChangeStatus.msg
    ObstacleAvoidanceStatus.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    diagnostic_msgs
    autoware_planning_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    diagnostic_msgs
    autoware_planning_msgs
)
