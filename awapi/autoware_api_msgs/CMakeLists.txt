cmake_minimum_required(VERSION 3.0.2)
project(autoware_api_msgs)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geographic_msgs
  geometry_msgs
  diagnostic_msgs
  autoware_planning_msgs
  autoware_system_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    Euler.msg
    AwapiVehicleStatus.msg
    AwapiAutowareStatus.msg
    LaneChangeStatus.msg
    ObstacleAvoidanceStatus.msg
    DoorStatus.msg
    CrosswalkStatus.msg
    IntersectionStatus.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geographic_msgs
    geometry_msgs
    diagnostic_msgs
    autoware_planning_msgs
    autoware_system_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geographic_msgs
    geometry_msgs
    diagnostic_msgs
    autoware_planning_msgs
    autoware_system_msgs
)
