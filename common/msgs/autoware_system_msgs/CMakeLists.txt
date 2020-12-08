cmake_minimum_required(VERSION 3.0.2)
project(autoware_system_msgs)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  std_msgs
  message_generation
)

add_message_files(FILES
  AutowareState.msg
  DrivingCapability.msg
  HazardStatus.msg
  HazardStatusStamped.msg
)

generate_messages(DEPENDENCIES
  diagnostic_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
)
