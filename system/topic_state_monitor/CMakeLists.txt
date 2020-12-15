cmake_minimum_required(VERSION 3.0.2)
project(topic_state_monitor)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  dynamic_reconfigure
  roscpp
  topic_tools
)

generate_dynamic_reconfigure_options(
  cfg/TopicStateMonitor.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    diagnostic_updater
    dynamic_reconfigure
    topic_tools
  LIBRARIES
    topic_state_monitor
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Target
## topic_state_monitor
add_library(topic_state_monitor
  src/topic_state_monitor/topic_state_monitor.cpp
)

target_link_libraries(topic_state_monitor
  ${catkin_LIBRARIES}
)

add_dependencies(topic_state_monitor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## topic_state_monitor_node
add_executable(topic_state_monitor_node
  src/topic_state_monitor_node/main.cpp
  src/topic_state_monitor_node/topic_state_monitor_node.cpp
)

target_link_libraries(topic_state_monitor_node
  topic_state_monitor
  ${catkin_LIBRARIES}
)

add_dependencies(topic_state_monitor_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

# Install
## executables and libraries
install(
  TARGETS
    topic_state_monitor
    topic_state_monitor_node
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
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
