cmake_minimum_required(VERSION 3.0.2)
project(object_range_splitter)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  roscpp
  sensor_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(object_range_splitter_node
  src/node.cpp
  src/main.cpp
)

add_dependencies(object_range_splitter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(object_range_splitter_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS
    object_range_splitter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
