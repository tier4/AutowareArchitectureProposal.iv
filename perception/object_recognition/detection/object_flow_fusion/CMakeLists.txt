cmake_minimum_required(VERSION 2.8.3)
project(object_flow_fusion)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf
  autoware_perception_msgs
  eigen_conversions
  )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_definitions(-std=c++14)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  LIBRARIES ${PROJECT_NAME}
)

add_executable(object_flow_fusion_node
  src/main.cpp
  src/node.cpp
  src/object_flow_fusion.cpp
  src/utils.cpp
  )
target_link_libraries(object_flow_fusion_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

install(
  TARGETS
  object_flow_fusion_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME}/
  )
