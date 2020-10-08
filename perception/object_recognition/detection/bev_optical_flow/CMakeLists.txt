cmake_minimum_required(VERSION 2.8.3)
project(bev_optical_flow)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
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

add_executable(optical_flow_node
  src/main.cpp
  src/node.cpp
  src/lidar_to_image.cpp
  src/flow_calculator.cpp
  src/utils.cpp
  src/debugger.cpp
  )
target_link_libraries(optical_flow_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

install(
  TARGETS
  optical_flow_node
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
