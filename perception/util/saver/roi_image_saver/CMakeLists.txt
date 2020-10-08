cmake_minimum_required(VERSION 3.0.2)
project(roi_image_saver)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  autoware_perception_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
)

catkin_package(INCLUDE_DIRS include
  CATKIN_DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(traffic_light_roi_image_saver_node
  src/node.cpp
  src/main.cpp
)

add_dependencies(traffic_light_roi_image_saver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(traffic_light_roi_image_saver_node ${catkin_LIBRARIES})

install(TARGETS traffic_light_roi_image_saver_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

