cmake_minimum_required(VERSION 3.0.2)
project(kalman_filter)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    kalman_filter
    time_delay_kalman_filter
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(kalman_filter src/kalman_filter.cpp)
add_library(time_delay_kalman_filter src/time_delay_kalman_filter.cpp)

target_link_libraries(kalman_filter
  ${catkin_LIBRARIES}
)
target_link_libraries(time_delay_kalman_filter
  ${catkin_LIBRARIES}
)

add_dependencies(kalman_filter
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(time_delay_kalman_filter
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS
    kalman_filter time_delay_kalman_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
