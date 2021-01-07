cmake_minimum_required(VERSION 3.0.2)
project(autoware_localization_rviz_plugin)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  geometry_msgs
)

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  CATKIN_DEPENDS roscpp rviz
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${Qt5_INCLUDE_DIRS}
)

add_library(autoware_localization_rviz_plugin
  src/pose_history/pose_history_display.hpp
  src/pose_history/pose_history_display.cpp
)

target_link_libraries(autoware_localization_rviz_plugin
  ${catkin_LIBRARIES} ${QT_LIBRARIES}
)

install(TARGETS autoware_localization_rviz_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY
    icons
    plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
