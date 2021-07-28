cmake_minimum_required(VERSION 3.5)
project(lane_change_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  add_compile_options(-Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

ament_auto_add_library(lane_change_planner_node SHARED
  src/lane_changer.cpp
  src/data_manager.cpp
  src/route_handler.cpp
  src/state_machine.cpp
  src/utilities.cpp
  src/state/following_lane.cpp
  src/state/forcing_lane_change.cpp
  src/state/executing_lane_change.cpp
  src/state/aborting_lane_change.cpp
  src/state/state_base_class.cpp
  src/state/blocked_by_obstacle.cpp
  src/state/stopping_lane_change.cpp
  src/state/common_functions.cpp
)

target_link_libraries(lane_change_planner_node
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(lane_change_planner_node
  PLUGIN "lane_change_planner::LaneChanger"
  EXECUTABLE lane_change_planner
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
