cmake_minimum_required(VERSION 3.5)
project(pose2twist)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(pose2twist
  src/pose2twist_node.cpp
  src/pose2twist_core.cpp
)

ament_target_dependencies(pose2twist rclcpp 
  tf2_geometry_msgs 
  std_msgs geometry_msgs
)

install(
  TARGETS
  pose2twist
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(geometry_msgs)
ament_package()
