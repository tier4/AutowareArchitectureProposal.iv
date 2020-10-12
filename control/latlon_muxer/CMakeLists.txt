cmake_minimum_required(VERSION 3.5)
project(latlon_muxer)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_control_msgs REQUIRED)

add_executable(latlon_muxer src/node.cpp src/main.cpp)

set(LATLON_MUXER_DEPENDENCIES
  autoware_control_msgs
  rclcpp
)

ament_target_dependencies(latlon_muxer ${LATLON_MUXER_DEPENDENCIES})
target_include_directories(latlon_muxer PRIVATE include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS latlon_muxer
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include)

ament_package()
