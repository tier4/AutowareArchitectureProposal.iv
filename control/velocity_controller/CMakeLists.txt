cmake_minimum_required(VERSION 3.5)
project(velocity_controller)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(velocity_controller_SRC
  src/velocity_controller_core.cpp
  src/pid.cpp
  src/velocity_controller_utils.cpp
)

ament_auto_add_library(velocity_controller_core SHARED
  ${velocity_controller_SRC}
)

rclcpp_components_register_node(velocity_controller_core
  PLUGIN "VelocityController"
  EXECUTABLE velocity_controller
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES
    test/test_debug_values.cpp
    test/test_pid.cpp
    test/test_smooth_stop.cpp
    test/test_velocity_controller.cpp
    test/test_velocity_controller_utils.cpp
  )
  set(TEST_VELOCITY_CONTROLLER test_velocity_controller)
  ament_add_gtest(${TEST_VELOCITY_CONTROLLER} ${TEST_SOURCES})
  target_link_libraries(${TEST_VELOCITY_CONTROLLER} velocity_controller_core)

endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
