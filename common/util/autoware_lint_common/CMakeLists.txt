cmake_minimum_required(VERSION 3.5)

project(autoware_lint_common NONE)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_export_dependencies REQUIRED)
find_package(ament_cmake_test REQUIRED)

ament_package_xml()
ament_export_dependencies(${${PROJECT_NAME}_EXEC_DEPENDS})
ament_package(
  CONFIG_EXTRAS "autoware_lint_common-extras.cmake"
)

install(
  DIRECTORY configuration
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()

  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
endif()
