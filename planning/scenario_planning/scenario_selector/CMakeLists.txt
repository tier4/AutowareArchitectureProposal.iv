cmake_minimum_required(VERSION 3.5)
project(scenario_selector)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

## Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Target

## Target executable
set(SCENARIO_SELECTOR_SRC
  src/scenario_selector_node/scenario_selector_node.cpp
)

## scenario_selector_node
ament_auto_add_executable(scenario_selector
  src/scenario_selector_node/main.cpp
  ${SCENARIO_SELECTOR_SRC}
)

# target_link_libraries(scenario_selector
#   ${catkin_LIBRARIES}
# )

# add_dependencies(scenario_selector
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

# # Install
# ## executables and libraries
# install(
#   TARGETS
#     scenario_selector
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ## project namespaced headers
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# ## launch files
# install(
#   DIRECTORY
#     launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

ament_auto_package(INSTALL_TO_SHARE
  launch
)
