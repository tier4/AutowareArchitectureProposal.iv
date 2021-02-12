cmake_minimum_required(VERSION 3.5)
project(autoware_debug_tools)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR "src/")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ignore python
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pyflake_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(PROGRAMS src/stop_reason2pose.py src/pose2tf.py src/tf2pose.py
                 src/case_converter.py src/self_pose_listener.py
                 scripts/stop_reason2tf DESTINATION lib/${PROJECT_NAME})

install(FILES DESTINATION share/${PROJECT_NAME})
